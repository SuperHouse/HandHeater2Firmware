/*
  Hand heater controller for ESP32
  Copyright 2019 SuperHouse Automation Pty Ltd
  Author: Jonathan Oxer <jon@oxer.com.au>

  ESP32 board profile:
  Go into Preferences -> Additional Board Manager URLs
  Add:
   xxxx ADD URL HERE

  Then go to Tools -> Board -> Boards Manager..., update, and add "esp32 by Espressif Systems"

  Arduino IDE settings:
  Board:            ESP32 Dev Board
  Upload speed:     115200bps
  CPU Speed:        240MHz
  Flash Frequency:  40MHz
  Flash Mode:       DOUT
  Flash Size:       4MB
  Partition Scheme: Default
  Core Debug Level: None
  PSRAM:            Disabled

  ﻿GPIO5: CAN TX
  GPIO4: CAN RX
  GPIO14: Button
  GPIO16: Heater PWM
  GPIO17: Fan PWM
  GPIO25: Status LED
  GPIO32: Temp Sensor
  GPIO33: Fan tacho
*/

#include "BluetoothSerial.h"

// Pin definitions
#define CAN_TX            5
#define CAN_RX            4
#define BUTTON_PIN       14
#define HEATER_PIN       16  // PWM
#define FAN_PIN          17  // PWM
#define STATUS_LED_PIN   25
#define TEMP_SENSOR_PIN  32
#define FAN_TACHO_PIN    33

// States
#define STATE_OFF   0
#define STATE_COLD  1
#define STATE_WARM  2
#define STATE_HOT   3

// Thermistor beta coefficient
#define BCOEFFICIENT 3380

// Maximum safe output temperature in Celsius
#define TEMPERATURE_SAFE_LIMIT 50

// How often to check the fan tacho, in seconds
#define FAN_CHECK_INTERVAL  1
uint16_t fan_last_checked = 0;

// How often to check the temperature, in seconds
#define TEMPERATURE_CHECK_INTERVAL  1
uint16_t temperature_last_checked = 0;

unsigned long previousMillis = 0;  
int led_blink_interval = 0;  // LED flash interval in milliseconds (0 is always on)
int led_state = LOW;   

uint8_t  heater_state      = STATE_OFF;
uint8_t  last_button_state = 0;
uint16_t last_button_press = 0;
uint16_t fan_pulses        = 0;
uint16_t fan_speed         = 0;

#define FAN_FREQUENCY     15000   // PWM frequency for fan control
#define FAN_CHANNEL           0   // PWM output channel for fan
#define FAN_RESOLUTION        8   // PWM resolution for fan
#define HEATER_FREQUENCY   5000   // PWM frequency for heater control
#define HEATER_CHANNEL        1   // PWM output channel for heater
#define HEATER_RESOLUTION     8   // PWM resolution for heater

// Make sure Bluetooth is available in the ESP32 core
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

/**
 * Setup
 */
void setup() {
  Serial.begin(9600);
  Serial.println("Hand heater 2 starting up");

  SerialBT.begin("Heater"); //Bluetooth device name
  Serial.println("Bluetooth has started. Pair with the device called 'Heater'");
  
  pinMode(HEATER_PIN,      OUTPUT);
  pinMode(FAN_PIN,         OUTPUT);
  pinMode(BUTTON_PIN,      INPUT_PULLUP);
  pinMode(STATUS_LED_PIN,  OUTPUT);
  pinMode(TEMP_SENSOR_PIN, INPUT);

  // Use the LED driving timers for fan and heater output
  ledcSetup(FAN_CHANNEL, FAN_FREQUENCY, FAN_RESOLUTION);
  ledcAttachPin(FAN_PIN, FAN_CHANNEL);
  ledcWrite(FAN_CHANNEL, 0);

  ledcSetup(HEATER_CHANNEL, HEATER_FREQUENCY, HEATER_RESOLUTION);
  ledcAttachPin(HEATER_PIN, HEATER_CHANNEL);
  ledcWrite(HEATER_CHANNEL, 0);

  // Start with the LED off
  digitalWrite(STATUS_LED_PIN, LOW);

  // Detect pulses from the fan tacho using an interrupt
  pinMode(FAN_TACHO_PIN, INPUT_PULLUP);
  attachInterrupt(FAN_TACHO_PIN, fan_tacho_isr, FALLING);
}

/**
 * Called when a pulse is detected from the fan tacho
 */
void fan_tacho_isr()
{
  fan_pulses++;
}

/**
 * Loop
 */
void loop() {
  // Check if the button has been pressed
  check_button();

  // Check if a command has been sent via the regular serial port
  check_usb_serial();

  // Check if a command has been sent via the Bluetooth serial port
  check_bt_serial();

  // Flash the status LED
  led_flasher();

  // Process accumulated fan tacho pulses to determine the speed
  process_fan_tacho();

  // Check the temperature sensor for an over-temp condition. Do this
  // after all other checks so that it will override commands from other
  // sources
  check_temperature();

  // Main state machine
  //Serial.print("State: ");
  //Serial.println(heater_state);
  switch(heater_state){
    case STATE_OFF:
      led_blink_interval = 0;
      ledcWrite(HEATER_CHANNEL, 0);
      ledcWrite(FAN_CHANNEL, 0);
      break;
    case STATE_COLD:
      led_blink_interval = 800;
      ledcWrite(HEATER_CHANNEL, 0);
      ledcWrite(FAN_CHANNEL, 80);
      break;
    case STATE_WARM:
      led_blink_interval = 200;
      ledcWrite(HEATER_CHANNEL, 100);
      ledcWrite(FAN_CHANNEL, 120);
      break;
    case STATE_HOT:
      digitalWrite(STATUS_LED_PIN, HIGH);
      ledcWrite(HEATER_CHANNEL, 150);
      ledcWrite(FAN_CHANNEL, 200);
      break;
  }
}


/**
 * Check how many pulses have been received from the fan tacho
 */
void process_fan_tacho()
{
  uint16_t time_now = millis();
  uint16_t fan_speed = 0;
  if((time_now - fan_last_checked) > (FAN_CHECK_INTERVAL * 1000))
  {
    //Serial.print(fan_pulses);
    //Serial.print("  ");
    fan_speed = fan_pulses * (60 / FAN_CHECK_INTERVAL);
    fan_pulses = 0;
    fan_last_checked = time_now;
    Serial.print("Fan speed: ");
    Serial.println(fan_speed);
  }
}

/**
 * See if the button has been pressed, and apply de-bouncing
 */
void check_button()
{
  byte button_state = digitalRead(BUTTON_PIN);
  //Serial.print(button_state);
  if((button_state == LOW) && (last_button_state == HIGH) && (millis() - last_button_press > 200))
  {
    // We've detected a button press. Change state.
    increment_heater_state();
    last_button_press = millis();
  }
  last_button_state = button_state;
}

/**
 * Make the output hotter. Wrap around from max back to min
 */
void increment_heater_state()
{
  heater_state++;
  if(heater_state > STATE_HOT) // If we've hit the end state, loop back to the start
  {
    heater_state = STATE_OFF;
  }
  Serial.print("Setting heater level to ");
  Serial.println(heater_state);
  SerialBT.print("Setting heater level to ");
  SerialBT.println(heater_state);
}

/**
 * Make the output colder. Do NOT wrap around from min back to max, for safety
 */
void decrement_heater_state()
{
  
  if(heater_state > STATE_OFF) // Only decrement while it's above the lowest setting
  {
    heater_state--;
  }
  Serial.print("Setting heater level to ");
  Serial.println(heater_state);
  SerialBT.print("Setting heater level to ");
  SerialBT.println(heater_state);
}


/**
 * See if there is a command from the USB serial port
 */
void check_usb_serial()
{
  if (Serial.available())
  {
    byte character = Serial.read();
    if(character == '+')
    {
      increment_heater_state();
    }
    if(character == '-')
    {
      decrement_heater_state();
    }
  }
}


/**
 * See if there is a command from the Bluetooth serial port
 */
void check_bt_serial()
{
  if (SerialBT.available())
  {
    byte character = SerialBT.read();
    if(character == '+')
    {
      increment_heater_state();
    }
    if(character == '-')
    {
      decrement_heater_state();
    }
  }
}

/**
 * Blink the LED at a set interval
 */
void led_flasher()
{
  if ((millis() - previousMillis >= led_blink_interval) && (led_blink_interval >0)) {
    // save the last time you blinked the LED
    previousMillis = millis();
    // if the LED is off turn it on and vice-versa:
    led_state = !led_state;
    digitalWrite(STATUS_LED_PIN, led_state);
  } else if(led_blink_interval == 0) {
    digitalWrite(STATUS_LED_PIN, LOW);
  }
}

/**
 * Check whether we've exceeded the temperature limit and shut down if necessary
 */
void check_temperature()
{
  if ((millis() - temperature_last_checked >= TEMPERATURE_CHECK_INTERVAL) && (TEMPERATURE_CHECK_INTERVAL >0)) {
    // save the last time we checked temperature
    temperature_last_checked = millis();
    int current_temperature = getTemp();
    Serial.print("Temperature: ");
    Serial.print(current_temperature);
    Serial.println("C");

    if(current_temperature > TEMPERATURE_SAFE_LIMIT)
    {
      heater_state = STATE_OFF;
    }
  }
}

/**
 * Read the temperature in C from the front thermistor
 * https://chrisholdt.com/2018/01/03/esp32-light-temperature-oled/
 */
int getTemp() {
  double thermalSamples[5];
  double average, kelvin, resistance, celsius;
  int i;
    
  // Collect SAMPLERATE (default 5) samples
  for (i=0; i<5; i++) {
    thermalSamples[i] = analogRead(TEMP_SENSOR_PIN);
    delay(10);
  }
    
  // Calculate the average value of the samples
  average = 0;
  for (i=0; i<5; i++) {
    average += thermalSamples[i];
  }
  average /= 5;

  // Convert to resistance
  resistance = 4095 / average - 1;
  resistance = 10000 / resistance;

  /*
   * Use Steinhart equation (simplified B parameter equation) to convert resistance to kelvin
   * B param eq: T = 1/( 1/To + 1/B * ln(R/Ro) )
   * T  = Temperature in Kelvin
   * R  = Resistance measured
   * Ro = Resistance at nominal temperature
   * B  = Coefficent of the thermistor
   * To = Nominal temperature in kelvin
   */
  kelvin = resistance/10000;            // R/Ro
  kelvin = log(kelvin);                 // ln(R/Ro)
  kelvin = (1.0/BCOEFFICIENT) * kelvin; // 1/B * ln(R/Ro)
  kelvin = (1.0/(25+273.15)) + kelvin;  // 1/To + 1/B * ln(R/Ro)
  kelvin = 1.0/kelvin;                  // 1/( 1/To + 1/B * ln(R/Ro) )
    
  // Convert Kelvin to Celsius
  celsius = kelvin - 273.15;
    
  // Send the value back to be displayed
  return celsius;
}
