/*
  Hand heater controller for ESP32.

  ESP32 board profile:
  Go into Preferences -> Additional Board Manager URLs
  Add:
   xxxx ADD URL HERE

  Then go to Tools -> Board -> Board Manager, update, and add "ESP32"

  Arduino IDE settings:
  Board: ""
  Frequency: "240MHz"

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
#define STATUS_LED_PIN   13 //25
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

unsigned long previousMillis = 0;  
int interval = 0;  // LED flash interval in milliseconds
int led_state = LOW;   

uint8_t heater_state = STATE_OFF;
byte last_button_state = 0;
uint16_t last_button_press = 0;

#define FAN_FREQUENCY     15000
#define FAN_CHANNEL           0
#define FAN_RESOLUTION        8
#define HEATER_FREQUENCY   5000
#define HEATER_CHANNEL        1
#define HEATER_RESOLUTION     8


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
  Serial.println("Bluetooth has started, now you can pair with the device called 'Heater'");
  
  ledcSetup(FAN_CHANNEL, FAN_FREQUENCY, FAN_RESOLUTION);
  ledcAttachPin(FAN_PIN, FAN_CHANNEL);

  ledcSetup(HEATER_CHANNEL, HEATER_FREQUENCY, HEATER_RESOLUTION);
  ledcAttachPin(HEATER_PIN, HEATER_CHANNEL);
  
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(TEMP_SENSOR_PIN, INPUT);

  digitalWrite(STATUS_LED_PIN, LOW);
  
  ledcWrite(FAN_CHANNEL,    0);
  ledcWrite(HEATER_CHANNEL, 0);
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

  // Check the temperature sensor for an over-temp condition. Do this
  // after all other checks so that it will override commands from other
  // sources
  check_temperature();

  // Main state machine
  switch(heater_state){
    case 0:
      interval = 0;
      ledcWrite(HEATER_PIN, 0);
      ledcWrite(FAN_PIN, 0);
      break;
    case 1:
      interval = 800;
      ledcWrite(HEATER_PIN, 0);
      ledcWrite(FAN_PIN, 80);
      break;
    case 2:
      interval = 200;
      ledcWrite(HEATER_PIN, 170);
      ledcWrite(FAN_PIN, 130);
      break;
    case 3:
      digitalWrite(STATUS_LED_PIN, HIGH);
      ledcWrite(HEATER_PIN, 255);
      ledcWrite(FAN_PIN, 200);
      break;
  }
}


/**
 * See if the button has been pressed, and apply de-bouncing
 */
void check_button()
{
  byte button_state = digitalRead(BUTTON_PIN);
  if((button_state == HIGH) && (last_button_state == LOW) && (millis() - last_button_press > 200))
  {
    // We've detected a button press. Change state.
    heater_state++;
    if(heater_state >= STATE_HOT) // If we've hit the end state, loop back to the start
    {
      heater_state = 0;
    }
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
  if ((millis() - previousMillis >= interval) && (interval >0)) {
    // save the last time you blinked the LED
    previousMillis = millis();
    // if the LED is off turn it on and vice-versa:
    led_state = !led_state;
    digitalWrite(STATUS_LED_PIN, led_state);
  } else if(interval == 0) {
    digitalWrite(STATUS_LED_PIN, LOW);
  }
}


/**
 * Check whether we've exceeded the temperature limit and shut down if necessary
 */
void check_temperature()
{
  int current_temperature = getTemp();

  if(current_temperature > TEMPERATURE_SAFE_LIMIT)
  {
    heater_state = STATE_OFF;
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
