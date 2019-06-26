/*
  Hand heater controller for ESP32
  Copyright 2019 SuperHouse Automation Pty Ltd
  Author: Jonathan Oxer <jon@oxer.com.au>

  Arduino IDE ESP32 board profile:
  Go into Preferences -> Additional Board Manager URLs
  Add:
   https://dl.espressif.com/dl/package_esp32_index.json

  Then go to Tools -> Board -> Boards Manager..., update, and add "esp32 by Espressif Systems"

  Arduino IDE settings under "Tools":
  Board:            ESP32 Dev Module
  Upload speed:     115200bps
  CPU Speed:        240MHz
  Flash Frequency:  40MHz
  Flash Mode:       DOUT
  Flash Size:       4MB
  Partition Scheme: Minimal SPIFFS (1.9MB APP with OTA/190KB SPIFFS)
  Core Debug Level: None
  PSRAM:            Disabled

  CAN bus library:
  Install library from https://github.com/nhatuan84/arduino-esp32-can-demo

  ﻿GPIO5: CAN TX
  GPIO4: CAN RX
  GPIO14: Button
  GPIO16: Heater PWM
  GPIO17: Fan PWM
  GPIO25: Status LED
  GPIO32: Temp Sensor
  GPIO33: Fan tacho
*/

/*    YOUR LOCAL CONFIGURATION   */
#include "config.h"

/* WiFi library */
#ifdef ENABLE_WIFI
  #include <WiFi.h>
  WiFiClient espClient;
#endif

/* Bluetooth library */
#ifdef ENABLE_BLUETOOTH
  #include "BluetoothSerial.h"
#endif

/* CAN bus library setup */
#ifdef ENABLE_CAN
  #include <ESP32CAN.h>
  #include <CAN_config.h>
  CAN_device_t CAN_cfg;  // The variable name CAN_cfg is fixed, do not change
  #define CAN_SPEED CAN_SPEED_1000KBPS
#endif

/* MQTT library */
#ifdef ENABLE_MQTT
  #include <PubSubClient.h>
  PubSubClient client(espClient);
  long lastMsg = 0;
  char msg[50];
  char command_topic[20];
  //String device_id = "default";
#endif


// Pin definitions
#define CAN_TX_PIN       GPIO_NUM_5
#define CAN_RX_PIN       GPIO_NUM_4
#define BUTTON_PIN       GPIO_NUM_14
#define HEATER_PIN       GPIO_NUM_16  // PWM
#define FAN_PIN          GPIO_NUM_17  // PWM
#define STATUS_LED_PIN   GPIO_NUM_25
#define TEMP_SENSOR_PIN  GPIO_NUM_32
#define FAN_TACHO_PIN    GPIO_NUM_33

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
uint32_t fan_last_checked = 0;

// How often to report the fan tacho, in seconds
#define FAN_SPEED_REPORT_INTERVAL  2
long fan_speed_last_reported = 0;

// How often to check the temperature, in seconds
#define TEMPERATURE_CHECK_INTERVAL  0.5
uint32_t temperature_last_checked = 0;

// How often to report the temperature, in seconds
#define TEMPERATURE_REPORT_INTERVAL  2
long temperature_last_reported = 0;

// LED flash interval in milliseconds. Updated in state machine
uint32_t led_blink_interval = 0;
uint32_t led_last_blinked   = 0;
uint8_t  led_state          = LOW;

uint8_t  heater_state        = STATE_OFF;
uint8_t  last_button_state   = 0;
uint32_t last_button_press   = 0;
uint32_t fan_pulses          = 0;
uint32_t fan_speed           = 0;
int16_t  current_temperature = 0;
uint64_t chip_id;
char device_id[8];

#define FAN_FREQUENCY     15000   // PWM frequency for fan control
#define FAN_CHANNEL           0   // PWM output channel for fan
#define FAN_RESOLUTION        8   // PWM resolution for fan
#define HEATER_FREQUENCY   5000   // PWM frequency for heater control
#define HEATER_CHANNEL        1   // PWM output channel for heater
#define HEATER_RESOLUTION     8   // PWM resolution for heater

#ifdef ENABLE_BLUETOOTH
  // Make sure Bluetooth is available in the ESP32 core
  #if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
    #error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
  #endif
  BluetoothSerial SerialBT;
#endif

/**
 * Setup
 */
void setup() {
  Serial.begin(9600);
  Serial.println("Hand heater 2 starting up");

  // Get a unique device ID to use for MQTT connections
  chip_id = ESP.getEfuseMac();  //The chip ID is essentially its MAC address(length: 6 bytes).
  //Serial.printf("%08X\n",(uint32_t)chipid);//print Low 4bytes.

  sprintf(device_id, "%x", chip_id);
  //device_id = String(chip_id, HEX);  // Get the unique ID of the ESP32 chip in hex
  Serial.print("Device ID: ");
  Serial.println(device_id);

  #ifdef ENABLE_BLUETOOTH
    SerialBT.begin("Heater2"); //Bluetooth device name
    Serial.println("== Bluetooth has started. Pair with the device called 'Heater2'");
  #else
    Serial.println("== Bluetooth not activated");
  #endif

  #ifdef ENABLE_WIFI
    setup_wifi();
    Serial.println("== WiFi started");
  #else
    Serial.println("== WiFi not activated");
  #endif

  #ifdef ENABLE_CAN
    initialise_can_bus();
    Serial.println("== CAN bus started");
  #else
    Serial.println("== CAN bus not activated");
  #endif

  #ifdef ENABLE_MQTT
    client.setServer(MQTT_BROKER, 1883);
    client.setCallback(mqtt_callback);
    Serial.println("== MQTT started");
    // Set up the topics for publishing sensor readings. By inserting the unique ID,
    // the result is of the form: "cmnd/d9616f/POWER"
    sprintf(command_topic, "cmnd/%x/POWER", chip_id);  // For receiving messages
    // Report the topics to the serial console
    Serial.print("Command topic: ");
    Serial.println(command_topic);
  #else
    Serial.println("== MQTT not activated");
  #endif
  
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
 * Connect to WiFi
 */
#ifdef ENABLE_WIFI
void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}
#endif

#ifdef ENABLE_MQTT
/**
 * Called when a message arrives
 */
void mqtt_callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  if (String(topic) == "cmnd/heater2/POWER") {
    Serial.println(messageTemp);
    if(messageTemp == "+"){
      increment_heater_state();
    }
    if(messageTemp == "-"){
      decrement_heater_state();
    }
  }

  // Feel free to add more if statements to control more GPIOs with MQTT
  /*
  // If a message is received on the topic esp32/output, you check if the message is either "on" or "off". 
  // Changes the output state according to the message
  if (String(topic) == "esp32/output") {
    Serial.print("Changing output to ");
    if(messageTemp == "on"){
      Serial.println("on");
      digitalWrite(ledPin, HIGH);
    }
    else if(messageTemp == "off"){
      Serial.println("off");
      digitalWrite(ledPin, LOW);
    }
  }
  */
}

/**
 * Attempt connection to broker
 */
void reconnect_mqtt() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
      // Subscribe
      client.subscribe("cmnd/heater2/POWER");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
#endif


/**
 * Called when a pulse is detected from the fan tacho
 */
void fan_tacho_isr()
{
  fan_pulses++;
}

/**
 * Set up the bus
 */
#ifdef ENABLE_CAN
void initialise_can_bus()
{
  /* set CAN pins and baudrate */
  CAN_cfg.speed     = CAN_SPEED;
  CAN_cfg.tx_pin_id = CAN_TX_PIN;
  CAN_cfg.rx_pin_id = CAN_RX_PIN;
  /* create a queue for CAN receiving */
  CAN_cfg.rx_queue = xQueueCreate(10,sizeof(CAN_frame_t));
  //initialize CAN Module
  ESP32Can.CANInit();
}
#endif

/**
 * Loop
 */
void loop() {
  // Process the MQTT queue
  #ifdef ENABLE_MQTT
    if (!client.connected()) {
      reconnect_mqtt();
    }
    client.loop();
  #endif
  
  // Check if the button has been pressed
  check_button();

  // Check if a command has been sent via the regular serial port
  check_usb_serial();

  // Check if a command has been sent via the Bluetooth serial port
  #ifdef ENABLE_BLUETOOTH
    check_bt_serial();
  #endif

  // Check if a command has been sent via CAN bus
  #ifdef ENABLE_CAN
    check_can_bus();
  #endif

  // Flash the status LED
  led_flasher();

  // Process accumulated fan tacho pulses to determine the speed
  process_fan_tacho_pulses();
  report_fan_speed();

  // Check the temperature sensor for an over-temp condition. Do this
  // after all other checks so that it will override commands from other
  // sources
  check_temperature();
  report_temperature();

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
void process_fan_tacho_pulses()
{
  uint32_t time_now = millis();
  if((time_now - fan_last_checked) > (FAN_CHECK_INTERVAL * 1000))
  {
    fan_last_checked = time_now;
    //Serial.print(fan_pulses);
    //Serial.print("  ");
    fan_speed = fan_pulses * (60 / FAN_CHECK_INTERVAL);
    fan_pulses = 0;
  }
}

/**
 * Report the latest fan speed reading
 */
void report_fan_speed()
{
  uint32_t time_now = millis();
  if((time_now - fan_speed_last_reported) > (FAN_SPEED_REPORT_INTERVAL * 1000))
  {
    fan_speed_last_reported = time_now;  // save the last time we reported the fan speed
    Serial.print("Fan speed: ");
    Serial.print(fan_speed);
    Serial.println("rpm");

    #ifdef ENABLE_BLUETOOTH
    SerialBT.print("Fan speed: ");
    SerialBT.print(fan_speed);
    SerialBT.println("rpm");
    #endif

    #ifdef ENABLE_MQTT
      char mqttCharBuf[12];
      String mqttStringOne;
      mqttStringOne += fan_speed;
      mqttStringOne.toCharArray(mqttCharBuf, mqttStringOne.length()+1);
      client.publish("stat/heater2/FAN",mqttCharBuf);
    #endif

    char charBuf[20];
    String stringOne = "Fan:";
    stringOne += fan_speed;
    stringOne.toCharArray(charBuf, stringOne.length()+1);

    #ifdef ENABLE_CAN
    CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = 5;
    tx_frame.FIR.B.DLC = stringOne.length()+1;
    for(int i = 0; i < tx_frame.FIR.B.DLC; i++){
      tx_frame.data.u8[i] = charBuf[i];
    }
    
    ESP32Can.CANWriteFrame(&tx_frame);
    #endif
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
  #ifdef ENABLE_BLUETOOTH
    SerialBT.print("Setting heater level to ");
    SerialBT.println(heater_state);
  #endif
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
  #ifdef ENABLE_BLUETOOTH
    SerialBT.print("Setting heater level to ");
    SerialBT.println(heater_state);
  #endif
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
#ifdef ENABLE_BLUETOOTH
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
#endif

/**
 * See if there is a command from CAN bus
 */
#ifdef ENABLE_CAN
void check_can_bus()
{
  CAN_frame_t rx_frame;
  //receive next CAN frame from queue
  if(xQueueReceive(CAN_cfg.rx_queue,&rx_frame, 3*portTICK_PERIOD_MS)==pdTRUE)
  {
    if(rx_frame.FIR.B.FF==CAN_frame_std)
    {
      //printf("New standard frame");
    } else {
      //printf("New extended frame");
    }

    if(rx_frame.FIR.B.RTR==CAN_RTR)
    {
      //printf(" RTR from 0x%08x, DLC %d\r\n",rx_frame.MsgID,  rx_frame.FIR.B.DLC);
    } else {
      //printf(" from 0x%08x, DLC %d\n",rx_frame.MsgID,  rx_frame.FIR.B.DLC);

      if(rx_frame.MsgID == 3)
      {
        /* Check if we've been sent an up or down command */
        // The loop could be removed to read 1 character, but it's left here to
        // make it easier to have longer commands later
        //for(int i = 0; i < 8; i++)
        for(int i = 0; i < 1; i++)
        {
          if(rx_frame.data.u8[i] == '+')
          {
            increment_heater_state();
          }
          if(rx_frame.data.u8[i] == '-')
          {
            decrement_heater_state();
         }
          //rx_frame.data.u8[i] = rx_frame.data.u8[i] - 32;
        }
      }
    }
    //respond to sender
    ESP32Can.CANWriteFrame(&rx_frame);
  }
}
#endif

/**
 * Blink the LED at a set interval
 */
void led_flasher()
{
  uint32_t time_now = millis();
  if ((time_now - led_last_blinked >= led_blink_interval) && (led_blink_interval > 0)) {
    // save the last time you blinked the LED
    led_last_blinked = time_now;
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
  uint32_t time_now = millis();
  if((time_now - temperature_last_checked) > (TEMPERATURE_CHECK_INTERVAL * 1000))
  {
    // save the last time we checked temperature
    temperature_last_checked = millis();
    current_temperature = getTemp();

    if(current_temperature > TEMPERATURE_SAFE_LIMIT)
    {
      Serial.println("ALARM: OVER TEMPERATURE LIMIT. Heater disabled");
      Serial.print("Temperature: ");
      Serial.print(current_temperature);
      Serial.println("C");
      heater_state = STATE_OFF;
    }
  }
}

/**
 * Report the latest temperature reading
 */
void report_temperature()
{
  uint32_t time_now = millis();
  if((time_now - temperature_last_reported) > (TEMPERATURE_REPORT_INTERVAL * 1000))
  {
    // save the last time we reported temperature
    temperature_last_reported = time_now;

    // Report to the USB serial port
    Serial.print("Temperature: ");
    Serial.print(current_temperature);
    Serial.println("C");

    // Report to the BT serial port
    #ifdef ENABLE_BLUETOOTH
      SerialBT.print("Temperature: ");
      SerialBT.print(current_temperature);
      SerialBT.println("C");
    #endif

    #ifdef ENABLE_MQTT
      char mqttCharBuf[12];
      String mqttStringOne;
      mqttStringOne += current_temperature;
      mqttStringOne.toCharArray(mqttCharBuf, mqttStringOne.length()+1);
      client.publish("stat/heater2/TEMP",mqttCharBuf);
    #endif

    #ifdef ENABLE_CAN
      char charBuf[12];
      String stringOne = "Temp:";
      stringOne += current_temperature;
      stringOne.toCharArray(charBuf, stringOne.length()+1);
    
      CAN_frame_t tx_frame;
      tx_frame.FIR.B.FF = CAN_frame_std;
      tx_frame.MsgID = 4;
      tx_frame.FIR.B.DLC = stringOne.length()+1;
      for(int i = 0; i < tx_frame.FIR.B.DLC; i++){
        tx_frame.data.u8[i] = charBuf[i];
      }
    
      ESP32Can.CANWriteFrame(&tx_frame);
    #endif
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
