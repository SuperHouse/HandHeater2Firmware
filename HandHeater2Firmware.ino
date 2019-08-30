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

/*   YOUR LOCAL CONFIGURATION   */
#include "config.h"


/*   NO NEED TO CHANGE ANYTHING BELOW THIS LINE FOR CONFIGURATION   */
/* WiFi library */
#if ENABLE_WIFI
#include <WiFi.h>
WiFiClient espClient;
#endif

#if ENABLE_WIFI
// Set web server port number to 80
WiFiServer server(80);
// Variable to store the HTTP request
String header;
#endif

/* Bluetooth library */
#if ENABLE_BLUETOOTH
#include "BluetoothSerial.h"
#endif

/* CAN bus library setup */
#if ENABLE_CAN
#include <ESP32CAN.h>
#include <CAN_config.h>
CAN_device_t CAN_cfg;  // The variable name CAN_cfg is fixed, do not change
#define CAN_SPEED CAN_SPEED_1000KBPS
#endif

/* MQTT library */
#if ENABLE_MQTT
#include <PubSubClient.h>
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
char command_topic[30];
char status_topic[30];
char temperature_topic[30];
char target_topic[30];
char rssi_topic[30];
char fan_topic[30];
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

// How often to process accumulated fan tacho pulses, in seconds
#define FAN_TACHO_PROCESS_INTERVAL  0.5
uint32_t fan_tacho_last_processed = 0;

// How often to check that the fan is functional, in seconds
#define FAN_CHECK_INTERVAL   2
#define FAN_FAIL_COUNT_LIMIT 3
uint32_t fan_last_checked  = 0;

// Whether the fan needs to be on, regardless of current state
bool fan_forced_on = false;

// How often to report the fan tacho, in seconds
#define FAN_SPEED_REPORT_INTERVAL  1
uint32_t fan_speed_last_reported = 0;

// How often to check the temperature, in seconds
#define TEMPERATURE_CHECK_INTERVAL  1
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
uint32_t fan_pulses          = 0;   // Accumulate pulses from fan tacho
uint32_t fan_speed           = 0;   // The current fan speed
uint32_t fan_fail_count      = 0;   // We have to see it fail on multiple consecutive checks
int16_t  current_temperature = 0;
int16_t  target_temperature  = 0;
uint8_t  wifi_status_reported = false;
uint64_t chip_id;
char device_id[18];
int measurementNumber = 0;

#define FAN_FREQUENCY     15000   // PWM frequency for fan control
#define FAN_CHANNEL           0   // PWM output channel for fan
#define FAN_RESOLUTION        8   // PWM resolution for fan
//#define HEATER_FREQUENCY   5000   // PWM frequency for heater control
//#define HEATER_CHANNEL        1   // PWM output channel for heater
//#define HEATER_RESOLUTION     8   // PWM resolution for heater

#if ENABLE_BLUETOOTH
// Make sure Bluetooth is available in the ESP32 core
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
BluetoothSerial SerialBT;
#endif


/**
   Called when a pulse is detected from the fan tacho
*/
void IRAM_ATTR fan_tacho_isr()
{
  fan_pulses++;
}


/**
   Setup
*/
void setup() {
  Serial.begin(9600);
  Serial.println("Hand heater 2 starting up");
  Serial.print("Firmware version: ");
  Serial.println(FIRMWARE_VERSION);

  // Get a unique device ID to use for MQTT connections, Bluetooth, etc
  chip_id = ESP.getEfuseMac(); // The chip ID is essentially its MAC address (length 6 bytes)
  //sprintf(device_id, "%04X", (uint16_t)chip_id); // Just use the last 4 bytes
  sprintf(device_id, "Heater-%08X", chip_id); // Use all the bytes
  Serial.print("Device ID: ");
  Serial.println(device_id);

#if ENABLE_BLUETOOTH
  //char bt_id[18];
  //sprintf(bt_id,"Heater-%08X",chip_id);
  /* The BT "begin" below uses the current method in the Arduino IDE, which doesn't
      support a PIN. PIN support was added in PR 2765:
      https://github.com/espressif/arduino-esp32/pull/2765
      When this has been released, we can replace the line below with a PIN such as:
      SerialBT.begin(bt_id, 123456);
  */
  SerialBT.begin(device_id); //Bluetooth device name
  Serial.print("== Bluetooth has started. Pair with the device called '");
  Serial.print(device_id);
  Serial.println("'");
#else
  Serial.println("== Bluetooth not activated");
#endif

#if ENABLE_WIFI
  setup_wifi();
  Serial.println("== WiFi started");
#else
  Serial.println("== WiFi not activated");
#endif

#if ENABLE_WIFI
  server.begin();
#endif

#if ENABLE_CAN
  initialise_can_bus();
  Serial.println("== CAN bus started");
#else
  Serial.println("== CAN bus not activated");
#endif

#if ENABLE_MQTT
  client.setServer(MQTT_BROKER, 1883);
  client.setCallback(mqtt_callback);
  Serial.println("== MQTT started");
  // Set up the topics for publishing sensor readings. By inserting the unique ID,
  // the result is of the form: "cmnd/D9616F/POWER"
  sprintf(command_topic, "cmnd/%s/POWER", device_id);     // Receive power commands
  sprintf(status_topic, "stat/%s/POWER", device_id);      // Report power status
  sprintf(temperature_topic, "stat/%s/TEMP", device_id);  // Report current temperature
  sprintf(target_topic, "stat/%s/TRGT", device_id);       // Report target temperature
  sprintf(fan_topic, "stat/%s/FAN", device_id);           // Report fan speed
  sprintf(rssi_topic, "stat/%s/RSSI", device_id);         // Report WiFi signal strength

  // Report the topics to the serial console
  Serial.print("Command topic: ");
  Serial.println(command_topic);
  Serial.print("Status topic: ");
  Serial.println(status_topic);
  Serial.print("Temperature topic: ");
  Serial.println(temperature_topic);
  Serial.print("Fan speed topic: ");
  Serial.println(fan_topic);
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

  //ledcSetup(HEATER_CHANNEL, HEATER_FREQUENCY, HEATER_RESOLUTION);
  //ledcAttachPin(HEATER_PIN, HEATER_CHANNEL);
  //ledcWrite(HEATER_CHANNEL, 0);

  // Start with the heater off
  digitalWrite(HEATER_PIN, LOW);

  // Start with the LED off
  digitalWrite(STATUS_LED_PIN, LOW);

  // Detect pulses from the fan tacho using an interrupt
  pinMode(FAN_TACHO_PIN, INPUT_PULLUP);
  attachInterrupt(FAN_TACHO_PIN, fan_tacho_isr, FALLING);
}

/**
   Connect to WiFi
*/
#if ENABLE_WIFI
void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}
#endif


/**
   Report WiFi status
*/
#if ENABLE_WIFI
void report_wifi_status()
{
  if (wifi_status_reported != true)
  {
    if (WiFi.status() == WL_CONNECTED)
    {
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      wifi_status_reported = true;
    }
  }
}
#endif

/**
   Called when a message arrives
*/
#if ENABLE_MQTT
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

  if (String(topic) == command_topic) {
    Serial.println(messageTemp);
    if (messageTemp == "OFF") {
      heater_state = STATE_OFF;
      //increment_heater_state();
    }
    if (messageTemp == "COLD") {
      heater_state = STATE_COLD;
      //decrement_heater_state();
    }
    if (messageTemp == "WARM") {
      heater_state = STATE_WARM;
    }
    if (messageTemp == "HOT") {
      heater_state = STATE_HOT;
    }
  }
}
#endif

/**
   Attempt connection to broker
*/
#if ENABLE_MQTT
void reconnect_mqtt() {
  // Loop until we're reconnected
  if (WiFi.status() == WL_CONNECTED)
  {
    while (!client.connected()) {
      Serial.print("Attempting MQTT connection...");
      // Attempt to connect
      if (client.connect(device_id)) {
        Serial.println("connected");
        // Subscribe
        client.subscribe(command_topic);
      } else {
        Serial.print("failed, rc=");
        Serial.print(client.state());
        Serial.println(" try again in 5 seconds");
        // Wait 5 seconds before retrying
        delay(5000);
      }
    }
  }
}
#endif


/**
   Set up the bus
*/
#if ENABLE_CAN
void initialise_can_bus()
{
  /* set CAN pins and baudrate */
  CAN_cfg.speed     = CAN_SPEED;
  CAN_cfg.tx_pin_id = CAN_TX_PIN;
  CAN_cfg.rx_pin_id = CAN_RX_PIN;
  /* create a queue for CAN receiving */
  CAN_cfg.rx_queue = xQueueCreate(10, sizeof(CAN_frame_t));
  //initialize CAN Module
  ESP32Can.CANInit();
}
#endif

/**
   Loop
*/
void loop() {
  // Process the MQTT queue
#if ENABLE_MQTT
  if (!client.connected()) {
    reconnect_mqtt();
  }
  client.loop();
#endif

  // Report our IP address etc once only. Don't report again after that
#if ENABLE_WIFI
  if (wifi_status_reported == false)
  {
    report_wifi_status();
  }
#endif

#if ENABLE_WIFI
  WiFiClient client = server.available();   // Listen for incoming clients
  display_web_page(client);
#endif

  // Check if the button has been pressed
  check_button();

  // Check if a command has been sent via the regular serial port
  check_usb_serial();

  // Check if a command has been sent via the Bluetooth serial port
#if ENABLE_BLUETOOTH
  check_bt_serial();
#endif

  // Check if a command has been sent via CAN bus
#if ENABLE_CAN
  check_can_bus();
#endif

  // Flash the status LED
  led_flasher();

  // Process accumulated fan tacho pulses to determine the speed
  process_fan_tacho_pulses();
  report_fan_speed();

  // Check that the fan is spinning, and kill the output if it's not
  check_fan();

  // Check the temperature sensor for an over-temp condition. Do this
  // after all other checks so that it will override commands from other
  // sources
  check_temperature();
  report_temperature();

  // Main state machine
  //Serial.print("State: ");
  //Serial.println(heater_state);
  switch (heater_state) {
    case STATE_OFF:
      led_blink_interval = 0;
      target_temperature = 0;
      if(fan_forced_on == true)
      {
        ledcWrite(FAN_CHANNEL, FAN_WARM_SPEED);
      } else {
        ledcWrite(FAN_CHANNEL, 0);
      }
      break;
    case STATE_COLD:
      led_blink_interval = 800;
      target_temperature = 0;
      ledcWrite(FAN_CHANNEL, FAN_COLD_SPEED);
      break;
    case STATE_WARM:
      led_blink_interval = 200;
      target_temperature = TEMPERATURE_WARM;
      ledcWrite(FAN_CHANNEL, FAN_WARM_SPEED);
      break;
    case STATE_HOT:
      digitalWrite(STATUS_LED_PIN, HIGH);
      target_temperature = TEMPERATURE_HOT;
      ledcWrite(FAN_CHANNEL, FAN_HOT_SPEED);
      break;
  }
}

#if ENABLE_WIFI
void display_web_page(WiFiClient client)
{
  if (client) {                             // If a new client connects,
    Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();

            if (header.indexOf("GET /OFF") >= 0) {
              heater_state = STATE_OFF;
            } else if (header.indexOf("GET /COLD") >= 0) {
              heater_state = STATE_COLD;
            } else if (header.indexOf("GET /WARM") >= 0) {
              heater_state = STATE_WARM;
            } else if (header.indexOf("GET /HOT") >= 0) {
              heater_state = STATE_HOT;
            }

            // Display the HTML web page
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            client.println("<meta http-equiv=\"refresh\" content=\"10;url=/\" />");
            // CSS to style the on/off buttons
            // Feel free to change the background-color and font-size attributes to fit your preferences
            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println(".button { background-color: #555555; border: none; color: white; padding: 16px 40px;");
            client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            client.println(".button_off  {background-color: #4CAF50;}");
            client.println(".button_cold {background-color: #000099;}");
            client.println(".button_warm {background-color: #faa00f;}");
            client.println(".button_hot  {background-color: #990000;}");
            client.println("</style></head>");

            // Web Page Heading
            client.println("<body><h1>Hand Heater</h1>");

            String httpStringOne;
            httpStringOne += current_temperature;
            client.println("<p>Current temperature: " + httpStringOne + "&deg;</p>");

            if (heater_state == STATE_HOT) {
              client.println("<p><a href=\"/HOT\"><button class=\"button button_hot\">Hot</button></a></p>");
            } else {
              client.println("<p><a href=\"/HOT\"><button class=\"button\">Hot</button></a></p>");
            }

            if (heater_state == STATE_WARM) {
              client.println("<p><a href=\"/WARM\"><button class=\"button button_warm\">Warm</button></a></p>");
            } else {
              client.println("<p><a href=\"/WARM\"><button class=\"button\">Warm</button></a></p>");
            }

            if (heater_state == STATE_COLD) {
              client.println("<p><a href=\"/COLD\"><button class=\"button button_cold\">Cold</button></a></p>");
            } else {
              client.println("<p><a href=\"/COLD\"><button class=\"button\">Cold</button></a></p>");
            }

            if (heater_state == STATE_OFF) {
              client.println("<p><a href=\"/OFF\"><button class=\"button button_off\">Off</button></a></p>");
            } else {
              client.println("<p><a href=\"/OFF\"><button class=\"button\">Off</button></a></p>");
            }

            client.println("</body></html>");

            // The HTTP response ends with another blank line
            client.println();
            // Break out of the while loop
            break;
          } else { // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
  }
}
#endif


/**
   Check how many pulses have been received from the fan tacho
*/
void process_fan_tacho_pulses()
{
  uint32_t time_now = millis();
  if ((time_now - fan_tacho_last_processed) > (FAN_TACHO_PROCESS_INTERVAL * 1000))
  {
    fan_tacho_last_processed = time_now;
    //Serial.print(fan_pulses);
    //Serial.print("  ");
    fan_speed = fan_pulses * (60 / FAN_TACHO_PROCESS_INTERVAL);
    fan_pulses = 0;
  }
}

/**
   Report the latest fan speed reading
*/
void report_fan_speed()
{
  uint32_t time_now = millis();
  if ((time_now - fan_speed_last_reported) > (FAN_SPEED_REPORT_INTERVAL * 1000))
  {
    fan_speed_last_reported = time_now;  // save the last time we reported the fan speed
    Serial.print("Fan speed: ");
    Serial.print(fan_speed);
    Serial.println("rpm");

#if ENABLE_BLUETOOTH
    SerialBT.print("Fan speed: ");
    SerialBT.print(fan_speed);
    SerialBT.println("rpm");
#endif

#if ENABLE_MQTT
    char mqttCharBuf[12];
    String mqttStringOne;
    mqttStringOne += fan_speed;
    mqttStringOne.toCharArray(mqttCharBuf, mqttStringOne.length() + 1);
    client.publish(fan_topic, mqttCharBuf);
#endif

    char charBuf[20];
    String stringOne = "Fan:";
    stringOne += fan_speed;
    stringOne.toCharArray(charBuf, stringOne.length() + 1);

#if ENABLE_CAN
    CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = 5;
    tx_frame.FIR.B.DLC = stringOne.length() + 1;
    for (int i = 0; i < tx_frame.FIR.B.DLC; i++) {
      tx_frame.data.u8[i] = charBuf[i];
    }

    ESP32Can.CANWriteFrame(&tx_frame);
#endif
  }
}

/**
   See if the button has been pressed, and apply de-bouncing
*/
void check_button()
{
  byte button_state = digitalRead(BUTTON_PIN);
  //Serial.print(button_state);
  if ((button_state == LOW) && (last_button_state == HIGH) && (millis() - last_button_press > 200))
  {
    // We've detected a button press. Change state.
    increment_heater_state();
    last_button_press = millis();
  }
  last_button_state = button_state;
}

/**
   Make the output hotter. Wrap around from max back to min
*/
void increment_heater_state()
{
  heater_state++;
  if (heater_state > STATE_HOT) // If we've hit the end state, loop back to the start
  {
    heater_state = STATE_OFF;
  }
  Serial.print("Setting heater level to ");
  Serial.println(heater_state);
#if ENABLE_BLUETOOTH
  SerialBT.print("Setting heater level to ");
  SerialBT.println(heater_state);
#endif
}

/**
   Make the output colder. Do NOT wrap around from min back to max, for safety
*/
void decrement_heater_state()
{

  if (heater_state > STATE_OFF) // Only decrement while it's above the lowest setting
  {
    heater_state--;
  }
  Serial.print("Setting heater level to ");
  Serial.println(heater_state);
#if ENABLE_BLUETOOTH
  SerialBT.print("Setting heater level to ");
  SerialBT.println(heater_state);
#endif
}


/**
   See if there is a command from the USB serial port
*/
void check_usb_serial()
{
  if (Serial.available())
  {
    String serial_message;
    serial_message = Serial.readStringUntil('\n');
    //byte character = Serial.read();
    if (serial_message == "+")
    {
      increment_heater_state();
    }
    if (serial_message == "-")
    {
      decrement_heater_state();
    }
    if (serial_message == "ON")
    {
      heater_state = STATE_HOT;
    }
    if (serial_message == "OFF")
    {
      heater_state = STATE_OFF;
    }
  }
}


/**
   See if there is a command from the Bluetooth serial port
*/
#if ENABLE_BLUETOOTH
void check_bt_serial()
{
  if (SerialBT.available())
  {
    //String message_up = str_concat(BT_PASSWORD," +");
    String message_up   = BT_PASSWORD " +";
    String message_down = BT_PASSWORD " -";
    String message_on   = BT_PASSWORD " ON";
    String message_off  = BT_PASSWORD " OFF";

    String serial_message;
    serial_message = SerialBT.readStringUntil('\n');
    //byte character = Serial.read();
    if (serial_message == message_up)
    {
      increment_heater_state();
    }
    if (serial_message == message_down)
    {
      decrement_heater_state();
    }
    if (serial_message == message_on)
    {
      heater_state = STATE_HOT;
    }
    if (serial_message == message_off)
    {
      heater_state = STATE_OFF;
    }
  }
}
#endif

/**
   See if there is a command from CAN bus
*/
#if ENABLE_CAN
void check_can_bus()
{
  CAN_frame_t rx_frame;
  //receive next CAN frame from queue
  if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE)
  {
    if (rx_frame.FIR.B.FF == CAN_frame_std)
    {
      //printf("New standard frame");
    } else {
      //printf("New extended frame");
    }

    if (rx_frame.FIR.B.RTR == CAN_RTR)
    {
      //printf(" RTR from 0x%08x, DLC %d\r\n",rx_frame.MsgID,  rx_frame.FIR.B.DLC);
    } else {
      //printf(" from 0x%08x, DLC %d\n",rx_frame.MsgID,  rx_frame.FIR.B.DLC);

      if (rx_frame.MsgID == 3)
      {
        /* Check if we've been sent an up or down command */
        // The loop could be removed to read 1 character, but it's left here to
        // make it easier to have longer commands later
        //for(int i = 0; i < 8; i++)
        for (int i = 0; i < 1; i++)
        {
          if (rx_frame.data.u8[i] == '+')
          {
            increment_heater_state();
          }
          if (rx_frame.data.u8[i] == '-')
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
   Blink the LED at a set interval
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
  } else if (led_blink_interval == 0) {
    digitalWrite(STATUS_LED_PIN, LOW);
  }
}

/**
   Check whether we're above or below the target temperature, and turn the
   heating element on or off if necessary.
   Also check whether we've exceeded the temperature limit and turn off
   if we have, as a safety measure
*/
void check_temperature()
{
  uint32_t time_now = millis();
  if ((time_now - temperature_last_checked) > (TEMPERATURE_CHECK_INTERVAL * 1000))
  {
    // save the last time we checked temperature
    temperature_last_checked = millis();
    current_temperature = getTemp();

    if (current_temperature > target_temperature)
    {
      // Turn the heater off
      digitalWrite(HEATER_PIN, LOW);
    }
    
    if (current_temperature < target_temperature)
    {
      // Turn the heater on
      digitalWrite(HEATER_PIN, HIGH);
    }

    if (current_temperature > TEMPERATURE_SAFE_LIMIT)
    {
      Serial.println("ALARM: OVER TEMPERATURE LIMIT. Heater disabled");
      Serial.print("Actual Temperature: ");
      Serial.print(current_temperature);
      Serial.println("C");
      if ((heater_state == STATE_HOT) || (heater_state == STATE_WARM))
      {
        //ledcWrite(HEATER_CHANNEL, 0);
        // Turn the heater off
        digitalWrite(HEATER_PIN, LOW);
        heater_state = STATE_OFF;
      }
    }

    // This is so the fan will run even if the heater is
    // turned off, because turning the fan off immediately
    // when shutting down causes a temperature spike. If
    // we force the fan to be on at any time the temperature
    // is over the target, it helps prevent runaways.
    if (current_temperature > TEMPERATURE_WARM)
    {
      fan_forced_on = true;
    } else {
      fan_forced_on = false;
    }
  }
}


/**
 *  Check whether the fan has stalled and shut down if necessary
 */
void check_fan()
{
  uint32_t time_now = millis();
  if ((time_now - fan_last_checked) > (FAN_CHECK_INTERVAL * 1000))
  {
    fan_last_checked = time_now;  // save the last time we checked the fan speed
    if (heater_state > STATE_COLD)
    {
      if (fan_speed < 100)
      {
        fan_fail_count++;
        if (fan_fail_count >= FAN_FAIL_COUNT_LIMIT)
        {
          Serial.println("ALARM: FAN STALLED. Heater turned off");
#if ENABLE_BLUETOOTH
          SerialBT.println("ALARM: FAN STALLED. Heater turned off");
#endif
          heater_state = STATE_OFF;
        } else {
          Serial.println("ALARM: FAN STALLED. Will check again before turning off");
#if ENABLE_BLUETOOTH
          SerialBT.println("ALARM: FAN STALLED. Will check again before turning off");
#endif
        }
      } else {
        fan_fail_count = 0;  // Reset the fail counter if the fan is spinning
      }
    }
  }
}


/**
 * Report the latest temperature reading
 */
void report_temperature()
{
  uint32_t time_now = millis();
  if ((time_now - temperature_last_reported) > (TEMPERATURE_REPORT_INTERVAL * 1000))
  {
    // save the last time we reported temperature
    temperature_last_reported = time_now;

    // Report to the USB serial port
    Serial.print("Target Temperature: ");
    Serial.print(target_temperature);
    Serial.println("C");
    Serial.print("Actual Temperature: ");
    Serial.print(current_temperature);
    Serial.println("C");


    // Report to the BT serial port
#if ENABLE_BLUETOOTH
    SerialBT.print("Target Temperature: ");
    SerialBT.print(target_temperature);
    SerialBT.println("C");
    SerialBT.print("Actual Temperature: ");
    SerialBT.print(current_temperature);
    SerialBT.println("C");
#endif

#if ENABLE_MQTT
    char mqttCharBuf[12];
    String mqttStringOne;
    mqttStringOne += current_temperature;
    mqttStringOne.toCharArray(mqttCharBuf, mqttStringOne.length() + 1);
    client.publish(temperature_topic, mqttCharBuf);

    mqttStringOne = getWifiStrength();
    mqttStringOne.toCharArray(mqttCharBuf, mqttStringOne.length() + 1);
    client.publish(rssi_topic, mqttCharBuf);

    mqttStringOne = target_temperature;
    mqttStringOne.toCharArray(mqttCharBuf, mqttStringOne.length() + 1);
    client.publish(target_topic, mqttCharBuf);
#endif

#if ENABLE_CAN
    char charBuf[12];
    String stringOne = "Temp:";
    stringOne += current_temperature;
    stringOne.toCharArray(charBuf, stringOne.length() + 1);

    CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = 4;
    tx_frame.FIR.B.DLC = stringOne.length() + 1;
    for (int i = 0; i < tx_frame.FIR.B.DLC; i++) {
      tx_frame.data.u8[i] = charBuf[i];
    }

    ESP32Can.CANWriteFrame(&tx_frame);

    stringOne = "Trgt:";
    stringOne += target_temperature;
    stringOne.toCharArray(charBuf, stringOne.length() + 1);

    //CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = 4;
    tx_frame.FIR.B.DLC = stringOne.length() + 1;
    for (int i = 0; i < tx_frame.FIR.B.DLC; i++) {
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
  for (i = 0; i < 5; i++) {
    thermalSamples[i] = analogRead(TEMP_SENSOR_PIN);
    delay(10);
  }

  // Calculate the average value of the samples
  average = 0;
  for (i = 0; i < 5; i++) {
    average += thermalSamples[i];
  }
  average /= 5;

  // Convert to resistance
  resistance = 4095 / average - 1;
  resistance = 10000 / resistance;

  /*
     Use Steinhart equation (simplified B parameter equation) to convert resistance to kelvin
     B param eq: T = 1/( 1/To + 1/B * ln(R/Ro) )
     T  = Temperature in Kelvin
     R  = Resistance measured
     Ro = Resistance at nominal temperature
     B  = Coefficent of the thermistor
     To = Nominal temperature in kelvin
  */
  kelvin = resistance / 10000;          // R/Ro
  kelvin = log(kelvin);                 // ln(R/Ro)
  kelvin = (1.0 / BCOEFFICIENT) * kelvin; // 1/B * ln(R/Ro)
  kelvin = (1.0 / (25 + 273.15)) + kelvin; // 1/To + 1/B * ln(R/Ro)
  kelvin = 1.0 / kelvin;                // 1/( 1/To + 1/B * ln(R/Ro) )

  // Convert Kelvin to Celsius
  celsius = kelvin - 273.15;

  celsius += TEMPERATURE_OFFSET;

  // Send the value back to be displayed
  return celsius;
}


/**
 * Take 5 measurements of the Wi-Fi strength and return the average result.
 */
#if ENABLE_WIFI
int getWifiStrength() {
  long rssi = 0;
  long averageRSSI = 0;

  for (int i = 0; i < 5; i++) {
    rssi += WiFi.RSSI();
    delay(20);
  }

  averageRSSI = rssi / 5;
  return averageRSSI;
}
#endif
