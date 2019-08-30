// Your SSID/Password:
#define WIFI_SSID       "Your SSID"
#define WIFI_PASSWORD   "Your PSK"

// Your MQTT Broker IP address:
#define MQTT_BROKER     "192.168.0.96"

// All messages via Bluetooth must be prepended with this key
#define BT_PASSWORD     "MyKey"

// Activate features
#define ENABLE_WIFI       true
#define ENABLE_MQTT       true
#define ENABLE_BLUETOOTH  true
#define ENABLE_CAN        false

#define TEMPERATURE_OFFSET -7
#define FIRMWARE_VERSION    3

#define FAN_COLD_SPEED    80
#define FAN_WARM_SPEED    80
#define FAN_HOT_SPEED    100

#define TEMPERATURE_WARM  80
#define TEMPERATURE_HOT   120

// Maximum safe output temperature in Celsius
#define TEMPERATURE_SAFE_LIMIT 130

/** To Do
 *  - Simple setup of WiFi config. Maybe AP mode?
 *  - Factory reset mode. Perhaps hold down button while powering up?
 *  - Bluetooth security? Allows any device to pair. Perhaps prepend all
 *     control messages with a unique key (PSK)
 */
