// Language config
//#define CURRENT_LANG INTL_LANG
#define CURRENT_LANG "EN"


// Wifi config
const char WLANSSID[] PROGMEM = "luftdaten";
const char WLANPWD[] PROGMEM = "luftd4ten";

// BasicAuth config
const char WWW_USERNAME[] PROGMEM = "admin";
const char WWW_PASSWORD[] PROGMEM = "";
#define WWW_BASICAUTH_ENABLED 0

// Sensor Wifi config (config mode)
#define FS_SSID ""
#define FS_PWD "cohub66cfg"

#define HAS_WIFI 1
#define HAS_LORA 0
const char APPEUI[] = "0000000000000000";
const char DEVEUI [] = "0000000000000000";
const char APPKEY[] = "00000000000000000000000000000000";

// Where to send the data?
#define SEND2SENSORCOMMUNITY 0
#define SSL_SENSORCOMMUNITY 0
#define SEND2MADAVI 0
#define SSL_MADAVI 0
#define SEND2CSV 0
#define SEND2CUSTOM 0
#define SEND2CUSTOM2 0

enum LoggerEntry {
    LoggerSensorCommunity,
    LoggerMadavi,
    LoggerCustom,
    LoggerCustom2,
    LoggerCount
};

struct LoggerConfig {
    uint16_t destport;
    uint16_t errors;
    void* session;
};

// IMPORTANT: NO MORE CHANGES TO VARIABLE NAMES NEEDED FOR EXTERNAL APIS
static const char HOST_MADAVI[] PROGMEM = "api-rrd.madavi.de";
static const char URL_MADAVI[] PROGMEM = "/data.php";
#define PORT_MADAVI 80

static const char HOST_SENSORCOMMUNITY[] PROGMEM = "api.sensor.community";
static const char URL_SENSORCOMMUNITY[] PROGMEM = "/v1/push-sensor-data/";
#define PORT_SENSORCOMMUNITY 80

static const char NTP_SERVER_1[] PROGMEM = "ntp-p1.obspm.fr";
static const char NTP_SERVER_2[] PROGMEM = "ntp.obspm.fr";

// define own API
static const char HOST_CUSTOM[] PROGMEM = "192.168.234.1";
static const char URL_CUSTOM[] PROGMEM = "/data.php";
#define PORT_CUSTOM 80
#define USER_CUSTOM ""
#define PWD_CUSTOM ""
#define SSL_CUSTOM 0

// define own API
static const char HOST_CUSTOM2[] PROGMEM = "192.168.234.1";
static const char URL_CUSTOM2[] PROGMEM = "/data.php";
#define PORT_CUSTOM2 80
#define USER_CUSTOM2 ""
#define PWD_CUSTOM2 ""
#define SSL_CUSTOM2 0


#if defined(ARDUINO_HELTEC_WIFI_LORA_32_V2)

#define I2C_SCREEN_SCL D15
#define I2C_SCREEN_SDA D4
#define OLED_RESET D16
#define PM_SERIAL_RX D23
#define PM_SERIAL_TX D17

// #define ONEWIRE_PIN D32
// #define PM_SERIAL_RX D27
// #define PM_SERIAL_TX D33

#define I2C_PIN_SCL D22
#define I2C_PIN_SDA D21
// #define GPS_SERIAL_RX D12
// #define GPS_SERIAL_TX D13

// const lmic_pinmap lmic_pins = {
// 	.nss = D18,
// 	.rxtx = LMIC_UNUSED_PIN,
// 	.rst = D14,
// 	.dio = {/*dio0*/ D26, /*dio1*/ D35, /*dio2*/ D34},
// 	.rxtx_rx_active = 0,
// 	.rssi_cal = 10,
// 	.spi_freq = 8000000 /* 8 MHz */
// };
#endif

// SDS011, the more expensive version of the particle sensor
#define SDS_READ 0
#define SDS_API_PIN 1

//Location

const char LATITUDE[] PROGMEM = "43.296";
const char LONGITUDE[] PROGMEM = "5.369";

// OLED Display SSD1306
#define HAS_SSD1306 0

//Actual Data

#define DISPLAY_MEASURE 0
#define DISPLAY_FORECAST 0

// Show wifi info on displays
#define DISPLAY_WIFI_INFO 1

// Show wifi info on displays
#define DISPLAY_LORA_INFO 0

// Show device info on displays
#define DISPLAY_DEVICE_INFO 1

// Set debug level for serial output?
#define DEBUG 3

static const char URL_API_SENSORCOMMUNITY[] PROGMEM = "https://data.sensor.community/airrohr/v1/sensor/";
