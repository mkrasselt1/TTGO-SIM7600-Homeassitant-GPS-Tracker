#include <Arduino.h>

#define TINY_GSM_MODEM_SIM7600
#define SerialMon Serial
#define SerialAT Serial1
// #define DUMP_AT_COMMANDS // See all AT commands, if wanted
#define TINY_GSM_DEBUG SerialMon
#define ST(A) #A
#define STR(A) ST(A)
// #define TEST_RING_RI_PIN            true
#define TINY_GSM_RX_BUFFER 1024 // Set RX buffer to 1Kb

float battery = 0.0;

// Your GPRS credentials, if any
const char apn[] = STR(NETWORK_APN);
const char gprsUser[] = "";
const char gprsPass[] = "";

String server = STR(SERVER);
String resource = STR(WEBHOOK_URL);
uint16_t portNumber = SERVER_PORT;
String IMEI = "";

#include <SPI.h>
#include <SD.h>
#include <Ticker.h>
#include <TinyGsmClient.h>
#include <ArduinoHttpClient.h>

#define uS_TO_S_FACTOR 1000000ULL /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 30          /* Time ESP32 will go to sleep (in seconds) */
#if !defined(TINY_GSM_RX_BUFFER)
#define TINY_GSM_RX_BUFFER 650
#endif

#define UART_BAUD 115200

#define MODEM_TX 27
#define MODEM_RX 26
#define MODEM_PWRKEY 4
#define MODEM_DTR 32
#define MODEM_RI 33
#define MODEM_FLIGHT 25
#define MODEM_STATUS 34

#define SD_MISO 2
#define SD_MOSI 15
#define SD_SCLK 14
#define SD_CS 13
#define BAT_ADC 35
#define LED_PIN 12

TinyGsm modem(SerialAT);

TinyGsmClientSecure client(modem);
HttpClient http(client, server.c_str(), portNumber);

float ReadBattery()
{
  float vref = 1.100;
  uint16_t volt = analogRead(BAT_ADC);
  float battery_voltage = ((float)volt / 4095.0) * 2.0 * 3.3 * (vref);
  return battery_voltage;
}

void enableGPS(void)
{

  Serial.println("Start positioning . Make sure to locate outdoors.");
  Serial.println("The blue indicator light flashes to indicate positioning.");
  modem.sendAT("+SGPIO=0,4,1,1");
  if (modem.waitResponse(10000L) != 1)
  {
    DBG(" SGPIO=0,4,1,1 false ");
  }
  modem.enableGPS();
}

void disableGPS(void)
{
  modem.sendAT("+SGPIO=0,4,1,0");
  if (modem.waitResponse(10000L) != 1)
  {
    DBG(" SGPIO=0,4,1,0 false ");
  }
  modem.disableGPS();
}

void send_data(float lat, float lon, float speed, float alt, float accuracy, float battery)
{

  String FINAL_LATI = "0", FINAL_LOGI = "0", FINAL_SPEED = "0", FINAL_ALT = "0", FINAL_ACCURACY = "0", FINAL_BAT = "0", FINAL_BAT_LEVEL = "";
  FINAL_LATI = String(lat, 8);
  FINAL_LOGI = String(lon, 8);
  FINAL_SPEED = String(speed, 2);
  FINAL_ALT = String(alt, 1);
  FINAL_ACCURACY = String(accuracy, 2);
  if (battery == 0)
  {
    FINAL_BAT_LEVEL = "0.0";
    FINAL_BAT = "0.0";
  }
  else
  {
    float batteryLevel = (((float)battery - 3) / 1.2) * 100;
    FINAL_BAT = String(battery, 2);
    if (batteryLevel > 100)
    {
      batteryLevel = 100;
    }
    FINAL_BAT_LEVEL = String(batteryLevel, 0);
  }
  // http.addHeader();
  FINAL_ALT.trim();
  String payload = "latitude=" + FINAL_LATI +
                   "&longitude=" + FINAL_LOGI +
                   "&device=" + IMEI +
                   "&accuracy=" + FINAL_ACCURACY +
                   "&battery=" + FINAL_BAT_LEVEL +
                   "&speed=" + FINAL_SPEED +
                   "&altitude=" + FINAL_ALT +
                   "&provider=TTGO&activity=1";
  int err = http.post("https://" + server + ":" + portNumber + resource, "application/x-www-form-urlencoded", payload);
  Serial.println(payload);
  if (err != 0)
  {
    SerialMon.print("Url: ");
    SerialMon.println("https://" + server + ":" + portNumber + resource);
    SerialMon.println(F("failed to connect"));
    delay(10000);
    return;
  }

  int status = http.responseStatusCode();

  if (!status)
  {
    delay(10000);
    return;
  }

  String body = http.responseBody();
  SerialMon.println(F("Response:"));
  SerialMon.println(body);

  // Shutdown
  http.stop();
  SerialMon.println(F("Server disconnected bye bye will connect soon"));

  /*
  TinyGsmClient client(modem, 0);
  const int port = 80;
  DBG("Connecting to ", server);
  if (!client.connect(server, port)) {
    DBG("... failed");
  } else {
    // Make a HTTP GET request:
    client.print(String("GET ") + resource + " HTTP/1.0\r\n");
    client.print(String("Host: ") + server + "\r\n");
    client.print("Connection: close\r\n\r\n");

    // Wait for data to arrive
    uint32_t start = millis();
    while (client.connected() && !client.available() &&
           millis() - start < 30000L) {
      delay(100);
    };

    // Read data
    start = millis();
    while (client.connected() && millis() - start < 5000L) {
      while (client.available()) {
        SerialMon.write(client.read());
        start = millis();
      }
    }
    client.stop();
  }


  DBG("Enabling GPS/GNSS/GLONASS");
  modem.enableGPS();
  light_sleep(2);

  float lat2      = 0;
  float lon2      = 0;
  float speed2    = 0;
  float alt2      = 0;
  int   vsat2     = 0;
  int   usat2     = 0;
  float accuracy2 = 0;
  int   year2     = 0;
  int   month2    = 0;
  int   day2      = 0;
  int   hour2     = 0;
  int   min2      = 0;
  int   sec2      = 0;
  DBG("Requesting current GPS/GNSS/GLONASS location");
  for (;;) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    if (modem.getGPS(&lat2, &lon2, &speed2, &alt2, &vsat2, &usat2, &accuracy2,
                     &year2, &month2, &day2, &hour2, &min2, &sec2)) {
      DBG("Latitude:", String(lat2, 8), "\tLongitude:", String(lon2, 8));
      DBG("Speed:", speed2, "\tAltitude:", alt2);
      DBG("Visible Satellites:", vsat2, "\tUsed Satellites:", usat2);
      DBG("Accuracy:", accuracy2);
      DBG("Year:", year2, "\tMonth:", month2, "\tDay:", day2);
      DBG("Hour:", hour2, "\tMinute:", min2, "\tSecond:", sec2);
      break;
    } else {
      light_sleep(2);
    }
  }
  DBG("Retrieving GPS/GNSS/GLONASS location again as a string");
  String gps_raw = modem.getGPSraw();
  DBG("GPS/GNSS Based Location String:", gps_raw);
  DBG("Disabling GPS");
//   modem.disableGPS();


  int year3 = 0;
  int month3 = 0;
  int day3 = 0;
  int hour3 = 0;
  int min3 = 0;
  int sec3 = 0;
  float timezone = 0;
  for (int8_t i = 5; i; i--) {
    DBG("Requesting current network time");
    if (modem.getNetworkTime(&year3, &month3, &day3, &hour3, &min3, &sec3,
                             &timezone)) {
      DBG("Year:", year3, "\tMonth:", month3, "\tDay:", day3);
      DBG("Hour:", hour3, "\tMinute:", min3, "\tSecond:", sec3);
      DBG("Timezone:", timezone);
      break;
    } else {
      DBG("Couldn't get network time, retrying in 15s.");
      light_sleep(15);
    }
  }
  DBG("Retrieving time again as a string");
  String time = modem.getGSMDateTime(DATE_FULL);
  DBG("Current Network Time:", time);

  // modem.gprsDisconnect();
  // light_sleep(5);
  // if (!modem.isGprsConnected()) {
  //   DBG("GPRS disconnected");
  // } else {
  //   DBG("GPRS disconnect: Failed.");
  // }
  */
}

void light_sleep(uint32_t sec)
{
  esp_sleep_enable_timer_wakeup(sec * 1000000ULL);
  esp_light_sleep_start();
}

void setup()
{
  // Set console baud rate
  SerialMon.begin(115200);
  delay(10);

  // Set GSM module baud rate
  SerialAT.begin(UART_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(10000);

  /*
    The indicator light of the board can be controlled
  */
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  /*
    MODEM_PWRKEY IO:4 The power-on signal of the modulator must be given to it,
    otherwise the modulator will not reply when the command is sent
  */
  pinMode(MODEM_PWRKEY, OUTPUT);
  digitalWrite(MODEM_PWRKEY, HIGH);
  delay(300); // Need delay
  digitalWrite(MODEM_PWRKEY, LOW);

  /*
    MODEM_FLIGHT IO:25 Modulator flight mode control,
    need to enable modulator, this pin must be set to high
  */
  pinMode(MODEM_FLIGHT, OUTPUT);
  digitalWrite(MODEM_FLIGHT, HIGH);

  Serial.println("Initializing modem...");
  bool res;
  DBG("Initializing modem...");
  if (!modem.init())
  {
    DBG("Failed to restart modem, delaying 10s and retrying");
    return;
  }

  String ret;
  //    do {
  //        ret = modem.setNetworkMode(2);
  //        delay(500);
  //    } while (ret != "OK");
  ret = modem.setNetworkMode(2);
  DBG("setNetworkMode:", ret);

  uint8_t mode = modem.getGNSSMode();
  DBG("GNSS Mode:", mode);

  modem.setGNSSMode(0, 1); // GLONASS
  modem.setGNSSMode(1, 1); // BEIDOU
  modem.setGNSSMode(2, 1); // GALILEO
  modem.setGNSSMode(3, 1); // QZSS
  light_sleep(1);

  String name = modem.getModemName();
  DBG("Modem Name:", name);

  String modemInfo = modem.getModemInfo();
  DBG("Modem Info:", modemInfo);

  // Unlock your SIM card with a PIN if needed
  if (SIM_PIN && modem.getSimStatus() != 3)
  {
    modem.simUnlock(STR(SIM_PIN));
  }

  DBG("Waiting for network...");
  if (!modem.waitForNetwork(600000L))
  {
    light_sleep(10);
    return;
  }

  if (modem.isNetworkConnected())
  {
    DBG("Network connected");
  }

  DBG("Connecting to", apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass))
  {
    light_sleep(10);
    return;
  }

  res = modem.isGprsConnected();
  DBG("GPRS status:", res ? "connected" : "not connected");

  String ccid = modem.getSimCCID();
  DBG("CCID:", ccid);

  IMEI = modem.getIMEI();
  DBG("IMEI:", IMEI);

  String imsi = modem.getIMSI();
  DBG("IMSI:", imsi);

  String cop = modem.getOperator();
  DBG("Operator:", cop);

  IPAddress local = modem.localIP();
  DBG("Local IP:", local);

  int csq = modem.getSignalQuality();
  DBG("Signal quality:", csq);

  client.setCertValidation(false);
}

void loop()
{

  // GPRS connection parameters are usually set after network registration
  SerialMon.print(F("Connecting to "));
  SerialMon.print(apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass))
  {
    SerialMon.println(" fail");
    delay(30000);
    return;
  }
  SerialMon.println(" success");

  if (modem.isGprsConnected())
  {
    SerialMon.println("GPRS connected");
  }

  float lat = 0, lon = 0, speed = 0, alt = 0, accuracy = 0;
  int vsat = 0, usat = 0, year = 0, month = 0, day = 0, hour = 0, min = 0, sec = 0;
  enableGPS();
  
  while (1)
  {
    battery = ReadBattery();
    if (modem.getGPS(&lat, &lon, &speed, &alt, &vsat, &usat, &accuracy, &year, &month, &day, &hour, &min, &sec))
    {
      Serial.print("Updating Location lat:");
      Serial.print(lat);
      Serial.print(" lon: ");
      Serial.println(lon);
      send_data(lat, lon, speed, alt, accuracy, battery);
    }
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    if (battery == 0)
    {
      SerialMon.println("USB Power");
      delay(INTERVAL_USB);
    }
    else
    {
      int count = 0;
      while ((battery > 0) and (count < 100))
      {
        battery = ReadBattery();
        delay(INTERVAL_BATTERY/100);
        count++;
      }
    }
  }
}
