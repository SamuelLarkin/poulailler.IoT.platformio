// [ESP8266 DS18B20 Temperature Sensor with Arduino IDE (Single, Multiple, Web Server)](https://randomnerdtutorials.com/esp8266-ds18b20-temperature-sensor-web-server-with-arduino-ide/)
// [Arduino the Object Oriented way](http://paulmurraycbr.github.io/ArduinoTheOOWay.html)
// [Multicast Domain Name System](https://tttapa.github.io/ESP8266/Chap08%20-%20mDNS.html)
// [ESPAsyncWebServer](https://github.com/me-no-dev/ESPAsyncWebServer)
// [Path variable](https://github.com/me-no-dev/ESPAsyncWebServer#path-variable)

// Import required libraries
#include <Arduino.h>

#ifdef M5ATOM
#include <FastLED.h>
#endif

#ifdef ESP32
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPmDNS.h>
#include <ESPAsyncWebServer.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESP8266mDNS.h>        // Include the mDNS library
#include <ESPAsyncWebServer.h>
#endif

#include <Adafruit_Sensor.h>
#include <ArduinoJson.h>   // [ArduinoJSON](https://arduinojson.org)
#include <DallasTemperature.h>
#include <DHT.h>
#include <OneWire.h>
#include <PubSubClient.h>   // MQTT Client
#include <SPIFFS.h>

#include "temps.h"
#include "credential.h"

// Replace with your network credentials
/*
  const char* ssid="SSID";
  const char* password="SSID_PASSWORD";
  const char* mqtt_server = "192.168.XX.YY";
*/

#ifdef M5ATOM
#define LED_TYPE WS2812
#define COLOR_ORDER GRB
#define BRIGHTNESS 20
// Internal LED controller
#define NUM_LEDS    1
CRGB ledAtom[NUM_LEDS];
#endif

StaticJsonDocument<1024> metriques;

const bool send_to_MQTT = true;
char mqtt_buffer[1024];

// Defined in plaformio.ini
// DHT 22 (AM2302)
DHT dht(DHTPIN, DHT22);

// current temperature & humidity, updated in loop()
float DHT22_temperature = -100.0;
float DHT22_humidity    = -100.0;


/*
08:28:41.098 -> ROM = 28 9E 7F 19 3A 19 1 A1
08:28:41.132 ->   Chip = DS18B20
08:28:42.114 ->   Data = 1 50 1 4B 46 1F FF 1F 10 C1  CRC=C1
08:28:42.148 ->   Temperature = 21.00 Celsius, 69.80 Fahrenheit

08:28:42.182 -> ROM = 28 C1 CE 23 3A 19 1 7A
08:28:42.215 ->   Chip = DS18B20
08:28:43.130 ->   Data = 1 50 1 4B 46 1F FF 1F 10 C1  CRC=C1
08:28:43.164 ->   Temperature = 21.00 Celsius, 69.80 Fahrenheit

08:28:43.232 -> ROM = 28 71 D3 D 3A 19 1 88
08:28:43.266 ->   Chip = DS18B20
08:28:44.178 ->   Data = 1 49 1 4B 46 7F FF C 10 D5  CRC=D5
08:28:44.212 ->   Temperature = 20.56 Celsius, 69.01 Fahrenheit

08:28:44.280 -> ROM = 28 D5 2C 18 3A 19 1 78
08:28:44.280 ->   Chip = DS18B20
08:28:45.192 ->   Data = 1 55 1 4B 46 7F FF C 10 BE  CRC=BE
08:28:45.260 ->   Temperature = 21.31 Celsius, 70.36 Fahrenheit

08:28:45.294 -> ROM = 28 D7 9A F0 39 19 1 74
08:28:45.327 ->   Chip = DS18B20
08:28:46.241 ->   Data = 1 55 1 4B 46 7F FF C 10 BE  CRC=BE
08:28:46.275 ->   Temperature = 21.31 Celsius, 70.36 Fahrenheit

08:28:46.343 -> No more addresses.
*/
DeviceAddress DS18B20_address[] = {
  { 0x28, 0x71, 0xD3, 0x0D, 0x3A, 0x19, 0x01, 0x88 },
  { 0x28, 0x9E, 0x7F, 0x19, 0x3A, 0x19, 0x01, 0xA1 },
  { 0x28, 0xC1, 0xCE, 0x23, 0x3A, 0x19, 0x01, 0x7A },
  { 0x28, 0xD5, 0x2C, 0x18, 0x3A, 0x19, 0x01, 0x78 },
  { 0x28, 0xD7, 0x9A, 0xF0, 0x39, 0x19, 0x01, 0x74 },
};

// GPIO where the DS18B20 is connected to
// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONEWIREBUS);

// Pass our oneWire reference to Dallas Temperature sensor
DallasTemperature sensors(&oneWire);

float DS18B20_temperature = -100.0;


// Create AsyncWebServer object on port 80
AsyncWebServer web_server(80);

WiFiClient espClient;
PubSubClient mqtt_client(espClient);


// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;    // will store last time DHT was updated

// Updates DHT readings every 10 seconds
const long interval = 10000;




void setup_mDNS() {
  if (!MDNS.begin("poulailler")) {             // Start the mDNS responder for poulailler.local
    Serial.println(F("Error setting up MDNS responder!"));
  }
  else {
    Serial.println(F("mDNS responder started"));
  }
}



void setup_wifi() {
  delay(500);

  int led_state = 0;
  // https://github.com/esikora/ESP32App_Led_IR/blob/master/ESP32App_Led_IR.ino
  //FastLED.addLeds<NEOPIXEL, LED_BUILTIN>(ledAtom, NUM_LEDS);
  //FastLED.addLeds<LED_TYPE, LED_BUILTIN, GRB>(ledAtom, NUM_LEDS);
  FastLED.addLeds<LED_TYPE, LED_BUILTIN>(ledAtom, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
  ledAtom[0] = CRGB::Green;
  FastLED.show();

  // We start by connecting to a WiFi network
  Serial.printf("\nConnecting to %s\n", ssid);

  WiFi.begin(ssid, password);
#ifdef ESP32
  WiFi.setHostname("poulailler");
#else
  WiFi.hostname("poulailler");
#endif

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(F("."));

    // Blink the builtin led until we are connected to WiFi. This is an external
    // queue to manually reset the device.
    if (++led_state % 2 == 0) {
      ledAtom[0] = CRGB::Black;
    }
    else {
      ledAtom[0] = CRGB::Green;
    }
    FastLED.show();
  }

  randomSeed(micros());

  Serial.println(F("\nWiFi connected"));
  Serial.printf("IP address: %s\n", WiFi.localIP().toString().c_str());
  delay(1000);
  ledAtom[0] = CRGB::Black;
  FastLED.show();

  setup_mDNS();
}



void setup_web_server(AsyncWebServer& web_server) {
  // Route for root / web page
  //web_server.serveStatic("/", SPIFFS, "/").setDefaultFile("/index.html");
  //web_server.serveStatic("/", SPIFFS, "/index.html");
  web_server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/index.html", "text/html");
  });

  web_server.on("/DHT22/temperature", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, F("text/html"), String(DHT22_temperature).c_str());
  });

  web_server.on("/DHT22/humidity", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, F("text/html"), String(DHT22_humidity).c_str());
  });

  web_server.on("/DS18B20/temperature", HTTP_GET, [](AsyncWebServerRequest * request) {
    String json;
    serializeJsonPretty(metriques[F("DS18B20")], json);
    request->send_P(200, F("application/json"), json.c_str());
  });

  web_server.on("/metriques", HTTP_GET, [](AsyncWebServerRequest * request) {
    String json;
    serializeJsonPretty(metriques, json);
    request->send_P(200, F("application/json"), json.c_str());
  });

#ifdef ASYNCWEBSERVER_REGEX
  web_server.on("^\\/sensor\\/([0-9]+)$", HTTP_GET, [] (AsyncWebServerRequest * request) {
    const String sensorId = request->pathArg(0);
    request->send_P(200, F("text/html"), sensorId.c_str());
  });
#endif

  web_server.onNotFound([](AsyncWebServerRequest * request) {
    request->send(200, F("text/html"), F("Il n'y a pas de page"));
  });

  // Start server
  web_server.begin();
}



void setup_serial() {
  // Serial port for debugging purposes
  Serial.begin(115200);
  // wait for serial monitor to open
  while (! Serial);
}



// function to print a device address
String deviceAddressToString(const DeviceAddress& deviceAddress) {
  String mac;
  mac.reserve(16);
  for (uint8_t i = 0; i < 8; i++) {
    // zero pad the address if necessary
    if (deviceAddress[i] < 16) mac += "0";
    mac += String(deviceAddress[i], HEX);
  }

  return mac;
}



void setup_sensors(DHT& dht, DallasTemperature& sensors) {
  dht.begin();
  {
    JsonObject data = metriques.createNestedObject(F("DHT22"));
    data[F("humidity")]    = -44.0f;
    data[F("temperature")] = -45.0f;
  }

  // Start the DS18B20 sensor
#ifdef M5ATOM
  oneWire.begin(ONEWIREBUS);
  delay(100);
  sensors.begin();   // WTF Why do we need to call `sensors.begin()` twice?
#endif

  sensors.begin();
  sensors.setResolution(9);
  delay(1200);
  {
    const unsigned int num_DS18B20 = sensors.getDeviceCount();
    JsonArray data = metriques.createNestedArray(F("DS18B20"));
    for (unsigned int i=0; i<num_DS18B20; ++i) {
      auto m = data.createNestedObject();
      DeviceAddress mac;
      sensors.getAddress(mac, i);
      m["mac"] = deviceAddressToString(mac);
      m["temperature"] = -120.0f;
    }
  }

  serializeJson(metriques, Serial);
  Serial.println();
}



void setup() {
  delay(1000);

  setup_serial();
  SPIFFS.begin();

  setup_sensors(dht, sensors);
  setup_wifi();
  setup_web_server(web_server);
  setup_time();

  mqtt_client.setServer(mqtt_server, 1883);
}



void reconnect_mqtt() {
  // Loop until we're reconnected
  while (!mqtt_client.connected()) {
    Serial.print(F("Attempting MQTT connection..."));
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (mqtt_client.connect(clientId.c_str())) {
      Serial.println(F("connected"));
      // Once connected, publish an announcement...
      //mqtt_client.publish("poulailler", "hello world");
      // ... and resubscribe
      //mqtt_client.subscribe("inTopic");
    }
    else {
      Serial.print(F("failed, rc="));
      Serial.print(mqtt_client.state());
      Serial.println(F(" try again in 5 seconds"));
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}




void updateDHT22(const unsigned long currentMillis) {
  // Read temperature as Celsius (the default)
  const float newT = dht.readTemperature();
  // if temperature read failed, don't change t value
  if (isnan(newT)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    DHT22_temperature = -200.0;
  }
  else {
    DHT22_temperature = newT;
    Serial.printf("%s - DHT22: %f°C\n", String(currentMillis).c_str(), DHT22_temperature);
  }

  // Read humidity
  const float newH = dht.readHumidity();
  // if humidity read failed, don't change h value
  if (isnan(newH)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    DHT22_humidity = -200.0;
  }
  else {
    DHT22_humidity = newH;
    Serial.printf("%s - DHT22: %f%%H\n", String(currentMillis).c_str(), DHT22_humidity);
  }

  JsonObject data = metriques[F("DHT22")];
  data[F("humidity")]    = DHT22_humidity;
  data[F("temperature")] = DHT22_temperature;

  if (send_to_MQTT) {
    if (false) {
      serializeJson(data, mqtt_buffer);
      Serial.println(mqtt_buffer);
      const bool response1 = mqtt_client.publish("poulailler/DHT22", mqtt_buffer);
    }
    const bool temperature_response = mqtt_client.publish("poulailler/temperature/DHT22/0", String(DHT22_temperature).c_str());
    const bool humidity_response    = mqtt_client.publish("poulailler/humidity/DHT22/0", String(DHT22_humidity).c_str());
    //Serial.println(response ? F("MQTT DHT22 Success") : F("MQTT DHT22 Failed"));
  }
}



void updateDS18B20(const unsigned long currentMillis) {
  Serial.printf("Wait for convertion: %d\n", sensors.getWaitForConversion());  // Debugging
  Serial.printf("Sensors' resolution: %d\n", sensors.getResolution());   // Debugging
  Serial.printf("Wait %dms\n", sensors.millisToWaitForConversion(sensors.getResolution()));   // Debugging
  Serial.printf("Power Supply: %f\n", sensors.readPowerSupply(DS18B20_address[0]));   // Debugging

  sensors.requestTemperatures();
  //delay(750);  // This seems critical to get the sensors to work with AtomLite
  delay(sensors.millisToWaitForConversion(sensors.getResolution()));

  const unsigned int num_DS18B20 = sensors.getDeviceCount();
  Serial.printf("Number of DS18B20: %d\n", num_DS18B20);

  JsonArray data = metriques[F("DS18B20")];
  for (unsigned int i=0; i<num_DS18B20; ++i) {
    auto m = data[i];
    DeviceAddress mac;
    sensors.getAddress(mac, i);
    delay(10);
    const String mac_s      = deviceAddressToString(mac);
    const float temperature = sensors.getTempCByIndex(i);
    //Serial.printf("%s - %f\n", mac_s.c_str(), temperature);
    m[F("mac")]         = mac_s;
    m[F("temperature")] = temperature;

    if (send_to_MQTT) {
      // NOTE:
      // [MAX_PACKET_SIZE value in PubSubClient.h? That payload looks like it'll exceed the default 128 bytes](https://github.com/knolleary/pubsubclient/issues/382#issuecomment-367408738)
      if (false) {
        const size_t s = serializeJson(m[F("temperature")], mqtt_buffer);
        assert(s < 1024);
        Serial.println(mqtt_buffer);
      }
      String topic(F("poulailler/temperature/DS18B20/"));
      //topic += i;
      //topic = topic + String(m[F("mac")]);
      topic += mac_s;
      const bool response = mqtt_client.publish(topic.c_str(), String(temperature).c_str());
      //Serial.println(response ? F("MQTT DS18B20 Success") : F("MQTT DS18B20 Failed"));
    }
  }
}



void loop() {
  if (!mqtt_client.connected()) {
    reconnect_mqtt();
  }
  mqtt_client.loop();

  const unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    // save the last time you updated the DHT values
    previousMillis = currentMillis;

    Serial.printf("DHT22 pin: %d\n", DHTPIN);
    Serial.printf("DS18B20 pin: %d\n", ONEWIREBUS);

    updateDHT22(currentMillis);

    updateDS18B20(currentMillis);

#ifdef ASYNCWEBSERVER_REGEX
  Serial.println("REGEX");
#else
  Serial.println("NO REGEX");
#endif

    printLocalTime();
    
    // NOTE: It may take some time to update the NTP time.
    //time_t now;
    time_t now = time(nullptr);
    // [struct tm](https://github.com/esp8266/Arduino/blob/master/tools/sdk/libc/xtensa-lx106-elf/include/time.h)
    struct tm * timeinfo;
    time(&now);
    timeinfo = localtime(&now);  
    char local_time[32];
    const size_t a = strftime(local_time, 30, "%Y-%m-%d %H:%M:%S %Z", timeinfo);
    // The number of seconds since the Epoch, 1970-01-01 00:00:00 +0000 (UTC). (TZ) (Calculated from mktime(tm).)
    //size_t strftime(local_time, 30, "%s", timeinfo);

    //Serial.println(timeinfo->tm_hour);
    //Serial.printf("tm_min: %d\n", timeinfo->tm_min);
    //Serial.println(timeinfo);

    metriques[F("time")] = local_time;

    serializeJsonPretty(metriques, Serial);
    Serial.println();
    Serial.println();
  }
}