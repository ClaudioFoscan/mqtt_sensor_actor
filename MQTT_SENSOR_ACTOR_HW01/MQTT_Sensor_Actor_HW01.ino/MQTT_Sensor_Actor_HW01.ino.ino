// MQTT-Sensor-Actor EXAMPLE
// ESP32S3 Based
// by Claudio Foscan

// MQTT-Library
// by Joël Gähwiler
// https://github.com/256dpi/arduino-mqtt


//Sensors
#include <AHT20.h>
#include <BMP280_DEV.h>  // Include the BMP280_DEV.h library
AHT20 aht20;
BMP280_DEV bmp280(Wire);
float temperature_lres, pressure, altitude;  // Create the temperature, pressure and altitude variables
float temperature_hres;
float humidity_hres;
uint8_t aht20Flag = 0;
uint8_t bmp280Flag = 0;

//Network
#include <WiFi.h>
#include <MQTT.h>
WiFiClient net;
MQTTClient client;
const char ssid[] = "xxx";
const char pass[] = "xxx";
const char mqttBroker[] = "192.168.1.30";
const char mqttTopic[256] = "/casa/erdgeschoss/wohnzimmer"; //casa / keller,erdgeschoss,obergeschoss,dachgeschoss
const char HOSTNAME[] = "MQTT_SENSOR_WOHNZIMMER";

//Hardware
#define INPUT_A 4
#define INPUT_B 6
#define INPUT_C 15
#define OUTPUT_A 5
#define OUTPUT_B 7
#define OUTPUT_C 16
#define RELAIS 17
#define LED_1 18
#define MODE_SW_4 42
#define MODE_SW_3 41
#define MODE_SW_2 40
#define MODE_SW_1 39
#define I2C_SDA 8
#define I2C_SCL 9

//support
#define PUBLISH_CYCLE_FAST 10000   // [ms] publishes every x 10 Seconds the new Values
#define PUBLISH_CYCLE_SLOW 600000  // [ms] publishes every 10 Minutes the new Values
#define HEARTBEAD_RATE 1000
unsigned long lastMillis = 0;
unsigned long publishCycle = 0;
unsigned long lastHeartBeatMillis = 0;

void connect() {
  Serial.print("checking wifi...");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }

  Serial.print("\nconnecting...");
  while (!client.connect("ESP32 MQTT-Sensor-Actor")) {
    Serial.print(".");
    delay(1000);
  }

  Serial.println("\nconnected!");

  // Example for subscribe to a topic
  // client.subscribe("/hello");
  // client.unsubscribe("/hello");
}

void messageReceived(String &topic, String &payload) {
  Serial.println("incoming: " + topic + " - " + payload);

  // Note: Do not use the client in the callback to publish, subscribe or
  // unsubscribe as it may cause deadlocks when other things arrive while
  // sending and receiving acknowledgments. Instead, change a global variable,
  // or push to a queue and handle it in the loop after calling `client.loop()`.
}

void setup() {
  //init HW
  pinMode(INPUT_A, INPUT_PULLUP);
  pinMode(INPUT_B, INPUT_PULLUP);
  pinMode(INPUT_B, INPUT_PULLUP);
  pinMode(MODE_SW_1, INPUT_PULLUP);
  pinMode(MODE_SW_2, INPUT_PULLUP);
  pinMode(MODE_SW_3, INPUT_PULLUP);
  pinMode(MODE_SW_4, INPUT_PULLUP);
  pinMode(OUTPUT_A, OUTPUT);
  digitalWrite(OUTPUT_A, LOW);
  pinMode(OUTPUT_B, OUTPUT);
  digitalWrite(OUTPUT_B, LOW);
  pinMode(OUTPUT_C, OUTPUT);
  digitalWrite(OUTPUT_C, LOW);
  pinMode(RELAIS, OUTPUT);
  digitalWrite(RELAIS, LOW);
  pinMode(LED_1, OUTPUT);
  digitalWrite(LED_1, LOW);

  Serial.begin(115200);
  WiFi.setHostname(HOSTNAME);
  WiFi.begin(ssid, pass);

  // Note: Local domain names (e.g. "Computer.local" on OSX) are not supported
  // by Arduino. You need to set the IP address directly.
  client.begin(mqttBroker, net);
  client.onMessage(messageReceived);

  connect();

  Wire.setPins(I2C_SDA, I2C_SCL);
  Wire.begin();  //Join I2C bus

  if (!bmp280.begin())  // Default initialisation, place the BMP280 into SLEEP_MODE
  {
    Serial.println("BMP280 not detectet. Please check wiring");
  } else {
    Serial.println("BMP280 acknowledged.");
  }
  //bmp280.setPresOversampling(OVERSAMPLING_X4);    // Set the pressure oversampling to X4
  //bmp280.setTempOversampling(OVERSAMPLING_X1);    // Set the temperature oversampling to X1
  //bmp280.setIIRFilter(IIR_FILTER_4);              // Set the IIR filter to setting 4
  bmp280.setTimeStandby(TIME_STANDBY_2000MS);  // Set the standby time to 2 seconds
  bmp280.startNormalConversion();              // Start BMP280 continuous conversion in NORMAL_MODE


  //Check if the AHT20 will acknowledge

  if (aht20.begin() == false) {
    Serial.println("AHT20 not detected. Please check wiring. Freezing.");
    while (1)
      ;
  }
  Serial.println("AHT20 acknowledged.");

  if (digitalRead(MODE_SW_4) == LOW) {
    publishCycle = PUBLISH_CYCLE_FAST;
    Serial.println("Sensor set to FAST Publishcycle");
  } else {
    publishCycle = PUBLISH_CYCLE_SLOW;
    Serial.println("Sensor set to SLOW Publishcycle");
  }
}

void loop() {
  client.loop();
  delay(10);  // <- fixes some issues with WiFi stability

  if (!client.connected()) {
    connect();
  }

  if (bmp280.getMeasurements(temperature_lres, pressure, altitude))  // Check if the measurement is complete
  {
    bmp280Flag = 1;
  }
  if (aht20.available() == true) {
    aht20Flag = 1;
    //Get the new temperature and humidity value
    temperature_hres = aht20.getTemperature();
    humidity_hres = aht20.getHumidity();
  }

  if (aht20Flag && bmp280Flag) {
    aht20Flag = 0;
    bmp280Flag = 0;

    Serial.print(temperature_lres);  // Display the results
    Serial.print(F("*C   "));
    Serial.print(pressure);
    Serial.print(F("hPa   "));
    Serial.print(altitude);
    Serial.println(F("m"));

    Serial.print("Temperature HRES: ");
    Serial.print(temperature_hres, 2);
    Serial.print(" C\t");
    Serial.print("Humidity HRES: ");
    Serial.print(humidity_hres, 2);
    Serial.print("% RH");

    Serial.println();
  }

  //heartbeat
  if (millis() - lastHeartBeatMillis > HEARTBEAD_RATE) {
    lastHeartBeatMillis = millis();
    if (digitalRead(LED_1)) {
      digitalWrite(LED_1, LOW);
    } else {
      digitalWrite(LED_1, HIGH);
    }
  }

  // publish a message roughly every x second.
  if (millis() - lastMillis > publishCycle) {
    lastMillis = millis();
    String msg = String(mqttTopic);
    msg.concat("/temperatur");
    char value[32];
    sprintf(value, "%.2f", temperature_hres);
    client.publish(msg.c_str(), value, true, 0);
    sprintf(value, "%.2f", humidity_hres);
    msg.clear();
    msg.concat(mqttTopic);
    msg.concat("/humidity");
    client.publish(msg.c_str(), value, true, 0);
    sprintf(value, "%.2f", pressure);
    msg.clear();
    msg.concat(mqttTopic);
    msg.concat("/pressure");
    client.publish(msg.c_str(), value, true, 0);
  }
}