// This example uses an ESP32 Development Board
// to connect to shiftr.io.
//
// You can check on your device after a successful
// connection here: https://www.shiftr.io/try.
//
// by Joël Gähwiler
// https://github.com/256dpi/arduino-mqtt

#include <WiFi.h>
#include <MQTT.h>
#include <AHT20.h>
#include <BMP280_DEV.h>                           // Include the BMP280_DEV.h library

float temperature_lres, pressure, altitude;            // Create the temperature, pressure and altitude variables
float temperature_hres;
float humidity_hres;
uint8_t aht20Flag = 0;
uint8_t bmp280Flag = 0;

AHT20 aht20;     
BMP280_DEV bmp280(Wire); 
const char ssid[] = "***";
const char pass[] = "***";

char mqttTopic[256] = "/casa/keller/weinkeller";

WiFiClient net;
MQTTClient client;

unsigned long lastMillis = 0;

void connect() {
  Serial.print("checking wifi...");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }

  Serial.print("\nconnecting...");
  while (!client.connect("arduino")) {
    Serial.print(".");
    delay(1000);
  }

  Serial.println("\nconnected!");

  client.subscribe("/hello");
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
  Serial.begin(115200);
  WiFi.begin(ssid, pass);

  // Note: Local domain names (e.g. "Computer.local" on OSX) are not supported
  // by Arduino. You need to set the IP address directly.
  client.begin("192.168.1.30", net);
  client.onMessage(messageReceived);

  connect();

  Wire.setPins(25, 26);
  Wire.begin(); //Join I2C bus

  if(bmp280.begin())                                  // Default initialisation, place the BMP280 into SLEEP_MODE 
  {
    Serial.println("BMP280 not detectet. Please check wiring");
  }
  else
  {
    Serial.println("BMP280 acknowledged.");
  }
  //bmp280.setPresOversampling(OVERSAMPLING_X4);    // Set the pressure oversampling to X4
  //bmp280.setTempOversampling(OVERSAMPLING_X1);    // Set the temperature oversampling to X1
  //bmp280.setIIRFilter(IIR_FILTER_4);              // Set the IIR filter to setting 4
  bmp280.setTimeStandby(TIME_STANDBY_2000MS);       // Set the standby time to 2 seconds
  bmp280.startNormalConversion();                   // Start BMP280 continuous conversion in NORMAL_MODE  


  //Check if the AHT20 will acknowledge
  
  if (aht20.begin() == false)
  {
    Serial.println("AHT20 not detected. Please check wiring. Freezing.");
    while (1);
  }
  Serial.println("AHT20 acknowledged.");
  
}

void loop() {
  client.loop();
  delay(10);  // <- fixes some issues with WiFi stability

  if (!client.connected()) {
    connect();
  }

  if (bmp280.getMeasurements(temperature_lres, pressure, altitude))    // Check if the measurement is complete
  {
    bmp280Flag = 1;
  }
  if (aht20.available() == true)
  {
    aht20Flag = 1;
    //Get the new temperature and humidity value
    temperature_hres = aht20.getTemperature();
    humidity_hres = aht20.getHumidity();

  }

  if(aht20Flag && bmp280Flag)
  {
    aht20Flag = 0;
    bmp280Flag = 0;

    Serial.print(temperature_lres);                    // Display the results    
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

  

  // publish a message roughly every 10 second.
  if (millis() - lastMillis > 10000) {
    lastMillis = millis();
    client.publish("/hello", "world");
    String msg = String(mqttTopic);
    msg.concat("/temperatur");
    char value[32];
    sprintf(value, "%.2f",temperature_hres);
    client.publish(msg.c_str(), value, true, 0);
    sprintf(value,"%.2f",humidity_hres);
    msg.clear();
    msg.concat(mqttTopic);
    msg.concat("/humidity");
    client.publish(msg.c_str(), value, true, 0);
    sprintf(value,"%.2f",pressure);
    msg.clear();
    msg.concat(mqttTopic);
    msg.concat("/pressure");
    client.publish(msg.c_str(), value, true, 0);
  }
}
