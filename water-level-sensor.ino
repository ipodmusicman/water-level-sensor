/*
 * Water Level Sensor
 * 
 * Note that MQTT_MAX_PACKET_SIZE in the PubSubClient.h must be changed to 150.  The JSON message sent over MQTT contains more information that exceeds 128 bytes.
 */

#include <FS.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <WiFiManager.h>
#include <ArduinoJson.h>

#define PRESSURE_PIN 0       // Output pin from Pressure Sensor

#define PROPERTY_MQTT_SERVER_ADDRESS "mqtt_server_address"
#define PROPERTY_MQTT_SERVER_PORT "mqtt_server_port"
#define PROPERTY_TANK_HEIGHT "tank_height"
#define PROPERTY_TANK_CAPACITY "tank_capacity"

char* propertyValueMqttServerAddress;
char* propertyValueMqttServerPort;
char* propertyValueTankHeight;
char* propertyValueTankCapacity;

#define measurementTimeIntervals 10
#define averageMeasurementTimeIntervals 6

String mqttTopicStatus;
String mqttTopicData;
String mqttTopicDebug;
char* fullHostname;

const int offset = 156;  // AnalogeRead offset average is around 154 in an empty tank / tank with very little water in it.

const float fullscaleADCvoltage = 3.2; // Wemos D1’s max input on A0
const int fullscaleADCreading = 1023; // 3.2V on A0 yields analogueRead() of 1023
const int kPaPerVolt = 400; // 1.6MPa sensor outputs 1600/(4.5-0.5) = 400kPa/V
const float gravityAcceleration = 9.81;
float tankHeight;
float tankCapacity;

bool shouldSaveConfig = false;
long lastReconnectAttempt = 0;
String macAddressSuffix;
int measurementIntervalCounter = 0;
int averageMeasurementIntervalCounter = 0;
int lowestReadout = 0;
int averageReadoutOverTime = 0;
int calculationCountdownInSeconds = 120;

WiFiClient wifiClient;
PubSubClient client(wifiClient);

void setup() {
  Serial.begin (9600);
  macAddressSuffix = setupLast6ChaarctersOfMacAddress();
  trc ("MAC Address suffix " + macAddressSuffix);

  propertyValueMqttServerAddress = new char[40];
  propertyValueMqttServerPort = new char[10];
  propertyValueTankHeight = new char[10];
  propertyValueTankCapacity = new char[10];

  mqttTopicStatus = "waterlevelsensor/" + macAddressSuffix + "/status";
  mqttTopicData = "waterlevelsensor/" + macAddressSuffix + "/data";
  mqttTopicDebug = "waterlevelsensor/" + macAddressSuffix + "/debug";
  
  readConfig();
  setupWiFi();

  calculateAndSendWaterLevelMeasurement();

  sendMQTT (mqttTopicStatus, "Sleeping for 20 seconds");
  client.disconnect();

  //trc("Going into deep sleep for an hour");
  //ESP.deepSleep(3600000000UL); // 1 hour
  ESP.deepSleep(20e6); // 20e6 is 20 seconds
}

String setupLast6ChaarctersOfMacAddress() {
  String mac =  WiFi.macAddress();
  mac.replace(":","");
  String suffix = mac;
  suffix = suffix.substring (suffix.length() - 6);

  return suffix;
}

void readConfig() {
  if (SPIFFS.begin()) {
    trc("Mounted file system");
    if (SPIFFS.exists("/config.json")) {
      trc("Reading configuration file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        trc("Opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        if (json.success()) {
          strcpy(propertyValueMqttServerAddress, json[PROPERTY_MQTT_SERVER_ADDRESS]);
          strcpy(propertyValueMqttServerPort, json[PROPERTY_MQTT_SERVER_PORT]);
          strcpy(propertyValueTankHeight, json[PROPERTY_TANK_HEIGHT]);
          strcpy(propertyValueTankCapacity, json[PROPERTY_TANK_CAPACITY]);
          trc("Read configuraiton");              
        } else {
          trc("Failed to read configuration");
        }
      }
    } else {
      trc("File /config.json doesn't exist");
    }
  } else {
    trc("Failed to mount FS");
  }
}

void setupWiFi() {
  WiFiManagerParameter wpMqttServer(PROPERTY_MQTT_SERVER_ADDRESS, "MQTT Address", propertyValueMqttServerAddress, 40);
  WiFiManagerParameter wpMqttPort(PROPERTY_MQTT_SERVER_PORT, "MQTT Port", propertyValueMqttServerPort, 6);
  WiFiManagerParameter wpTankHeight(PROPERTY_TANK_HEIGHT, "Tank Height (m)", propertyValueTankHeight, 5);
  WiFiManagerParameter wpTankCapacity(PROPERTY_TANK_CAPACITY, "Tank Capacity (l)", propertyValueTankCapacity, 5);

  WiFiManager wifiManager;

  if (propertyValueMqttServerAddress == "" || propertyValueMqttServerPort == "") {
    trc("Resetting WiFi Manager");
    WiFi.disconnect();
    wifiManager.resetSettings();
    ESP.reset();
    delay(1000);
  }

  wifiManager.setSaveConfigCallback(saveConfigCallback);
  wifiManager.setConfigPortalTimeout(180);

  wifiManager.addParameter(&wpMqttServer);
  wifiManager.addParameter(&wpMqttPort);
  wifiManager.addParameter(&wpTankHeight);
  wifiManager.addParameter(&wpTankCapacity);

  if (!wifiManager.autoConnect(fullHostname, "")) {
    trc("Failed to Initialise Onboard Access Point");
    delay(3000);
    ESP.reset();
    delay(5000);
  }

  trc("WiFi Connected");

  if (shouldSaveConfig) {
    propertyValueMqttServerAddress = new char[40];
    propertyValueMqttServerPort = new char[10];
    propertyValueTankHeight = new char[10];
    propertyValueTankCapacity = new char[10];
    
    strcpy(propertyValueMqttServerAddress, wpMqttServer.getValue());
    strcpy(propertyValueMqttServerPort, wpMqttPort.getValue());
    strcpy(propertyValueTankHeight, wpTankHeight.getValue());
    strcpy(propertyValueTankCapacity, wpTankCapacity.getValue());
    
    trc("Saving configuration");
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json[PROPERTY_MQTT_SERVER_ADDRESS] = propertyValueMqttServerAddress;
    json[PROPERTY_MQTT_SERVER_PORT] = propertyValueMqttServerPort;
    json[PROPERTY_TANK_HEIGHT] = propertyValueTankHeight;
    json[PROPERTY_TANK_CAPACITY] = propertyValueTankCapacity;

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      trc("Failed to open configuration file");
    }

    json.printTo(configFile);
    configFile.close();
  }

  trc("Setting MQTT Server connection");
  unsigned int mqtt_port_x = atoi (propertyValueMqttServerPort);
  client.setServer(propertyValueMqttServerAddress, mqtt_port_x);

  tankHeight = atof(propertyValueTankHeight);
  tankCapacity = atof(propertyValueTankCapacity);
  
  reconnect();
}

void saveConfigCallback() {
  shouldSaveConfig = true;
}

boolean reconnect() {
  while (!client.connected()) {
    trc("Attempting to connect to MQTT Server");
    String mqname =  WiFi.macAddress();
    char charBuf[50];
    mqname.toCharArray(charBuf, 50) ;

    if (client.connect(charBuf)) {
      trc("Connected to MQTT Server");
    } else {
      trc("Failed to connect to MQTT Server");
      trc(String(client.state()));
      trc("Trying again in 5 seconds ...");
      delay(5000);
    }
  }
  return client.connected();
}

void trc(String msg) {
  Serial.println(msg);
}

/*
 * Every 2 seconds for 10 interations (20 seconds), we find the lowest readout from the pressure sensor
 * Every 20 seconds (for 2 minutes), we calculate the average readout value.  
 * After the 2 minutes has elapsed, we calculate all the required values and send them within the JSON message over MQTT.
 * Once done, we sleep for 20 seconds (for now during testing).
 */
void calculateAndSendWaterLevelMeasurement()
{
  while (averageMeasurementIntervalCounter < averageMeasurementTimeIntervals) {
    int readout = analogRead(PRESSURE_PIN);

    if (lowestReadout == 0 || readout < lowestReadout) {
      lowestReadout = readout;
    }
  
    if (measurementIntervalCounter >= measurementTimeIntervals) {
      averageMeasurementIntervalCounter++;
      averageReadoutOverTime = averageReadoutOverTime + lowestReadout;
  
      measurementIntervalCounter = 0;
      lowestReadout = 0;
    }

    if (averageMeasurementIntervalCounter >= averageMeasurementTimeIntervals) {
      float averageReadout = averageReadoutOverTime / averageMeasurementTimeIntervals;

      float averageVoltage = averageReadout * fullscaleADCvoltage / fullscaleADCreading;
      float pressure = (averageReadout - offset) * fullscaleADCvoltage / fullscaleADCreading * kPaPerVolt;
  
      if (pressure < 0) {
        pressure = 0;
      }
      
      float headHeight = (pressure / gravityAcceleration);
      float waterVolume = ((headHeight / tankHeight) * tankCapacity);
      int waterLevelPercentage = (headHeight / tankHeight) * 100;

      if (waterLevelPercentage > 100) {
        waterLevelPercentage = 100;
      } else if (waterLevelPercentage < 0) {
        waterLevelPercentage = 0;
      }

      DynamicJsonBuffer jsonBuffer;
      JsonObject& json = jsonBuffer.createObject();
      json["pressure"] = String (pressure, 1);
      json["head_height"] = headHeight;
      json["water_volume"] = waterVolume;
      json["level"] = waterLevelPercentage;
  
      char* jsonChar = new char[200];
  
      json.printTo((char*)jsonChar, json.measureLength() + 1);

      sendMQTT (mqttTopicData, jsonChar);

      DynamicJsonBuffer jsonBuffer2;
      JsonObject& json2 = jsonBuffer2.createObject();
      json2["average_readout"] = averageReadout;
      json2["average_voltage"] = averageVoltage;
  
      jsonChar = new char[200];
  
      json2.printTo((char*)jsonChar, json2.measureLength() + 1);

      sendMQTT (mqttTopicDebug, jsonChar);
    }
    
    measurementIntervalCounter++;
    sendMQTT (mqttTopicStatus, "Publishing update in " + String (calculationCountdownInSeconds));
    calculationCountdownInSeconds -= 2;

    delay(2000);
    client.loop();
  }
}

void sendMQTT(String topic, String payload) {
  char * payloadChar = new char[200];
  payloadChar = new char[200];
  payload.toCharArray (payloadChar, payload.length() + 1);
  sendMQTT (topic, payloadChar);
}

void sendMQTT(String topic, char * payload) {
  if (!client.connected()) {
    long now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      trc("MQTT not connected.  Attemping to reconnect ...");
      if (reconnect()) {
        lastReconnectAttempt = 0;
      }
    }
  } else {
    client.loop();
  }
  char topicChar[50];
  topic.toCharArray(topicChar, 50);
  if (!client.publish(topicChar, payload)) {
    trc("Message not published");
  }
}

void loop() {
}

