#include <WiFi.h>
#include <WiFiManager.h>
#include <WiFiClientSecure.h>
#include <ModbusMaster.h>
#include <PubSubClient.h>
#include "secrets.h"

#define RX_PIN 16
#define TX_PIN 17
#define MODBUS_ID 1
#define SERIAL_BAUD 4800

ModbusMaster node;
WiFiClientSecure net;
PubSubClient awsIot(net);

const unsigned long READ_INTERVAL = 10000;
unsigned long lastReadMillis = 0;

float voltage = NAN, current = NAN, power = NAN, energy = NAN;
float powerFactor = NAN, frequency = NAN;

String deviceId = "EnergInAI_001";

float readFloat32(uint16_t addr, float divisor) {
  uint8_t result = node.readHoldingRegisters(addr, 2);
  if (result == node.ku8MBSuccess) {
    uint32_t hi = node.getResponseBuffer(0);
    uint32_t lo = node.getResponseBuffer(1);
    uint32_t raw = ((uint32_t)hi << 16) | lo;
    return raw / divisor;
  }
  return NAN;
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Received: ");
  for (int i = 0; i < length; i++) Serial.print((char)payload[i]);
  Serial.println();
}

void connectAWS() {
  net.setCACert(AWS_IOT_ROOT_CA);
  net.setCertificate(AWS_IOT_DEVICE_CERT);
  net.setPrivateKey(AWS_IOT_PRIVATE_KEY);
  awsIot.setServer(AWS_IOT_ENDPOINT_ADDR, 8883);
  awsIot.setCallback(mqttCallback);
  while (!awsIot.connected()) {
    if (awsIot.connect(deviceId.c_str())) {
      awsIot.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);
    } else {
      delay(5000);
    }
  }
}

String getTimestampRaw() {
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return "";
  char buf[20];
  strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
  return String(buf);
}

String getTimestampDisplay() {
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return "";
  char buf[30];
  strftime(buf, sizeof(buf), "%B %d, %Y, %H:%M:%S", &timeinfo);
  return String(buf);
}

void readAll() {
  voltage     = readFloat32(0x0048, 10000.0);
  current     = readFloat32(0x0049, 10000.0);
  power       = readFloat32(0x004A, 10000.0);
  energy      = readFloat32(0x004B, 10000.0);
  powerFactor = readFloat32(0x004C, 1000.0);
  frequency   = readFloat32(0x004F, 100.0);

  if (current < 0.01) {
    voltage     = readFloat32(0x0050, 10000.0);
    current     = readFloat32(0x0051, 10000.0);
    power       = readFloat32(0x0052, 10000.0);
    energy      = readFloat32(0x0053, 10000.0);
    powerFactor = readFloat32(0x0054, 1000.0);
  }

  String timestampRaw     = getTimestampRaw();
  String timestampDisplay = getTimestampDisplay();

  char payload[512];
  snprintf(payload, sizeof(payload),
           "{\"deviceId\":\"%s\",\"timestampRaw\":\"%s\",\"timestampDisplay\":\"%s\","
           "\"V\":%.2f,\"I\":%.3f,\"P\":%.2f,\"kWh\":%.3f,\"PF\":%.3f,\"F\":%.2f}",
           deviceId.c_str(), timestampRaw.c_str(), timestampDisplay.c_str(),
           voltage, current, power, energy, powerFactor, frequency);

  if (awsIot.connected()) {
    awsIot.publish(AWS_IOT_PUBLISH_TOPIC, payload);
  }

  Serial.println(payload);
}

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname("EnergInAI");
  WiFiManager wm;
  wm.setConnectTimeout(30);
  wm.setConfigPortalTimeout(180);
  if (!wm.autoConnect(WIFI_AP_SSID, WIFI_AP_PASSWORD)) ESP.restart();
}

void setup() {
  Serial.begin(115200);
  delay(500);
  connectWiFi();
  Serial2.begin(SERIAL_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
  node.begin(MODBUS_ID, Serial2);
  configTime(19800, 0, "pool.ntp.org"); 
  connectAWS();
  lastReadMillis = millis();
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) connectWiFi();
  if (!awsIot.connected()) connectAWS();
  unsigned long now = millis();
  if (now - lastReadMillis >= READ_INTERVAL) {
    lastReadMillis = now;
    readAll();
  }
  awsIot.loop();
}
