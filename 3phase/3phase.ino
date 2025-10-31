#include <WiFi.h>
#include <WiFiManager.h>
#include <WiFiClientSecure.h>
#include <ModbusMaster.h>
#include <PubSubClient.h>
#include "secrets.h" 

#define RX_PIN 21
#define TX_PIN 22
#define MODBUS_ID 1
#define SERIAL_BAUD 9600

ModbusMaster node;
WiFiClientSecure net;
PubSubClient awsIot(net);

struct Phase {
  float V = NAN;
  float I = NAN;
  float P = NAN;
  float E = NAN;
  float PF = NAN;
  bool active = false;
};

Phase phases[3];
float frequencyHz = NAN;
float meterPF = NAN;
float totalEnergy = NAN;

unsigned long lastReadMillis = 0;
const unsigned long READ_INTERVAL = 10000; 

float readUInt16(uint16_t addr, float divisor = 1.0) {
  for (int r = 0; r < 3; r++) {
    uint8_t res = node.readHoldingRegisters(addr, 1);
    if (res == node.ku8MBSuccess) return node.getResponseBuffer(0) / divisor;
    delay(10);
  }
  return NAN;
}

float readUInt32(uint16_t addrHigh, float divisor = 1.0) {
  for (int r = 0; r < 3; r++) {
    uint8_t res = node.readHoldingRegisters(addrHigh, 2);
    if (res == node.ku8MBSuccess) {
      uint32_t hi = node.getResponseBuffer(0);
      uint32_t lo = node.getResponseBuffer(1);
      return ((hi << 16) | lo) / divisor;
    }
    delay(10);
  }
  return NAN;
}

bool isPhaseActive(float V, float I) {
  return !isnan(V) && !isnan(I) && V > 10.0 && I > 0.01;
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("📥 Received on topic ");
  Serial.print(topic);
  Serial.print(": ");
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
    Serial.println("Connecting to AWS IoT...");
    if (awsIot.connect(THINGNAME)) {
      Serial.println("Connected to AWS IoT");
      awsIot.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);
    } else {
      Serial.print("AWS connect failed, rc=");
      Serial.println(awsIot.state());
      delay(5000);
    }
  }
}

void readAllPhases() {
  int activeCount = 0;
  float vSum = 0, iSum = 0, pSum = 0, eSum = 0, pfSum = 0;

  uint16_t V_regs[3] = {0x0100, 0x0101, 0x0102};
  uint16_t I_regs[3] = {0x0103, 0x0104, 0x0105};
  uint16_t P_regs[3] = {0x0106, 0x0107, 0x0108};
  uint16_t PF_regs[3]= {0x0116, 0x0117, 0x0118};
  uint16_t E_regs[3] = {0x011A, 0x011C, 0x011E};

  for (int ph = 0; ph < 3; ph++) {
    phases[ph].V  = readUInt16(V_regs[ph], 100.0);
    phases[ph].I  = readUInt16(I_regs[ph], 100.0);
    phases[ph].P  = readUInt16(P_regs[ph], 1.0);
    phases[ph].PF = readUInt16(PF_regs[ph], 1000.0);
    phases[ph].E  = readUInt32(E_regs[ph], 100.0);
    phases[ph].active = isPhaseActive(phases[ph].V, phases[ph].I);

    if (phases[ph].active) {
      activeCount++;
      vSum += phases[ph].V;
      iSum += phases[ph].I;
      pSum += phases[ph].P;
      eSum += phases[ph].E;
      pfSum += phases[ph].PF;
    }
  }

  frequencyHz = readUInt16(0x0115, 100.0);
  meterPF = readUInt16(0x0119, 1000.0);
  totalEnergy = readUInt32(0x0120, 100.0);

  float Vavg = (activeCount > 0) ? vSum / activeCount : 0;

  Serial.printf("Active Phases: %d | Vavg: %.2f V | I: %.3f A | Ptot: %.2f W | Etot: %.3f kWh | PF: %.3f | F: %.2f Hz\n",
                activeCount, Vavg, iSum, pSum, totalEnergy, meterPF, frequencyHz);

  for (int ph = 0; ph < 3; ph++) {
    Serial.printf("  Ph%c: V=%.2f V | I=%.3f A | P=%.2f W | E=%.3f kWh | PF=%.3f | Active=%s\n",
                  'A'+ph, phases[ph].V, phases[ph].I, phases[ph].P, phases[ph].E, phases[ph].PF,
                  phases[ph].active ? "YES" : "NO");
  }

  if (awsIot.connected()) {
    char payload[512];
    snprintf(payload, sizeof(payload),
             "{\"Vavg\":%.2f,\"I\":%.3f,\"Ptot\":%.2f,\"Etot\":%.3f,\"PF\":%.3f,\"F\":%.2f}",
             Vavg, iSum, pSum, totalEnergy, meterPF, frequencyHz);
    awsIot.publish(AWS_IOT_PUBLISH_TOPIC, payload);
  }
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
  delay(200);
  Serial.println("\n=== EnergInAI JSY-333G 3-Phase Boot ===");

  connectWiFi();

  Serial2.begin(SERIAL_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
  node.begin(MODBUS_ID, Serial2);

  connectAWS();
  lastReadMillis = millis();

  Serial.println("EnergInAI 3-phase AWS monitoring started...");
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) connectWiFi();
  if (!awsIot.connected()) connectAWS();

  if (millis() - lastReadMillis >= READ_INTERVAL) {
    lastReadMillis = millis();
    readAllPhases();
  }

  awsIot.loop();
}
