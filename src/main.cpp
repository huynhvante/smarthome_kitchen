#include <Arduino.h>
#include "DHT.h"
#include <WiFi.h>
#include <esp_now.h>

#define DHTPIN 4
#define MQ2_PIN 5

#define LED_PIN 36

#define INA 18
#define INB 19

#define PWM_A 0
#define PWM_B 1
#define PWM_FREQ 5000
#define PWM_RES 8
#define FAN_SPEED 200

#define SENSOR_INTERVAL_MS 2000UL
#define FAN_OFF_DELAY_MS 60000UL
#define GAS_RAW_THRESHOLD 2200

DHT dht;

uint8_t gatewayMAC[] = {0x1C, 0xDB, 0xD4, 0xCB, 0xCE, 0xE8};

typedef struct __attribute__((packed)) {
  float temp;
  float humid;
  float gas_voltage;
  int gas_raw;
  bool fan;
  bool led;
  bool alarm;
} SensorData;

typedef struct __attribute__((packed)) {
  char command[20];
} ControlData;

SensorData sensorData;

bool manualMode = false;
bool alertMode = false;
bool fanState = false;
bool ledState = false;
unsigned long lastSensorMs = 0;

void fanOn()
{
  ledcWrite(PWM_A, 0);
  ledcWrite(PWM_B, FAN_SPEED);
  fanState = true;
}

void fanOff()
{
  ledcWrite(PWM_A, 0);
  ledcWrite(PWM_B, 0);
  fanState = false;
}

void ledOn()
{
  digitalWrite(LED_PIN, HIGH);
  ledState = true;
}

void ledOff()
{
  digitalWrite(LED_PIN, LOW);
  ledState = false;
}

void applyCommand(const char *command)
{
  if (strcmp(command, "FAN_ON") == 0 || strcmp(command, "ON") == 0) {
    manualMode = true;
    fanOn();
  } else if (strcmp(command, "FAN_OFF") == 0 || strcmp(command, "OFF") == 0) {
    manualMode = true;
    fanOff();
  } else if (strcmp(command, "LED_ON") == 0) {
    manualMode = true;
    ledOn();
  } else if (strcmp(command, "LED_OFF") == 0) {
    manualMode = true;
    ledOff();
  } else if (strcmp(command, "ALERT_ON") == 0) {
    alertMode = true;
    fanOn();
    ledOn();
  } else if (strcmp(command, "ALERT_OFF") == 0) {
    alertMode = false;
    ledOff();
  } else if (strcmp(command, "AUTO") == 0) {
    manualMode = false;
    alertMode = false;
  }
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len)
{
  if (len != sizeof(ControlData)) {
    return;
  }

  ControlData cmd;
  memcpy(&cmd, incomingData, sizeof(cmd));
  cmd.command[sizeof(cmd.command) - 1] = '\0';

  Serial.print("Command: ");
  Serial.println(cmd.command);
  applyCommand(cmd.command);
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.print("Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAIL");
}

void readSensor(float &t, float &h, int &gasRaw, float &gasVoltage)
{
  t = dht.getTemperature();
  h = dht.getHumidity();
  gasRaw = analogRead(MQ2_PIN);
  gasVoltage = gasRaw * (3.3f / 4095.0f);

}

void sendData(float t, float h, int gasRaw, float gasVoltage)
{
  sensorData.temp = t;
  sensorData.humid = h;
  sensorData.gas_voltage = gasVoltage;
  sensorData.gas_raw = gasRaw;
  sensorData.fan = fanState;
  sensorData.led = ledState;
  sensorData.alarm = alertMode;

  esp_now_send(gatewayMAC, (uint8_t *)&sensorData, sizeof(sensorData));
}

void initESPNow()
{
  WiFi.mode(WIFI_STA);
  Serial.print("Node MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed");
    while (true);
  }

  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, gatewayMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Add Peer Failed");
    while (true);
  }

  Serial.println("ESP-NOW READY");
}

void setup()
{
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  ledOff();

  ledcSetup(PWM_A, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_B, PWM_FREQ, PWM_RES);
  ledcAttachPin(INA, PWM_A);
  ledcAttachPin(INB, PWM_B);
  fanOff();

  dht.setup(DHTPIN, DHT::DHT22);

  initESPNow();
  Serial.println("KITCHEN NODE READY");
}

void loop()
{
  unsigned long now = millis();
  if (now - lastSensorMs < SENSOR_INTERVAL_MS) {
    return;
  }
  lastSensorMs = now;

  float t, h, gasVoltage;
  int gasRaw;
  readSensor(t, h, gasRaw, gasVoltage);

  Serial.printf("T=%.1f H=%.1f GAS_RAW=%d GAS=%.2fV\n",
                t, h, gasRaw, gasVoltage);
  sendData(t, h, gasRaw, gasVoltage);

  if (!manualMode) {
    bool gasDanger = gasRaw >= GAS_RAW_THRESHOLD;

    if (alertMode || gasDanger) {
      fanOn();
    } else {
      fanOff();
    }

    if (alertMode || gasDanger) {
      ledOn();
    } else {
      ledOff();
    }
  }
}