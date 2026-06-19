#include <Arduino.h>
#include "DHT.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ───── WIFI ─────
const char* WIFI_SSID = "Xuan Tri";
const char* WIFI_PASS = "08041992";

// ───── MQTT ─────
const char* MQTT_BROKER = "192.168.1.8";
const int MQTT_PORT = 1883;
const char* MQTT_ID = "esp32_kitchen";

// ───── TOPIC ─────
  // publish
const char* T_TEMP  = "smarthome/kitchen/temp";
const char* T_GAS   = "smarthome/kitchen/gas";
  // subscribe
const char* T_LED   = "smarthome/kitchen/alert_led";

const char* T_STATUS = "smarthome/kitchen/status";

// ───── PIN ─────
#define DHTPIN 4
#define MQ2_PIN 5

#define LED_PIN 36
#define BUZZER 45

#define INA 18
#define INB 19

#define PWM_A 0
#define PWM_B 1
#define PWM_FREQ 5000
#define PWM_RES 8
#define FAN_SPEED 200

DHT dht;
WiFiClient wifi;
PubSubClient mqtt(wifi);

// ───── STATE ─────
bool manualMode = false;
bool fanState = false;
bool ledState = false;
bool buzzerState = false;

// ───── WIFI ─────
void wifiConnect() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.print("WiFi connecting");
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi OK");
    Serial.print("ESP IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("Gateway: ");
    Serial.println(WiFi.gatewayIP());
  } else {
    Serial.println("\nWiFi FAILED!");
  }
}

// ───── ACTUATOR ─────
void fanOn() {
  ledcWrite(PWM_A, 0);
  ledcWrite(PWM_B, FAN_SPEED);
  fanState = true;
}

void fanOff() {
  ledcWrite(PWM_A, 0);
  ledcWrite(PWM_B, 0);
  fanState = false;
}

// ───── MQTT CALLBACK ─────
void callback(char* topic, byte* payload, unsigned int len) {

  char msg[32];
  memcpy(msg, payload, len);
  msg[len] = '\0';

  Serial.printf("[MQTT] %s -> %s\n", topic, msg);

  // Control LED, Fan, Buzzer via alert_led topic
  if (strcmp(topic, T_LED) == 0) {
    if (strcmp(msg, "ON") == 0) {
      manualMode = true;
      fanOn();
      digitalWrite(LED_PIN, HIGH);
      digitalWrite(BUZZER, HIGH);
      ledState = true;
      buzzerState = true;
    } else if (strcmp(msg, "OFF") == 0) {
      manualMode = true;
      fanOff();
      digitalWrite(LED_PIN, LOW);
      digitalWrite(BUZZER, LOW);
      ledState = false;
      buzzerState = false;
    } else if (strcmp(msg, "AUTO") == 0) {
      manualMode = false;
    }
  }
}


// ───── MQTT CONNECT ─────
void mqttConnect() {
  Serial.printf("MQTT: Attempting to connect to %s:%d\n", MQTT_BROKER, MQTT_PORT);
  
  while (!mqtt.connected()) {
    Serial.print("MQTT connecting...");

    if (mqtt.connect(MQTT_ID)) {
      Serial.println("OK");

      mqtt.subscribe(T_LED);

      mqtt.publish(T_STATUS, "online", true);
    } else {
      Serial.print("FAIL rc=");
      Serial.print(mqtt.state());
      Serial.println(" (0=MQTT_CONNECTED, -1=MQTT_CONNECT_FAILED, -2=MQTT_NOT_CONNECTED, -3=MQTT_BROKEN_WIRE, -4=MQTT_REFRESH_REQUIRED)");
      delay(5000);  // Increased delay to give broker time to respond
    }
  }
}

// ───── SENSOR ─────
void readSensor(float &t, float &h, float &gasV) {
  t = dht.getTemperature();
  h = dht.getHumidity();

  int raw = analogRead(MQ2_PIN);
  gasV = raw * (3.3f / 4095.0f);
}

// ───── PUBLISH ─────
void publish(float t, float h, float gasV) {

  StaticJsonDocument<128> doc;
  char buf[128];

  doc["temp"] = t;
  doc["humid"] = h;
  serializeJson(doc, buf);
  mqtt.publish(T_TEMP, buf);

  doc.clear();
  doc["voltage"] = gasV;
  doc["raw"] = analogRead(MQ2_PIN);
  serializeJson(doc, buf);
  mqtt.publish(T_GAS, buf);
}

// ───── SETUP ─────
void setup() {
  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  ledcSetup(PWM_A, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_B, PWM_FREQ, PWM_RES);
  ledcAttachPin(INA, PWM_A);
  ledcAttachPin(INB, PWM_B);

  dht.setup(DHTPIN, DHT::DHT22);

  wifiConnect();
  
  // Wait for WiFi to stabilize before connecting to MQTT
  delay(2000);

  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  mqtt.setCallback(callback);

  mqttConnect();

  Serial.println("ESP32 READY");
}

// ───── LOOP ─────
void loop() {

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[ERROR] WiFi disconnected, reconnecting...");
    wifiConnect();
    delay(2000);  // Wait for network to stabilize
  }
  
  if (!mqtt.connected()) {
    mqttConnect();
  }

  mqtt.loop();

  float t, h, gas;
  readSensor(t, h, gas);

  Serial.printf("T=%.1f H=%.1f GAS=%.2fV\n", t, h, gas);

  publish(t, h, gas);

  // AUTO MODE fallback
  if (!manualMode) {
    if (gas > 2.0) {
      fanOn();
      digitalWrite(BUZZER, HIGH);
      digitalWrite(LED_PIN, HIGH);
    } else {
      fanOff();
      digitalWrite(BUZZER, LOW);
      digitalWrite(LED_PIN, LOW);
    }
  }

  delay(3000);
}