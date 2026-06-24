#include <Arduino.h>
#include "DHT.h"
#include <WiFi.h>
#include <esp_now.h>

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

uint8_t gatewayMAC[] ={ 0x1C,0xDB,0xD4,0xCB,0xCE,0xE8};

typedef struct {
  float temp;
  float humid;
  float gas;
} SensorData;

typedef struct {
  char command[10];
} ControlData;

SensorData sensorData;

// ================= STATE =================
bool manualMode = false;
bool fanState = false;
bool ledState = false;
bool buzzerState = false;

// ================= FAN =================
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

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len)
{
  ControlData cmd;
  memcpy(&cmd,
         incomingData,
         sizeof(cmd));

  Serial.print("Command: ");
  Serial.println(cmd.command);

  if(strcmp(cmd.command,"ON")==0)
  {
    manualMode = true;

    fanOn();

    digitalWrite(LED_PIN,HIGH);
    digitalWrite(BUZZER,HIGH);

    ledState = true;
    buzzerState = true;
  }

  else if(strcmp(cmd.command,"OFF")==0)
  {
    manualMode = true;

    fanOff();

    digitalWrite(LED_PIN,LOW);
    digitalWrite(BUZZER,LOW);

    ledState = false;
    buzzerState = false;
  }

  else if(strcmp(cmd.command,"AUTO")==0)
  {
    manualMode = false;
  }
}


void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.print("Send Status: ");
  if(status == ESP_NOW_SEND_SUCCESS)
    Serial.println("SUCCESS");
  else
    Serial.println("FAIL");
}

void readSensor(float &t, float &h, float &gasV)
{
  t = dht.getTemperature();
  h = dht.getHumidity();
  int raw = analogRead(MQ2_PIN);
  gasV = raw * (3.3f / 4095.0f);
}


void sendData(float t, float h, float gas)
{
  sensorData.temp = t;
  sensorData.humid = h;
  sensorData.gas = gas;
  esp_now_send(
      gatewayMAC,
      (uint8_t *)&sensorData,
      sizeof(sensorData));
}

// Khởi tạo ESP-NOW và thiết lập chế độ WiFi cho ESP32.
void initESPNow()
{
  WiFi.mode(WIFI_STA);
  Serial.print("Node MAC: ");
  Serial.println(WiFi.macAddress());
  if(esp_now_init() != ESP_OK)
  {
    Serial.println("ESP-NOW Init Failed");
    while(true);
  }

  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);
  esp_now_peer_info_t peerInfo = {};

  memcpy(peerInfo.peer_addr, gatewayMAC, 6);

  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if(esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Add Peer Failed");
    while(true);
  }

  Serial.println("ESP-NOW READY");
}



void setup()
{
  Serial.begin(115200);

  pinMode(LED_PIN,OUTPUT);
  pinMode(BUZZER,OUTPUT);

  ledcSetup(PWM_A, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_B, PWM_FREQ, PWM_RES);
  ledcAttachPin(INA, PWM_A);
  ledcAttachPin(INB, PWM_B);

  dht.setup(DHTPIN, DHT::DHT22);

  initESPNow();
  Serial.println("NODE READY");
}



void loop()
{
  float t,h,gas;
  readSensor(t, h, gas);
  Serial.printf("T=%.1f H=%.1f GAS=%.2fV\n",t,h,gas);
  sendData(t,h,gas);

  if(!manualMode)
  {
    if(gas > 2.0)
    {
      fanOn();
      digitalWrite(BUZZER, HIGH);
      digitalWrite(LED_PIN, HIGH);
    }
    else
    {
      fanOff();
      digitalWrite(BUZZER, LOW);
      digitalWrite(LED_PIN, LOW);
    }
  }

  delay(3000);
} 