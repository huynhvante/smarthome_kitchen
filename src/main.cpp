#include <Arduino.h>
#include "DHT.h"
#include <WiFi.h>
#include <esp_now.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

// ===================== PIN / PWM =====================
#define DHTPIN 4
#define MQ2_PIN 5

#define LED_RED_PIN    36   // LED do: chi dung de canh bao (nhap nhay)
#define LED_WHITE_PIN  10   // LED trang: chieu sang (MOI)

#define INA 18
#define INB 19

// ---- Nut nhan tich hop LED ----
// SIG2: ngo ra trang thai nut (mac dinh HIGH, nhan = LOW)
// SIG1: ngo vao dieu khien LED nut (mac dinh LOW, HIGH = sang)
#define BTN_LED_SIG1     6   // Nut vang (den)
#define BTN_LED_SIG2     7
#define BTN_FAN_SIG1     8   // Nut xanh (quat)
#define BTN_FAN_SIG2     9
#define BTN_DEBOUNCE_MS  30UL
#define BTN_LONG_PRESS_MS 2000UL       // giu nut bao lau thi ve tu dong
#define OVERRIDE_RELEASE_MS 1800000UL  // 30 phut sau lan nhan cuoi -> tu ve auto (dat 0 de tat)

#define PWM_A 0
#define PWM_B 1
#define PWM_FREQ 5000
#define PWM_RES 8
constexpr uint8_t FAN_HIGH = 3;
constexpr uint8_t FAN_PWM[] = {0, 100, 150, 200};  // Tắt, nhẹ, trung bình, mạnh.
constexpr unsigned long FAN_RAMP_MS = 300UL;
constexpr unsigned long FAN_START_HOLD_MS = 120UL;
constexpr unsigned long FAN_TAP_GAP_MS = 500UL;

#define SENSOR_INTERVAL_MS 2000UL
#define FAN_OFF_DELAY_MS 60000UL
#define RED_BLINK_MS 300UL             // chu ky nhap nhay LED do

// ---- Nguong gas (raw MQ2): dat tu app -> gateway -> bridge -> node, luu vao flash ----
#define GAS_TH_DEFAULT 1500            // dung khi chua tung nhan nguong tu app (chinh theo thuc te)
#define GAS_TH_MIN     50              // phai khop GAS_THRESHOLD_MIN trong gateway.py
#define GAS_TH_MAX     3000            // phai khop GAS_THRESHOLD_MAX trong gateway.py

// ===================== OTA CONFIG =====================
#define OTA_HOSTNAME        "kitchen-node"
#define OTA_PASSWORD        "Kitchen@k18"
#define OTA_WIFI_SSID       "hvt"
#define OTA_WIFI_PASS       "10012511"
#define OTA_WINDOW_MS       300000UL   // 5 phut cho phep upload
#define OTA_WIFI_TIMEOUT_MS 15000UL    // timeout ket noi WiFi

DHT dht;
Preferences prefs;

uint8_t gatewayMAC[] = {0x1C, 0xDB, 0xD4, 0xCB, 0xCE, 0xE8};

typedef struct __attribute__((packed)) {
  float temp;
  float humid;
  float gas_voltage;
  int gas_raw;
  bool fan;
  bool led;
  bool alarm;
  float gas_threshold;
} SensorData;

typedef struct __attribute__((packed)) {
  char command[20];
} ControlData;

SensorData sensorData;
constexpr unsigned int COMMAND_QUEUE_DEPTH = 16;
QueueHandle_t commandQueue = nullptr;

bool manualMode = false;   // manual tu gateway
bool alertMode = false;
bool gasDanger = false;    // cap nhat moi chu ky 2s
uint8_t fanLevel = 0;
uint8_t fanDuty = 0;
bool fanStarting = false;
unsigned long fanStartMs = 0;
bool ledState = false;     // trang thai LED TRANG (chieu sang)
unsigned long lastSensorMs = 0;

// ---- Nguong gas ----
int gasThreshold = GAS_TH_DEFAULT;
volatile int pendingGasTh = -1;   // dat trong callback ESP-NOW, xu ly trong loop() (khong ghi flash trong callback)

// ---- Ghi de tu nut nhan (manual) ----
// Room cho quạt theo chế độ chung của bếp khi dùng các lệnh gateway cũ.
enum class FanMode { Room, Auto, Button, Gateway };
FanMode fanMode = FanMode::Room;
bool ledOverride = false;
unsigned long lastButtonMs = 0;

// ---- Snapshot trang thai truoc bao dong de khoi phuc ----
struct PreAlarmSnapshot
{
  bool valid;
  bool manualMode;
  FanMode fanMode;
  bool ledOverride;
  uint8_t fanLevel;
  bool led;
};
PreAlarmSnapshot preAlarm = {};

// ---- Nhap nhay LED do ----
bool redBlinkState = false;
unsigned long lastRedBlinkMs = 0;

// OTA state
volatile bool otaRequested = false;  // dat trong callback ESP-NOW, xu ly trong loop()
bool otaMode = false;
unsigned long otaStartMs = 0;

// ===================== PROTOTYPES (can cho .cpp) =====================
void fanOn();
void fanOff();
void ledOn();
void ledOff();
void updateRedLed();
void updateButtonLamps();
void blinkLamp(uint8_t pin);
bool isAlarmActive();
void snapshotBeforeAlarm();
void restoreAfterAlarm();
void handleButtons();
void loadGasThreshold();
void applyPendingGasThreshold();
void applyCommand(const char *command);
void enterOtaMode();

// Dang bao dong = gateway ra lenh ALERT_ON HOAC node tu phat hien gas vuot nguong
bool isAlarmActive()
{
  return alertMode || gasDanger;
}

// ===================== BUTTON =====================
enum BtnEvent { BTN_NONE, BTN_SHORT, BTN_LONG };

struct Button {
  uint8_t pin;
  bool lastReading;
  bool stableState;
  unsigned long lastChangeMs;
  unsigned long pressStartMs;
  bool longFired;
};

Button btnLed = {BTN_LED_SIG2, HIGH, HIGH, 0, 0, false};
Button btnFan = {BTN_FAN_SIG2, HIGH, HIGH, 0, 0, false};
uint8_t fanTapCount = 0;
unsigned long lastFanReleaseMs = 0;
bool ignoreFanUntilRelease = false;

// Nhan ngan: tra ve BTN_SHORT khi tha nut.
// Nhan giu: tra ve BTN_LONG ngay khi du thoi gian (khi dang giu).
BtnEvent readButton(Button &b)
{
  bool reading = digitalRead(b.pin);
  unsigned long now = millis();

  if (reading != b.lastReading) {
    b.lastReading = reading;
    b.lastChangeMs = now;
  }

  if ((now - b.lastChangeMs) > BTN_DEBOUNCE_MS && reading != b.stableState) {
    b.stableState = reading;
    if (b.stableState == LOW) {          // vua nhan xuong
      b.pressStartMs = now;
      b.longFired = false;
    } else if (!b.longFired) {           // vua tha ra, chua phai nhan giu
      return BTN_SHORT;
    }
  }

  if (b.stableState == LOW && !b.longFired &&
      (now - b.pressStartMs) >= BTN_LONG_PRESS_MS) {
    b.longFired = true;
    return BTN_LONG;
  }
  return BTN_NONE;
}

// ===================== ACTUATORS =====================
void updateFanPwm()
{
  uint8_t duty = FAN_PWM[fanLevel];
  if (fanStarting) {
    const unsigned long elapsed = millis() - fanStartMs;
    if (elapsed < FAN_RAMP_MS) {
      duty = static_cast<uint8_t>(FAN_PWM[FAN_HIGH] * elapsed / FAN_RAMP_MS);
    } else if (elapsed < FAN_RAMP_MS + FAN_START_HOLD_MS) {
      duty = FAN_PWM[FAN_HIGH];
    } else {
      fanStarting = false;
    }
  }

  if (duty != fanDuty) {
    ledcWrite(PWM_B, duty);
    fanDuty = duty;
  }
}

void setFanLevel(uint8_t level)
{
  if (level > FAN_HIGH || level == fanLevel) return;
  if (level == 0) {
    fanStarting = false;
  } else if (fanLevel == 0) {
    // Khởi động êm đến mức quạt đã chạy ổn, rồi hạ xuống mức đã chọn.
    fanStartMs = millis();
    fanStarting = true;
  }
  fanLevel = level;
  updateFanPwm();
}

void fanOn()  { setFanLevel(FAN_HIGH); }
void fanOff() { setFanLevel(0); }

bool fanIsAuto()
{
  return fanMode == FanMode::Auto || (fanMode == FanMode::Room && !manualMode);
}

void cancelFanGesture()
{
  fanTapCount = 0;
  ignoreFanUntilRelease = btnFan.stableState == LOW || digitalRead(btnFan.pin) == LOW;
}

// LED trang (chieu sang)
void ledOn()
{
  digitalWrite(LED_WHITE_PIN, HIGH);
  ledState = true;
}

void ledOff()
{
  digitalWrite(LED_WHITE_PIN, LOW);
  ledState = false;
}

// LED do: nhap nhay khi co canh bao, con lai luon tat. Khong chan (dung millis).
void updateRedLed()
{
  bool alarmActive = alertMode || gasDanger;
  if (!alarmActive) {
    if (redBlinkState) {
      redBlinkState = false;
      digitalWrite(LED_RED_PIN, LOW);
    }
    return;
  }

  unsigned long now = millis();
  if (now - lastRedBlinkMs >= RED_BLINK_MS) {
    lastRedBlinkMs = now;
    redBlinkState = !redBlinkState;
    digitalWrite(LED_RED_PIN, redBlinkState ? HIGH : LOW);
  }
}

// Chớp ba nhịp khi về AUTO. Vòng loop vẫn cập nhật PWM trong lúc chớp.
struct LampBlink {
  bool active;
  unsigned long startMs;
};
LampBlink ledBlink = {}, fanBlink = {};

void blinkLamp(uint8_t pin)
{
  LampBlink &blink = (pin == BTN_LED_SIG1) ? ledBlink : fanBlink;
  blink = {true, millis()};
}

void updateButtonLamp(uint8_t pin, LampBlink &blink, unsigned long now)
{
  const unsigned long elapsed = now - blink.startMs;
  if (elapsed >= 600UL) blink.active = false;
  const bool on = blink.active ? (elapsed / 100UL) % 2 == 0 : !ledState;
  digitalWrite(pin, on ? HIGH : LOW);
}

void updateButtonLamps()
{
  const unsigned long now = millis();
  updateButtonLamp(BTN_LED_SIG1, ledBlink, now);
  updateButtonLamp(BTN_FAN_SIG1, fanBlink, now);
}

// ===================== SNAPSHOT TRUOC BAO DONG =====================
// Chup trang thai truoc khi vao bao dong (chi chup o lan dau tien).
// Phai goi TRUOC khi fanOn()/ledOn() cua bao dong chay.
void snapshotBeforeAlarm()
{
  if (preAlarm.valid) return;
  preAlarm = {true, manualMode, fanMode, ledOverride, fanLevel, ledState};
  cancelFanGesture();
}

// Het bao dong (ca ALERT_OFF lan gas ve binh thuong): khoi phuc trang thai truoc do.
// Goi tu loop(), nen goi nhieu lan ALERT_OFF cung khong pha duoc ket qua.
void restoreAfterAlarm()
{
  PreAlarmSnapshot snap = preAlarm;
  preAlarm.valid = false;

  manualMode  = snap.manualMode;
  fanMode     = snap.fanMode;
  ledOverride = snap.ledOverride;

  // Khôi phục đúng mức quạt thủ công. Quạt AUTO được xét lại theo gas trong loop.
  if (!fanIsAuto()) setFanLevel(snap.fanLevel);
  if (manualMode || ledOverride) {
    if (snap.led) ledOn(); else ledOff();
  }

  lastButtonMs = millis();   // bo dem 30 phut bat dau lai
  updateButtonLamps();
  Serial.println("Alarm cleared: restored previous state");
}

void handleButtons()
{
  // Doc ca hai nut MOT lan duy nhat moi vong lap
  BtnEvent eled = readButton(btnLed);
  BtnEvent efan = readButton(btnFan);

  // An toan: dang bao dong thi bo qua nut
  if (isAlarmActive()) {
    cancelFanGesture();
    if (eled != BTN_NONE || efan != BTN_NONE) {
      Serial.println("Alarm active: ignore button press");
    }
    return;
  }

  // ---- Nut vang: den trang ----
  if (eled == BTN_SHORT) {
    ledOverride = true;
    lastButtonMs = millis();
    if (ledState) ledOff(); else ledOn();
    Serial.printf("Button LED -> %s (MANUAL)\n", ledState ? "ON" : "OFF");
  } else if (eled == BTN_LONG) {
    ledOverride = false;
    manualMode = false;
    blinkLamp(BTN_LED_SIG1);
    lastSensorMs = millis() - SENSOR_INTERVAL_MS;  // danh gia lai ngay
    Serial.println("Button LED -> AUTO");
  }

  // ---- Nút xanh: đếm cụm nhấn để chọn mức quạt ----
  if (ignoreFanUntilRelease) {
    if (btnFan.stableState == HIGH && btnFan.lastReading == HIGH) ignoreFanUntilRelease = false;
    return;
  }
  if (efan == BTN_LONG) {
    fanTapCount = 0;
    fanMode = FanMode::Room;
    manualMode = false;
    blinkLamp(BTN_FAN_SIG1);
    lastSensorMs = millis() - SENSOR_INTERVAL_MS;
    Serial.println("Button FAN -> AUTO");
  } else if (efan == BTN_SHORT) {
    if (fanTapCount <= FAN_HIGH) ++fanTapCount;
    lastFanReleaseMs = millis();
  }
  if (fanTapCount && btnFan.stableState == HIGH && btnFan.lastReading == HIGH &&
      millis() - lastFanReleaseMs >= FAN_TAP_GAP_MS) {
    const uint8_t taps = fanTapCount;
    fanTapCount = 0;
    if (taps <= FAN_HIGH) {
      const bool turnOff = taps == 1 && !fanIsAuto() && fanLevel != 0;
      fanMode = FanMode::Button;
      lastButtonMs = millis();
      setFanLevel(turnOff ? 0 : taps);
      Serial.printf("Button FAN -> level %u\n", static_cast<unsigned>(fanLevel));
    }
  }
}

// ===================== GAS THRESHOLD =====================
// Doc nguong da luu trong flash (neu chua co thi dung mac dinh)
void loadGasThreshold()
{
  prefs.begin("kitchen", false);
  int saved = prefs.getInt("gas_th", GAS_TH_DEFAULT);
  if (saved < GAS_TH_MIN || saved > GAS_TH_MAX) saved = GAS_TH_DEFAULT;
  gasThreshold = saved;
  Serial.printf("Gas threshold loaded: %d\n", gasThreshold);
}

// Goi trong loop(): ap dung nguong moi nhan duoc tu callback
void applyPendingGasThreshold()
{
  int th = pendingGasTh;
  if (th == -1) return;
  pendingGasTh = -1;

  if (th < GAS_TH_MIN || th > GAS_TH_MAX) {
    Serial.printf("Gas threshold %d ngoai khoang %d..%d, bo qua\n", th, GAS_TH_MIN, GAS_TH_MAX);
    return;
  }

  if (th != gasThreshold) {
    gasThreshold = th;
    prefs.putInt("gas_th", gasThreshold);   // chi ghi flash khi gia tri that su doi
  }
  Serial.printf("Gas threshold set: %d\n", gasThreshold);

  // Danh gia lai va gui telemetry ngay de gateway/app thay gia tri moi
  lastSensorMs = millis() - SENSOR_INTERVAL_MS;
}

// ===================== COMMAND HANDLING =====================
void applyCommand(const char *command)
{
  const bool levelCommand = strncmp(command, "FAN_", 4) == 0 &&
      command[4] >= '0' && command[4] <= '3' && command[5] == '\0';
  const bool fanAuto = strcmp(command, "FAN_AUTO") == 0;
  const bool on = strcmp(command, "FAN_ON") == 0 || strcmp(command, "ON") == 0;
  const bool off = strcmp(command, "FAN_OFF") == 0 || strcmp(command, "OFF") == 0;

  // Dang bao dong: chan lenh den/quat thu cong
  if (isAlarmActive() && (levelCommand || fanAuto || on || off ||
       strcmp(command, "LED_ON") == 0  || strcmp(command, "LED_OFF") == 0)) {
    Serial.println("Alarm active: ignore device command");
    return;
  }

  if (levelCommand || on || off) {
    cancelFanGesture();
    uint8_t level = on ? FAN_HIGH : 0;
    if (levelCommand) {
      fanMode = FanMode::Gateway;
      level = static_cast<uint8_t>(command[4] - '0');
    } else {
      manualMode = true;
      if (fanMode != FanMode::Button) fanMode = FanMode::Room;
    }
    setFanLevel(level);
  } else if (fanAuto) {
    cancelFanGesture();
    fanMode = FanMode::Auto;
    lastSensorMs = millis() - SENSOR_INTERVAL_MS;
  } else if (strcmp(command, "LED_ON") == 0) {
    manualMode = true;
    ledOn();
  } else if (strcmp(command, "LED_OFF") == 0) {
    manualMode = true;
    ledOff();
  } else if (strcmp(command, "ALERT_ON") == 0) {
    snapshotBeforeAlarm();            // phai dung TRUOC fanOn()/ledOn()
    alertMode = true;
    fanOn();
    ledOn();
  } else if (strcmp(command, "ALERT_OFF") == 0) {
    // Chi tat co: loop() se khoi phuc trang thai truoc bao dong (hoac ve auto)
    // va van giu den/quat neu node van tu phat hien gas vuot nguong.
    // Gateway gui lenh nay nhieu lan nen khong lam gi them o day.
    alertMode = false;
    lastSensorMs = millis() - SENSOR_INTERVAL_MS;   // danh gia lai ngay
  } else if (strcmp(command, "AUTO") == 0) {
    cancelFanGesture();
    preAlarm.valid = false;           // lenh AUTO tuong minh thang snapshot
    manualMode = false;
    alertMode = false;
    fanMode = FanMode::Room;
    ledOverride = false;
    lastSensorMs = millis() - SENSOR_INTERVAL_MS;
  } else if (strcmp(command, "OTA") == 0) {
    otaRequested = true;  // chi dat co, loop() se xu ly
  } else if (strncmp(command, "GAS_TH=", 7) == 0) {
    pendingGasTh = atoi(command + 7);   // chi dat co, loop() kiem tra khoang va ghi flash
  }
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len)
{
  if (len != sizeof(ControlData) || commandQueue == nullptr) {
    return;
  }

  ControlData cmd;
  memcpy(&cmd, incomingData, sizeof(cmd));
  cmd.command[sizeof(cmd.command) - 1] = '\0';

  // Chỉ vòng loop thay đổi PWM, mức quạt và trạng thái báo động.
  if (xQueueSend(commandQueue, &cmd, 0) != pdTRUE) {
    Serial.println("ESP-NOW command queue full");
  }
}

void handleNextCommand()
{
  ControlData cmd;
  if (commandQueue && xQueueReceive(commandQueue, &cmd, 0) == pdTRUE) {
    Serial.print("Command: ");
    Serial.println(cmd.command);
    applyCommand(cmd.command);
  }
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.print("Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAIL");
}

// ===================== SENSORS =====================
void readSensor(float &t, float &h, int &gasRaw, float &gasVoltage)
{
  t = dht.getTemperature();
  h = dht.getHumidity();
  float mq2_offset = 27; // Offset value for MQ2 sensor calibration
  gasRaw = analogRead(MQ2_PIN) - mq2_offset;
  if (gasRaw < 0) gasRaw = 0;
  gasVoltage = gasRaw * (3.3f / 4095.0f);
}

void sendData(float t, float h, int gasRaw, float gasVoltage)
{
  sensorData.temp = t;
  sensorData.humid = h;
  sensorData.gas_voltage = gasVoltage;
  sensorData.gas_raw = gasRaw;
  sensorData.fan = fanLevel != 0;
  sensorData.led = ledState;
  sensorData.alarm = alertMode;
  sensorData.gas_threshold = (float)gasThreshold;   // bao lai nguong that su dang dung

  esp_now_send(gatewayMAC, (uint8_t *)&sensorData, sizeof(sensorData));
}

// ===================== ESP-NOW =====================
void initESPNow()
{
  WiFi.mode(WIFI_STA);
  Serial.print("Node MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed");
    while (true);
  }

  commandQueue = xQueueCreate(COMMAND_QUEUE_DEPTH, sizeof(ControlData));
  if (commandQueue) esp_now_register_recv_cb(OnDataRecv);
  else Serial.println("ESP-NOW command queue unavailable");
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

// ===================== OTA =====================
void enterOtaMode()
{
  Serial.println("Entering OTA mode...");

  // An toan: tat co cau chap hanh truoc khi nap
  alertMode = false;
  gasDanger = false;
  preAlarm.valid = false;
  ledBlink.active = false;
  fanBlink.active = false;
  fanOff();
  ledOff();
  digitalWrite(LED_RED_PIN, LOW);
  redBlinkState = false;
  updateButtonLamps();

  // Ngat ESP-NOW de radio duoc tu do ket noi router
  esp_now_deinit();
  WiFi.mode(WIFI_OFF);
  delay(100);
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(OTA_HOSTNAME);
  WiFi.begin(OTA_WIFI_SSID, OTA_WIFI_PASS);

  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - t0 > OTA_WIFI_TIMEOUT_MS) {
      Serial.printf("OTA WiFi failed, status=%d\n", WiFi.status());
      ESP.restart();
    }
    delay(500);
  }

  Serial.print("OTA IP: ");
  Serial.println(WiFi.localIP());

  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.onStart([]() { Serial.println("OTA start"); });
  ArduinoOTA.onEnd([]() { Serial.println("\nOTA end"); });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("OTA %u%%\r", progress * 100 / total);
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA error %u, restarting\n", error);
    ESP.restart();
  });
  ArduinoOTA.begin();

  otaMode = true;
  otaStartMs = millis();
  Serial.println("OTA READY (waiting for upload)");
}

// ===================== SETUP / LOOP =====================
void setup()
{
  Serial.begin(115200);

  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_WHITE_PIN, OUTPUT);
  digitalWrite(LED_RED_PIN, LOW);
  ledOff();

  // Nut nhan: SIG2 la ngo ra cua module (mac dinh HIGH) -> doc vao
  //           SIG1 la ngo vao cua module (mac dinh LOW)  -> xuat ra
  pinMode(BTN_LED_SIG2, INPUT_PULLUP);
  pinMode(BTN_FAN_SIG2, INPUT_PULLUP);
  pinMode(BTN_LED_SIG1, OUTPUT);
  pinMode(BTN_FAN_SIG1, OUTPUT);

  ledcSetup(PWM_A, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_B, PWM_FREQ, PWM_RES);
  ledcAttachPin(INA, PWM_A);
  ledcAttachPin(INB, PWM_B);
  ledcWrite(PWM_A, 0);
  ledcWrite(PWM_B, 0);

  updateButtonLamps();

  dht.setup(DHTPIN, DHT::DHT22);

  loadGasThreshold();   // doc nguong da luu truoc khi bat dau danh gia gas

  initESPNow();
  Serial.println("KITCHEN NODE READY");
}

void loop()
{
  if (!otaMode) handleNextCommand();
  // --- Che do OTA: chi phuc vu upload, het han thi khoi dong lai ---
  if (otaRequested && !otaMode) {
    enterOtaMode();
  }
  if (otaMode) {
    ArduinoOTA.handle();
    if (millis() - otaStartMs > OTA_WINDOW_MS) {
      Serial.println("OTA timeout, restarting");
      ESP.restart();
    }
    return;
  }

  // --- Ap dung nguong gas moi (neu co) truoc khi danh gia ---
  applyPendingGasThreshold();

  // --- Nut nhan + LED do nhap nhay: quet lien tuc ---
  handleButtons();
  updateFanPwm();
  updateButtonLamps();
  updateRedLed();

  // --- Che do binh thuong (chu ky 2s) ---
  unsigned long now = millis();
  if (now - lastSensorMs < SENSOR_INTERVAL_MS) {
    return;
  }
  lastSensorMs = now;

  float t, h, gasVoltage;
  int gasRaw;
  readSensor(t, h, gasRaw, gasVoltage);

  gasDanger = gasRaw >= gasThreshold;
  bool alarmNow = isAlarmActive();

  // Vao bao dong: chup snapshot (lan dau). Het bao dong: khoi phuc.
  if (alarmNow) {
    snapshotBeforeAlarm();
  } else if (preAlarm.valid) {
    restoreAfterAlarm();
  }

  // Tu tra ve auto neu qua lau khong nhan nut (OVERRIDE_RELEASE_MS = 0: tat tinh nang nay)
  // Khong ap dung khi dang bao dong. So sanh co dau de khong bi tran khi lastButtonMs
  // vua duoc dat lai (lon hon 'now' mot chut).
  if (OVERRIDE_RELEASE_MS > 0 && !alarmNow && (fanMode == FanMode::Button || ledOverride) &&
      (long)(now - lastButtonMs) >= (long)OVERRIDE_RELEASE_MS) {
    Serial.println("Override timeout: back to AUTO");
    if (fanMode == FanMode::Button) fanMode = FanMode::Room;
    ledOverride = false;
  }

  Serial.printf("T=%.1f H=%.1f GAS_RAW=%d GAS=%.2fV TH=%d DANGER=%d FAN=%d LED=%d | fanMan=%d ledMan=%d gwMan=%d\n",
                t, h, gasRaw, gasVoltage, gasThreshold, gasDanger ? 1 : 0,
                fanLevel != 0 ? 1 : 0, ledState ? 1 : 0,
                fanMode == FanMode::Button ? 1 : 0, ledOverride ? 1 : 0,
                (manualMode || fanMode == FanMode::Gateway) ? 1 : 0);

  if (alarmNow) {
    // AN TOAN: canh bao luon bat quat + den trang, bat ke manual hay nut nhan
    // (LED do nhap nhay do updateRedLed() xu ly)
    fanOn();
    ledOn();
  } else {
    if (fanIsAuto()) fanOff();
    if (!manualMode && !ledOverride) ledOff();
  }

  updateButtonLamps();
  sendData(t, h, gasRaw, gasVoltage);
}
