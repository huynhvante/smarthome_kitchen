#include <Arduino.h>
#include "DHT.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


#define INA 18
#define INB 19
#define PWM_CHANNEL_A 0
#define PWM_CHANNEL_B 1
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8   // 0-255
#define speed_fan 200

DHT dht;
#define pinDHT 4
#define timedelay 500


// OLED config
#define I2C_SDA   8
#define I2C_SCL   9
#define OLED_ADDR 0x3C
#define OLED_W    128
#define OLED_H    64
#define OLED_RST  -1
Adafruit_SSD1306 display(OLED_W, OLED_H, &Wire, OLED_RST);

void setup() {
  Serial.begin(115200);
  dht.setup(pinDHT, DHT::DHT22);
  ledcSetup(PWM_CHANNEL_A, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_B, PWM_FREQ, PWM_RESOLUTION);

  ledcAttachPin(INA, PWM_CHANNEL_A);
  ledcAttachPin(INB, PWM_CHANNEL_B);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED not found"); 
    while (1);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.println("System Starting...");
  display.display();

  delay(2000);

}

// Điều khiển quạt quay với tốc độ speed
void forward(int speed) {
  ledcWrite(PWM_CHANNEL_A, 0);
  ledcWrite(PWM_CHANNEL_B, speed);
}

void stopMotor() {
  ledcWrite(PWM_CHANNEL_A, 0);
  ledcWrite(PWM_CHANNEL_B, 0);
}

// hàm đọc dht
void readDHT22(float &temperature, float &humidity) {
  temperature = dht.getTemperature();
  humidity    = dht.getHumidity();

  if (dht.getStatus() != DHT::ERROR_NONE) {
    Serial.println("Failed to read from DHT sensor!");
    temperature = NAN;
    humidity    = NAN;
    return;
  }

  Serial.printf("Temperature: %.1f C, Humidity: %.1f %%\n", temperature, humidity);
}

void oledDisplay(float temperature, float humidity) {

  display.clearDisplay();

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("DHT22 Monitor");

  display.setTextSize(2);

  display.setCursor(0, 20);
  display.print(temperature, 1);
  display.println(" C");

  display.setCursor(0, 45);
  display.print(humidity, 1);
  display.println(" %");

  display.display();
}


void loop() {
  float temperature, humidity;
  readDHT22(temperature, humidity);
  oledDisplay(temperature, humidity);

  // if temperature or humidity is NAN, skip motor control
  if (isnan(temperature) || isnan(humidity)) {
    delay(2000); // wait before trying again
    return;
  }

  // nếu nhiệt độ lớn hơn 30 độ C hoặc độ ẩm lớn hơn 70%, quạt quay
  if (temperature > 30.0 && humidity > 75.0) {
    forward(speed_fan); 
    
  }

  else {
    stopMotor();
  }
}