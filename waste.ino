#include <DHT.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>

#define SERVO1_PIN 13
#define SERVO2_PIN 14
#define ULTRASONIC1_TRIG_PIN 18
#define ULTRASONIC1_ECHO_PIN 5
#define ULTRASONIC2_TRIG_PIN 23
#define ULTRASONIC2_ECHO_PIN 25
#define DHT_PIN 16
#define DHTTYPE DHT11
#define MQ135_PIN 34
#define IR_PROXIMITY_PIN 25
#define INDUCTIVE_SENSOR_PIN 26
#define CAPACITIVE_SENSOR_PIN 27
#define LCD_SDA_PIN 21
#define LCD_SCL_PIN 22
#define DETECTION_DISTANCE1 10
#define DETECTION_DISTANCE2 20

const char* ssid = "PGAUniversity";
const char* password = "Minyakkita";

Servo servo1;
Servo servo2;
DHT dht(DHT_PIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x27, 16, 2);

bool prevDetection1 = false;
bool prevDetection2 = false;
bool prevIRDetection = false;
bool prevCapacitiveDetection = false;
bool wifiConnected = false;

unsigned long lastPrintTime = 0;
const unsigned long printInterval = 10000;
unsigned long lastLCDUpdateTime = 0;
const unsigned long lcdUpdateInterval = 2000;

float humidity = 0;
float temperature = 0;
int mq135Value = 0;
long distance1 = 0;
long distance2 = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("=== Sistem Pemilah Sampah Otomatis ===");

  Wire.begin(LCD_SDA_PIN, LCD_SCL_PIN);
  lcd.init();
  lcd.backlight();
  lcd.print("Pemilah Sampah");
  lcd.setCursor(0, 1);
  lcd.print("Inisialisasi...");

  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo1.write(0);
  servo2.write(0);

  dht.begin();

  pinMode(ULTRASONIC1_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC1_ECHO_PIN, INPUT);
  pinMode(ULTRASONIC2_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC2_ECHO_PIN, INPUT);
  pinMode(MQ135_PIN, INPUT);
  pinMode(IR_PROXIMITY_PIN, INPUT);
  pinMode(INDUCTIVE_SENSOR_PIN, INPUT);
  pinMode(CAPACITIVE_SENSOR_PIN, INPUT);

  connectToWiFi();

  lcd.clear();
  lcd.print("Sistem Siap");
  lcd.setCursor(0, 1);
  lcd.print(wifiConnected ? "WiFi: Connected" : "WiFi: Failed");
  delay(2000);
  lcd.clear();
  lcd.print("Menunggu Sampah");
}

void connectToWiFi() {
  lcd.clear();
  lcd.print("Connecting WiFi");
  WiFi.begin(ssid, password);

  int timeout = 0;
  while (WiFi.status() != WL_CONNECTED && timeout < 20) {
    delay(500);
    lcd.setCursor(timeout % 16, 1);
    lcd.print(".");
    timeout++;
  }

  wifiConnected = WiFi.status() == WL_CONNECTED;
  lcd.clear();
  lcd.print(wifiConnected ? "WiFi Connected" : "WiFi Failed");
  delay(2000);
}

long readUltrasonicDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  return pulseIn(echoPin, HIGH) / 58.2;
}

void updateLCDStatus(String line1, String line2) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(line2);
}

void readAllSensors() {
  distance1 = readUltrasonicDistance(ULTRASONIC1_TRIG_PIN, ULTRASONIC1_ECHO_PIN);
  distance2 = readUltrasonicDistance(ULTRASONIC2_TRIG_PIN, ULTRASONIC2_ECHO_PIN);
  humidity = dht.readHumidity();
  temperature = dht.readTemperatureC();
  mq135Value = analogRead(MQ135_PIN);
}

void loop() {
  readAllSensors();

  bool irDetection = digitalRead(IR_PROXIMITY_PIN) == LOW;
  bool metalDetection = irDetection && digitalRead(INDUCTIVE_SENSOR_PIN) == LOW;
  bool capacitiveDetection = irDetection && digitalRead(CAPACITIVE_SENSOR_PIN) == LOW;
  bool detection1 = distance1 < DETECTION_DISTANCE1;
  bool detection2 = distance2 < DETECTION_DISTANCE2;

  if (detection2 && !prevDetection2)
    updateLCDStatus("SAMPAH ANORGANIK", "Jarak: " + String(distance2) + " cm");

  if (irDetection && !prevIRDetection)
    updateLCDStatus("DETEKSI SAMPAH", "Sensor Infrared");

  if (capacitiveDetection && !prevCapacitiveDetection)
    updateLCDStatus("DETEKSI SAMPAH", "Sensor Kapasitif");

  if (detection1 && !prevDetection1)
    updateLCDStatus("SAMPAH ORGANIK", "Jarak: " + String(distance1) + " cm");

  if (irDetection && capacitiveDetection && metalDetection) {
  updateLCDStatus("LOGAM TERDETEKSI", "Servo Selesai");
  servo2.write(180); delay(3000); servo2.write(0);
} else if (irDetection && capacitiveDetection && !metalDetection) {
  updateLCDStatus("NON LOGAM", "Servo Selesai");
  servo2.write(180); delay(3000); servo2.write(0);
} else if (irDetection) {
  updateLCDStatus("ORGANIK DETECTED", "Servo Selesai");
  servo1.write(180); delay(3000); servo1.write(0);
}


  unsigned long currentTime = millis();
  if (currentTime - lastPrintTime >= printInterval) {
    Serial.println("\n=== UPDATE ===");
    Serial.printf("Suhu: %.1f C, Kelembaban: %.1f%%\n", temperature, humidity);
    Serial.printf("MQ135: %d, Jarak1: %ld cm, Jarak2: %ld cm\n", mq135Value, distance1, distance2);
    Serial.println("Infrared: " + String(irDetection));
    Serial.println("Kapasitif: " + String(capacitiveDetection));
    Serial.println("Induktif: " + String(metalDetection));

    if (WiFi.status() != WL_CONNECTED && wifiConnected) {
      wifiConnected = false;
      Serial.println("WiFi terputus. Mencoba reconnect...");
      WiFi.reconnect();
    } else if (WiFi.status() == WL_CONNECTED && !wifiConnected) {
      wifiConnected = true;
      Serial.println("WiFi terhubung kembali");
    }
    lastPrintTime = currentTime;
  }

  if (currentTime - lastLCDUpdateTime >= lcdUpdateInterval && 
      !detection1 && !detection2 && !irDetection && !capacitiveDetection) {
    static int mode = 0;
    mode = (mode + 1) % 3;
    switch (mode) {
      case 0:
        updateLCDStatus("Pemilah Sampah", wifiConnected ? "WiFi: Connected" : "WiFi: Disconnected");
        break;
      case 1:
        updateLCDStatus("Suhu: " + String(temperature) + " C", "Kelemb: " + String(humidity) + " %");
        break;
      case 2:
        updateLCDStatus("Org:" + String(distance1) + " Anor:" + String(distance2), "MQ135: " + String(mq135Value));
        break;
    }
    lastLCDUpdateTime = currentTime;
  }

  prevDetection1 = detection1;
  prevDetection2 = detection2;
  prevIRDetection = irDetection;
  prevCapacitiveDetection = capacitiveDetection;
  delay(100);
}