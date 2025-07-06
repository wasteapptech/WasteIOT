#include <DHT.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>

#define SERVO1_PIN 13
#define SERVO2_PIN 14
#define ULTRASONIC1_TRIG_PIN 18
#define ULTRASONIC1_ECHO_PIN 5
#define ULTRASONIC2_TRIG_PIN 19
#define ULTRASONIC2_ECHO_PIN 23
#define DHT_PIN 16
#define DHTTYPE DHT11
#define mq4_PIN 34
#define IR_PROXIMITY_PIN 25
#define INDUCTIVE_SENSOR_PIN 26
#define CAPACITIVE_SENSOR_PIN 27
#define LCD_SDA_PIN 21
#define LCD_SCL_PIN 22
#define DETECTION_DISTANCE 5
#define BIN_HEIGHT 40
#define SENSOR_HEIGHT 5
#define MAX_DISTANCE (BIN_HEIGHT + SENSOR_HEIGHT)
#define MIN_DISTANCE SENSOR_HEIGHT
#define MQ4_SAMPLES 10
#define MQ4_SAMPLE_INTERVAL 50
#define MQ4_CLEAN_AIR_RATIO 4.4
#define MQ4_CALIBRATION_TIME 5000

const char* ssid = "PGAUniversity";
const char* password = "Minyakkita";
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
const char* api_url = "https://api-wasteapp.vercel.app/api/sensor/data";

String device_id = "waste_sorter_" + String(random(1000, 9999));
String topic_sensor_data = "waste/" + device_id + "/sensor_data";
String topic_waste_detected = "waste/" + device_id + "/waste_detected";
String topic_status = "waste/" + device_id + "/status";
String topic_command = "waste/" + device_id + "/command";

Servo servo1;
Servo servo2;
DHT dht(DHT_PIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x27, 16, 2);
WiFiClient espClient;
PubSubClient client(espClient);
HTTPClient http;

bool mqttConnected = false;
bool wifiConnected = false;
float humidity = 0;
float temperature = 0;
float mq4Value = 0;
float mq4R0 = 10;
int organicWasteCount = 0;
int inorganicWasteCount = 0;
int metalWasteCount = 0;

unsigned long lastPrintTime = 0;
const unsigned long printInterval = 10000;
unsigned long lastLCDUpdateTime = 0;
const unsigned long lcdUpdateInterval = 2000;
unsigned long lastMQTTSend = 0;
const unsigned long mqttInterval = 4000;
unsigned long lastApiPostTime = 0;
const unsigned long apiPostInterval = 30000;

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
  pinMode(mq4_PIN, INPUT);
  pinMode(IR_PROXIMITY_PIN, INPUT);
  pinMode(INDUCTIVE_SENSOR_PIN, INPUT);
  pinMode(CAPACITIVE_SENSOR_PIN, INPUT);

  calibrateMQ4();
  connectToWiFi();
  if (wifiConnected) {
    connectToMQTT();
  }

  lcd.clear();
  lcd.print("Sistem Siap");
  delay(2000);
  lcd.clear();
  lcd.print("Menunggu Sampah");
}

void calibrateMQ4() {
  lcd.clear();
  lcd.print("Calibrating MQ4");
  
  unsigned long startTime = millis();
  float rs_gas = 0;
  int samples = 0;
  
  while (millis() - startTime < MQ4_CALIBRATION_TIME) {
    if (samples < 100) {
      rs_gas += getMQ4Resistance();
      samples++;
      delay(200);
    }
  }
  
  rs_gas = rs_gas / samples;
  mq4R0 = rs_gas / MQ4_CLEAN_AIR_RATIO;
  
  lcd.clear();
  lcd.print("MQ4 Calibrated");
  delay(2000);
}

float getMQ4Resistance() {
  int val = 0;
  for (int i = 0; i < MQ4_SAMPLES; i++) {
    val += analogRead(mq4_PIN);
    delay(MQ4_SAMPLE_INTERVAL);
  }
  val = val / MQ4_SAMPLES;
  float voltage = val * (3.3 / 4095.0);
  float rs = (3.3 - voltage) / voltage * 10.0;
  return rs;
}

float getMQ4PPM() {
  float rs = getMQ4Resistance();
  float ratio = rs / mq4R0;
  float ppm = 1000 * pow(ratio, -2.3);
  if (ppm < 0) ppm = 0;
  if (ppm > 10000) ppm = 10000;
  return ppm;
}

int calculateBinCapacity(long distance) {
  if (distance > MAX_DISTANCE || distance == 0) return 0;
  if (distance <= MIN_DISTANCE) return 100;
  int percentage = ((MAX_DISTANCE - distance) * 100) / (MAX_DISTANCE - MIN_DISTANCE);
  if (percentage < 0) percentage = 0;
  if (percentage > 100) percentage = 100;
  return percentage;
}

void connectToWiFi() {
  lcd.clear();
  lcd.print("Connecting WiFi");
  WiFi.begin(ssid, password);

  int timeout = 0;
  while (WiFi.status() != WL_CONNECTED && timeout < 20) {
    delay(500);
    timeout++;
  }

  wifiConnected = WiFi.status() == WL_CONNECTED;
  lcd.clear();
  lcd.print(wifiConnected ? "WiFi Connected" : "WiFi Failed");
  delay(2000);
}

void connectToMQTT() {
  if (!wifiConnected) return;
  
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);
  
  String clientId = "WasteSorter_" + String(random(0xffff), HEX);
  if (client.connect(clientId.c_str())) {
    mqttConnected = true;
    client.subscribe(topic_command.c_str());
    sendStatusData();
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  if (String(topic) == topic_command) {
    handleCommand(message);
  }
}

void handleCommand(String command) {
  if (command == "get_status") {
    sendStatusData();
  }
  else if (command == "get_sensors") {
    sendSensorData();
  }
}

void sendStatusData() {
  if (!mqttConnected) return;
  
  DynamicJsonDocument doc(512);
  
  doc["device_id"] = device_id;
  doc["timestamp"] = millis();
  doc["status"] = "online";
  doc["wifiConnected"] = wifiConnected;
  doc["mqttConnected"] = mqttConnected;
  doc["uptime"] = millis() / 1000;
  doc["freeHeap"] = ESP.getFreeHeap();
  
  if (wifiConnected) {
    doc["ipAddress"] = WiFi.localIP().toString();
    doc["rssi"] = WiFi.RSSI();
  }
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  client.publish(topic_status.c_str(), jsonString.c_str(), true);
  Serial.println("Status published to MQTT");
}

void sendSensorData() {
  if (!mqttConnected) return;
  
  DynamicJsonDocument doc(512);
  doc["device_id"] = device_id;
  doc["timestamp"] = millis();
  doc["temperature"] = temperature;
  doc["humidity"] = humidity;
  doc["mq4_value"] = mq4Value;
  doc["bin1_capacity"] = calculateBinCapacity(readUltrasonicDistance(ULTRASONIC1_TRIG_PIN, ULTRASONIC1_ECHO_PIN));
  doc["bin2_capacity"] = calculateBinCapacity(readUltrasonicDistance(ULTRASONIC2_TRIG_PIN, ULTRASONIC2_ECHO_PIN));
  doc["uptime"] = millis() / 1000;

  String jsonString;
  serializeJson(doc, jsonString);
  
  Serial.println("\n=== Publishing MQTT Message ===");
  Serial.println(jsonString);
  Serial.println("=============================");
  
  if (client.publish(topic_sensor_data.c_str(), jsonString.c_str())) {
    Serial.println("✓ Published successfully");
  } else {
    Serial.println("✗ Publish failed");
  }
}

void sendWasteDetectionEvent(String wasteType) {
  if (!mqttConnected) return;
  
  DynamicJsonDocument doc(512);
  doc["device_id"] = device_id;
  doc["wasteType"] = wasteType;
  doc["timestamp"] = millis();
  doc["organicCount"] = organicWasteCount;
  doc["inorganicCount"] = inorganicWasteCount;
  
  String jsonString;
  serializeJson(doc, jsonString);
  client.publish(topic_waste_detected.c_str(), jsonString.c_str());
}

void postToAPI(String nameSensor, float value) {
  if (!wifiConnected || millis() - lastApiPostTime < apiPostInterval) return;

  http.begin(api_url);
  http.addHeader("Content-Type", "application/json");

  DynamicJsonDocument doc(128);
  doc["nameSensor"] = nameSensor;
  doc["value"] = value;

  String jsonString;
  serializeJson(doc, jsonString);

  int httpResponseCode = http.POST(jsonString);
  if (httpResponseCode > 0) {
    lastApiPostTime = millis();
  }
  http.end();
}

long readUltrasonicDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);
  return duration * 0.034 / 2;
}

void updateLCDStatus(String line1, String line2) {
  lcd.clear();
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(line2);
}

void readAllSensors() {
  humidity = dht.readHumidity();
  temperature = dht.readTemperature();
  mq4Value = getMQ4PPM();
}

void loop() {
  if (wifiConnected && !client.connected()) {
    connectToMQTT();
  }
  
  if (mqttConnected) {
    client.loop();
  }

  readAllSensors();
  long distance1 = readUltrasonicDistance(ULTRASONIC1_TRIG_PIN, ULTRASONIC1_ECHO_PIN);
  long distance2 = readUltrasonicDistance(ULTRASONIC2_TRIG_PIN, ULTRASONIC2_ECHO_PIN);

  bool irDetection = digitalRead(IR_PROXIMITY_PIN) == LOW;
  bool metalDetection = irDetection && digitalRead(INDUCTIVE_SENSOR_PIN) == LOW;
  bool capacitiveDetection = irDetection && digitalRead(CAPACITIVE_SENSOR_PIN) == LOW;
  bool detection = (distance1 < DETECTION_DISTANCE) || (distance2 < DETECTION_DISTANCE);

  if (irDetection && metalDetection) {
    updateLCDStatus("LOGAM TERDETEKSI", "Kapasitas: " + String(calculateBinCapacity(distance2)) + "%");
    servo1.write(180); 
    delay(3000); 
    servo1.write(0);
    metalWasteCount++;
    sendWasteDetectionEvent("Metal");
  } 
  else if (irDetection && capacitiveDetection) {
    updateLCDStatus("NON LOGAM", "Kapasitas: " + String(calculateBinCapacity(distance2)) + "%");
    servo1.write(180); 
    delay(3000); 
    servo1.write(0);
    inorganicWasteCount++;
    sendWasteDetectionEvent("Non-Metal");
  } 
  else if (irDetection) {
    updateLCDStatus("ORGANIK DETECTED", "Kapasitas: " + String(calculateBinCapacity(distance1)) + "%");
    servo2.write(180); 
    delay(3000); 
    servo2.write(0);
    organicWasteCount++;
    sendWasteDetectionEvent("Organic");
  }

  if (millis() - lastPrintTime >= printInterval) {
    Serial.println("\n=== UPDATE ===");
    Serial.printf("Suhu: %.1f C, Kelembaban: %.1f%%\n", temperature, humidity);
    Serial.printf("MQ4: %.0f ppm\n", mq4Value);
    Serial.printf("Kapasitas Bin1: %d%%, Bin2: %d%%\n", calculateBinCapacity(distance1), calculateBinCapacity(distance2));
    lastPrintTime = millis();
  }

  if (millis() - lastLCDUpdateTime >= lcdUpdateInterval && !detection) {
    static int mode = 0;
    mode = (mode + 1) % 3;
    switch (mode) {
      case 0:
        updateLCDStatus("Pemilah Sampah", wifiConnected ? "WiFi: Connected" : "WiFi: Disconnected");
        break;
      case 1:
        updateLCDStatus("Suhu: " + String(temperature, 1) + " C", "Kelemb: " + String(humidity, 1) + " %");
        break;
      case 2:
        updateLCDStatus("Kapasitas:", "Org:" + String(calculateBinCapacity(distance1)) + "% Anor:" + String(calculateBinCapacity(distance2)) + "%");
        break;
    }
    lastLCDUpdateTime = millis();
  }

  if (mqttConnected && millis() - lastMQTTSend >= mqttInterval) {
    sendSensorData();
    lastMQTTSend = millis();
  }

  delay(100);
}