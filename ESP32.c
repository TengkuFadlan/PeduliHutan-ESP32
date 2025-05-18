#include <esp_now.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

const char* ssid = "BuzzNet 2.4Ghz";
const char* password = "BuzzNet6988";

const char* serverName = "http://44.214.24.233/api/esp-status";

#define TRIG_PIN 21    // Trigger pin for HC-SR04
#define ECHO_PIN 19    // Echo pin for HC-SR04 (via voltage divider)
#define RELAY_PIN 23   // Relay pin
#define BUZZER_PIN 17  // Buzzer pin

// FIRE COUNTER
#define FIRE_COUNTER 5
#define FIRE_SUCCESSION_TIME_MAX 2000
#define FIRE_SUCCESSION_TIME_MIN 500

unsigned long lastFoundFireMillis = 0;
int fireRecieveCounter = 0;

int sentFireStatus = 0;

// WATCHDOG TIMER
unsigned long lastDataReceivedMillis = 0;      // Waktu terakhir data diterima via ESP-NOW
const unsigned long WATCHDOG_TIMEOUT = 30000;  // Timeout 10 detik (dalam milidetik)

// HTTP CALL FLAG
volatile bool sendFireStatusFlag = false;
volatile char fireStatusToSend = '0';

float getWaterHeight() {
  // Measure distance using HC-SR04
  long duration;
  float distanceCm;

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH, 30000);  // Timeout after 30ms
  distanceCm = duration * 0.034 / 2;

  if (duration == 0) {
    Serial.println("No echo received (timeout).");
    return 0;
  } else {
    Serial.print("Distance: ");
    Serial.print(distanceCm);
    Serial.println(" cm");
  }

  return distanceCm;
}

void httpSend(char fireStatus) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(serverName);  // Langsung tanpa SSL
    http.addHeader("Content-Type", "application/json");

    float currentWaterHeight = getWaterHeight();
    String stringCurrentWaterHeight = String(currentWaterHeight);


    String fireStatusBool = "false";
    if (fireStatus == '1') {
      fireStatusBool = "true";
    }

    // Data JSON dummy
    String jsonData = "{\"water_level\":" + stringCurrentWaterHeight + ",\"fire\":" + fireStatusBool + "}";

    int httpResponseCode = http.POST(jsonData);

    Serial.print("Kode respon HTTP: ");
    Serial.println(httpResponseCode);

    if (httpResponseCode > 0) {
      String response = http.getString();
      Serial.println("Isi respon dari server:");
      Serial.println(response);

      // Parse JSON dari response
      DynamicJsonDocument doc(1024);
      DeserializationError error = deserializeJson(doc, response);

      if (!error) {
        const char* status = doc["status"];
        const char* message = doc["message"];

        Serial.print("Status dari JSON: ");
        Serial.println(status);

        Serial.print("Pesan dari JSON: ");
        Serial.println(message);
      } else {
        Serial.print("Gagal parse JSON: ");
        Serial.println(error.c_str());
      }

    } else {
      Serial.print("Gagal kirim data. Error: ");
      Serial.println(http.errorToString(httpResponseCode).c_str());
    }

    http.end();
  } else {
    Serial.println("WiFi tidak terkoneksi.");
  }
}

void tryHttpSend(char fireStatus) {
  fireStatusToSend = fireStatus;
  sendFireStatusFlag = true;
}

void SendOnFire() {
  if (sentFireStatus == 1) return;

  sentFireStatus = 1;

  Serial.println("ON FIRE: Activating relay and buzzer.");
  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(BUZZER_PIN, HIGH);

  tryHttpSend('1');
}

void SendNoFire() {
  if (sentFireStatus == 0) return;

  sentFireStatus = 0;

  Serial.println("NOT ON FIRE: Deactivating relay and buzzer.");
  digitalWrite(RELAY_PIN, HIGH);
  digitalWrite(BUZZER_PIN, LOW);

  tryHttpSend('0');
}

// Callback saat data diterima via ESP-NOW
void OnDataRecv(const esp_now_recv_info_t* recv_info, const uint8_t* incomingData, int len) {
  char input = ((char*)incomingData)[0];

  // Update waktu terakhir data diterima (WATCHDOG TIMER)
  lastDataReceivedMillis = millis();

  unsigned long differenceMillis = millis() - lastFoundFireMillis;

  if (input == '1') {

    if (differenceMillis >= FIRE_SUCCESSION_TIME_MIN) {
      lastFoundFireMillis = millis();
      fireRecieveCounter++;
    }

    if (fireRecieveCounter >= FIRE_COUNTER) {
      SendOnFire();
    }

  } else if (input == '0') {
    if (differenceMillis > FIRE_SUCCESSION_TIME_MAX) {
      fireRecieveCounter = 0;
      SendNoFire();
    }
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  digitalWrite(RELAY_PIN, HIGH);  // Initially off
  digitalWrite(BUZZER_PIN, LOW);  // Initially off

  // Set device as a Wi-Fi Station (required for ESP-NOW)
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register callback untuk menerima data
  esp_now_register_recv_cb(OnDataRecv);

  Serial.println("ESP-NOW Receiver ready.");

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi Connected!");

  // Inisialisasi lastDataReceivedMillis saat startup (WATCHDOG TIMER)
  lastDataReceivedMillis = millis();

  tryHttpSend('0');
}

void loop() {
  // WATCHDOG TIMER: cek timeout data diterima
  if (millis() - lastDataReceivedMillis > WATCHDOG_TIMEOUT) {
    Serial.println("No data received via ESP-NOW for timeout period. Restarting...");
    ESP.restart();
  }

  if (sendFireStatusFlag) {
    sendFireStatusFlag = false;
    httpSend(fireStatusToSend);
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi lost, reconnecting...");
    WiFi.reconnect();
    delay(1000);
  }
}
