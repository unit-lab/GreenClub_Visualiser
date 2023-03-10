#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <EEPROM.h>
#include <ESP32Servo.h>

#define SELECT_PIN_PM25 12
#define SELECT_PIN_PM10 14
#define SELECT_PIN_TEMP 27
#define SELECT_PIN_HUMIDITY 26
#define SELECT_PIN_PRESSURE 25

enum {
  pm25,
  pm10,
  temp,
  humidity,
  pressure
};

int minimums[] = {
  0,    // PM 2.5
  0,    // PM 10
  -20,  // Temperature
  0,    // Humidity
  870   // Pressure
};

int maximums[] = {
  200,  // PM 2.5
  500,  // PM 10
  50,   // Temperature
  100,  // Humidity
  1084  // Pressure
};

#define MAX_CHANNEL 13

#define LED_BUILTIN 2

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Structure to send data to visualisers
typedef struct struct_message {
  float temp;
  float pressure;
  float humidity;
  uint16_t pm25;
  uint16_t pm10;
} struct_message;

// Structure for pairing visualisers
typedef struct struct_pairing {
  uint8_t macAddr[6];
  uint8_t channel;
} struct_pairing;

struct_message inData;
struct_pairing pairingData;
esp_now_peer_info_t peerInfo;

boolean isPaired;
#define PAIR_REQUEST_WAIT 1000
unsigned long lastPairRequest;
int currentChannel;

Servo servo;
int servoPin = 13;

int currentDataSelected;

void printMAC(const uint8_t * mac_addr) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  isPaired = true;

  Serial.print("Packet received from: ");
  printMAC(mac_addr);
  Serial.println();
  Serial.print("data size = ");
  Serial.println(sizeof(incomingData));

  memcpy(&inData, incomingData, sizeof(inData));
  Serial.print("Temp = ");
  Serial.println(inData.temp);
  Serial.print("Pressure = ");
  Serial.println(inData.pressure);
  Serial.print("Humidity = ");
  Serial.println(inData.humidity);
  Serial.print("PM2.5 = ");
  Serial.println(inData.pm25);
  Serial.print("PM10 = ");
  Serial.println(inData.pm10);

  int dataToWrite;

  switch (currentDataSelected) {
    case pm25:
      dataToWrite = inData.pm25;
      break;
    case pm10:
      dataToWrite = inData.pm10;
      break;
    case temp:
      dataToWrite = int(inData.temp);
      break;
    case humidity:
      dataToWrite = int(inData.humidity);
      break;
    case pressure:
      dataToWrite = int(inData.pressure);
      break;
  }

  // Write data to the servo motor!
  writeDataToServo(dataToWrite);
}

boolean autoPaired() {
  if (isPaired) {
    return true;
  }
  else {
    if (millis() - lastPairRequest >= PAIR_REQUEST_WAIT) {
      lastPairRequest = millis();
      // Let's switch channel and ask for another pair.
      currentChannel++;
      if (currentChannel > MAX_CHANNEL) currentChannel = 1;

      initESPNow(currentChannel);

      // Send pairing message
      Serial.print("Sent pairing request to channel ");
      Serial.println(currentChannel);
      pairingData.channel = currentChannel;
      esp_now_send(broadcastAddress, (uint8_t *) &pairingData, sizeof(pairingData));
    }
    return false;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.print("Client Board MAC Address:  ");
  Serial.println(WiFi.macAddress());
  initESPNow(1);
  setupServo();
  currentDataSelected = getDataSelected();
}

void loop() {
  if (autoPaired()) {
    digitalWrite(LED_BUILTIN, HIGH);
  }
  else {
    digitalWrite(LED_BUILTIN, LOW);
  }
  vTaskDelay(1);
}

void initESPNow(int chan) {
  WiFi.mode(WIFI_STA);
  esp_wifi_start();
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(chan, WIFI_SECOND_CHAN_NONE);

  Serial.print("Channel ");
  Serial.println(chan);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  else {
    Serial.println("ESP-NOW initialised.");
  }

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = WiFi.channel();
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
  }

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
}

void setupServo() {
  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servo.setPeriodHertz(50);    // standard 50 hz servo
  servo.attach(servoPin, 150, 2500);
  servo.write(0);
  delay(1000);
  servo.write(180);
  delay(500);
}

void writeDataToServo(int data) {
  Serial.print("Value: ");
  Serial.print(data);
  Serial.print("\tAngle: ");
  int angle = map(data, minimums[currentDataSelected], maximums[currentDataSelected], 180, 0);
  angle = constrain(angle, 0, 180);
  Serial.println(angle);
  servo.write(angle);
}

int getDataSelected() {
  int dataSelected = pm25;
  pinMode(SELECT_PIN_PM25, INPUT_PULLUP);
  pinMode(SELECT_PIN_PM10, INPUT_PULLUP);
  pinMode(SELECT_PIN_TEMP, INPUT_PULLUP);
  pinMode(SELECT_PIN_HUMIDITY, INPUT_PULLUP);
  pinMode(SELECT_PIN_PRESSURE, INPUT_PULLUP);

  if (digitalRead(SELECT_PIN_PM25) == LOW) dataSelected = pm25;
  else if (digitalRead(SELECT_PIN_PM10) == LOW) dataSelected = pm10;
  else if (digitalRead(SELECT_PIN_TEMP) == LOW) dataSelected = temp;
  else if (digitalRead(SELECT_PIN_HUMIDITY) == LOW) dataSelected = humidity;
  else if (digitalRead(SELECT_PIN_PRESSURE) == LOW) dataSelected = pressure;

  Serial.print("Data selected: ");
  switch (dataSelected) {
    case pm25:
      Serial.println("PM 2.5");
      break;
    case pm10:
      Serial.println("PM 10");
      break;
    case temp:
      Serial.println("temperature");
      break;
    case humidity:
      Serial.println("humidity");
      break;
    case pressure:
      Serial.println("pressure");
      break;
  }

  return dataSelected;
}
