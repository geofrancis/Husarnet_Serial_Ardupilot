#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <husarnet.h>
#include <MAVLink.h>

const char* ssid = "2.4";
const char* password = "password";
const int tcpPort = 8888;
const int baudRate = 500000;

#define HOSTNAME "Ardupilot"
#define JOIN_CODE "***************"

// Separate task handles for safety
TaskHandle_t HusarnetTaskHandle = NULL;
TaskHandle_t SerialT_TaskHandle = NULL;
TaskHandle_t SerialR_TaskHandle = NULL;

#define RX2 11
#define TX2 12

WiFiServer server(tcpPort);
WiFiClient client;
HusarnetClient husarnet;

// Connection monitoring timers
unsigned long lastConnCheck = 0;
const unsigned long connCheckInterval = 5000; // Check every 5 seconds
int husarnetFailCount = 0;

void husarnettask(void* pvParameters) {
  Serial.println("Starting Husarnet initialization...");
  husarnet.join(HOSTNAME, JOIN_CODE);
  
  int count = 0;
  while (!husarnet.isJoined()) {
    Serial.println("Waiting for Husarnet network...");
    vTaskDelay(pdMS_TO_TICKS(1000));

    count++;
    if (count > 20) {
      Serial.println("Husarnet initial join failed. Loop will handle retries.");
      break; 
    }
  }
  
  if (husarnet.isJoined()) {
    Serial.println("Husarnet joined successfully!");
  }
  
  HusarnetTaskHandle = NULL;
  vTaskDelete(NULL); // Delete task safely; loop() will monitor state
}

// TCP -> Serial2
void SerialtaskT(void* pvParameters) {
  uint8_t tcpBuf[256]; 
  for (;;) {
    if (client && client.connected() && client.available()) {
      int len = client.read(tcpBuf, sizeof(tcpBuf));
      if (len > 0) {
        Serial2.write(tcpBuf, len);
      }
    } else {
      vTaskDelay(pdMS_TO_TICKS(10)); // Yield more when idle
    }
    vTaskDelay(pdMS_TO_TICKS(1)); 
  }
}

// Serial2 -> TCP
void SerialtaskR(void* pvParameters) {
  uint8_t serialBuf[256];
  for (;;) {
    if (Serial2.available()) {
      int bytesRead = 0;
      while (Serial2.available() && bytesRead < sizeof(serialBuf)) {
        serialBuf[bytesRead++] = Serial2.read();
      }
      
      if (client && client.connected() && bytesRead > 0) {
        client.write(serialBuf, bytesRead);
      }
    } else {
      vTaskDelay(pdMS_TO_TICKS(1)); 
    }
  }
}

// Helper to safely spin up Husarnet task if it's not running
void startHusarnetTask() {
  if (HusarnetTaskHandle == NULL) {
    xTaskCreatePinnedToCore(husarnettask, "HusarnetTask", 10000, NULL, 4, &HusarnetTaskHandle, 0);
  }
}

void setup() {
  Serial.begin(115200);
  
  Serial2.setRxBufferSize(2048); 
  Serial2.begin(baudRate, SERIAL_8N1, RX2, TX2);

  // Configure Wi-Fi for auto-reconnect at the base level
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);
  
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  
  // Quick initial check, but won't hang indefinitely if AP is down at boot
  int initTimeout = 0;
  while (WiFi.status() != WL_CONNECTED && initTimeout < 30) {
    delay(500);
    Serial.print(".");
    initTimeout++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi Connected!");
    startHusarnetTask();
  } else {
    Serial.println("\nWiFi boot connection timeout. Entering autonomous loop.");
  }

  server.begin();
  Serial.printf("TCP Server started on port %d\n", tcpPort);

  xTaskCreatePinnedToCore(SerialtaskT,  "SerialToTCP",   10000, NULL, 4, &SerialT_TaskHandle,  1);
  xTaskCreatePinnedToCore(SerialtaskR,  "TCPToSerial",   10000, NULL, 5, &SerialR_TaskHandle,  1);
}

void loop() {
  unsigned long currentMillis = millis();

  // --- CONNECTION MANAGEMENT (Every 5 seconds) ---
  if (currentMillis - lastConnCheck >= connCheckInterval) {
    lastConnCheck = currentMillis;

    // 1. Check Wi-Fi Status
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[WARN] WiFi disconnected. Attempting reconnection...");
      WiFi.disconnect();
      WiFi.begin(ssid, password);
      
      // Clear client if connection dropped
      if (client) {
        client.stop();
      }
    } 
    // 2. Check VPN (Husarnet) Status if Wi-Fi is good
    else {
      if (!husarnet.isJoined()) {
        husarnetFailCount++;
        Serial.printf("[WARN] Husarnet not joined. Failure count: %d/12\n", husarnetFailCount);
        
        // Force re-trigger the join task
        startHusarnetTask();

        // If Husarnet is completely stuck for 60 seconds (12 checks * 5s), reboot ESP
        if (husarnetFailCount >= 12) {
          Serial.println("[CRITICAL] Husarnet unresolved. Rebooting ESP...");
          ESP.restart();
        }
      } else {
        husarnetFailCount = 0; // Reset counter on successful verification
      }
    }
  }

  // --- CLIENT LISTENER ---
  if (!client || !client.connected()) {
    WiFiClient newClient = server.available();
    if (newClient) {
      client = newClient;
      client.setNoDelay(true); // Keep Nagle's algorithm disabled for MAVLink
      Serial.println("New TCP Client connected!");
    }
  }

  delay(10);
}
