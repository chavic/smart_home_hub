/*
 * ESP32 Smart Home Hub - Smart Door Lock Controller
 * 
 * This example demonstrates controlling a smart door lock
 * using the Smart Home Hub MQTT protocol.
 * 
 * Hardware:
 * - ESP32 DevKit
 * - Servo motor (SG90) connected to GPIO 21 for lock mechanism
 * - Reed switch/magnetic sensor on GPIO 22 for door position
 * - LED indicator on GPIO 2 for lock status
 * - Optional: Keypad for manual entry
 * 
 * Capabilities: "lck" (Lock)
 * Commands: "lock", "unlock", "status"
 * Events: "status", "door_opened", "door_closed", "lock_changed", "error"
 */

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>

// WiFi credentials
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// MQTT settings
const char* mqtt_server = "your-broker.local";
const int mqtt_port = 8883;
const char* mqtt_user = "hub";
const char* mqtt_pass = "supersecret";

// Device settings
const char* device_id = "lock-001";
const int SERVO_PIN = 21;
const int DOOR_SENSOR_PIN = 22;
const int STATUS_LED_PIN = 2;
const int HEARTBEAT_INTERVAL = 30000; // 30 seconds

// Lock positions
const int LOCKED_POSITION = 0;    // 0 degrees
const int UNLOCKED_POSITION = 90; // 90 degrees

// MQTT topics
String birth_topic = "iot/" + String(device_id) + "/birth";
String event_topic = "iot/" + String(device_id) + "/event";
String cmd_topic = "iot/" + String(device_id) + "/cmd";
String ack_topic = "iot/" + String(device_id) + "/ack";

WiFiClientSecure espClient;
PubSubClient client(espClient);
Servo lockServo;

unsigned long lastHeartbeat = 0;
unsigned long msgCounter = 0;

// Lock state
bool isLocked = true;
bool doorOpen = false;
bool lastDoorState = false;
unsigned long lockTimeout = 0;
bool autoLockEnabled = true;
int autoLockDelay = 10000; // 10 seconds

void setup() {
  Serial.begin(115200);
  
  // Initialize pins
  pinMode(DOOR_SENSOR_PIN, INPUT_PULLUP);
  pinMode(STATUS_LED_PIN, OUTPUT);
  
  // Initialize servo
  lockServo.attach(SERVO_PIN);
  lockServo.write(LOCKED_POSITION); // Start in locked position
  
  // Set initial LED state
  updateStatusLED();
  
  // Connect to WiFi
  setup_wifi();
  
  // Configure MQTT
  espClient.setInsecure(); // For development - use proper certificates in production
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  
  // Connect to MQTT
  reconnect();
  
  // Send birth message
  sendBirth();
  
  Serial.println("Smart door lock controller ready!");
}

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  Serial.println("Received: " + message);
  
  // Parse JSON envelope
  StaticJsonDocument<512> doc;
  deserializeJson(doc, message);
  
  if (doc["v"] == 1 && doc["c"] == "lck") {
    String command = doc["e"];
    int msgId = doc["i"];
    JsonObject data = doc["d"];
    
    handleCommand(command, data, msgId);
  }
}

void handleCommand(String command, JsonObject data, int msgId) {
  bool success = true;
  String result = "";
  
  if (command == "lock") {
    if (doorOpen) {
      success = false;
      result = "Cannot lock - door is open";
    } else {
      lockDoor();
      result = "Door locked";
    }
    
  } else if (command == "unlock") {
    unlockDoor();
    result = "Door unlocked";
    
    // Set auto-lock timer if enabled
    if (autoLockEnabled) {
      lockTimeout = millis() + autoLockDelay;
    }
    
  } else if (command == "status") {
    result = "Status retrieved";
    
  } else if (command == "config") {
    if (data.containsKey("auto_lock")) {
      autoLockEnabled = data["auto_lock"];
      result += "Auto-lock: " + String(autoLockEnabled ? "enabled" : "disabled") + " ";
    }
    if (data.containsKey("auto_lock_delay")) {
      autoLockDelay = data["auto_lock_delay"] * 1000; // Convert to milliseconds
      result += "Auto-lock delay: " + String(autoLockDelay/1000) + "s ";
    }
    if (result == "") {
      result = "Configuration retrieved";
    }
    
  } else {
    success = false;
    result = "Unknown command: " + command;
  }
  
  // Send acknowledgment
  sendAck(msgId, success, result);
  
  // Send status event
  sendEvent("status", createStatusData());
}

void lockDoor() {
  Serial.println("Locking door...");
  lockServo.write(LOCKED_POSITION);
  isLocked = true;
  updateStatusLED();
  lockTimeout = 0; // Clear any pending auto-lock
  
  // Send lock changed event
  StaticJsonDocument<256> doc;
  JsonObject data = doc.to<JsonObject>();
  data["locked"] = true;
  data["timestamp"] = millis();
  sendEvent("lock_changed", data);
}

void unlockDoor() {
  Serial.println("Unlocking door...");
  lockServo.write(UNLOCKED_POSITION);
  isLocked = false;
  updateStatusLED();
  
  // Send lock changed event
  StaticJsonDocument<256> doc;
  JsonObject data = doc.to<JsonObject>();
  data["locked"] = false;
  data["timestamp"] = millis();
  sendEvent("lock_changed", data);
}

void updateStatusLED() {
  // LED on when locked, off when unlocked
  digitalWrite(STATUS_LED_PIN, isLocked ? HIGH : LOW);
}

void checkDoorSensor() {
  bool currentDoorState = digitalRead(DOOR_SENSOR_PIN) == HIGH; // HIGH = door open (reed switch)
  
  if (currentDoorState != lastDoorState) {
    doorOpen = currentDoorState;
    lastDoorState = currentDoorState;
    
    // Send door state event
    StaticJsonDocument<256> doc;
    JsonObject data = doc.to<JsonObject>();
    data["door_open"] = doorOpen;
    data["timestamp"] = millis();
    
    if (doorOpen) {
      sendEvent("door_opened", data);
      Serial.println("Door opened");
      lockTimeout = 0; // Cancel auto-lock when door opens
    } else {
      sendEvent("door_closed", data);
      Serial.println("Door closed");
      
      // Start auto-lock timer if enabled and door is unlocked
      if (autoLockEnabled && !isLocked) {
        lockTimeout = millis() + autoLockDelay;
      }
    }
  }
}

void checkAutoLock() {
  if (lockTimeout > 0 && millis() > lockTimeout && !doorOpen && !isLocked) {
    Serial.println("Auto-locking door...");
    lockDoor();
    
    // Send auto-lock event
    StaticJsonDocument<256> doc;
    JsonObject data = doc.to<JsonObject>();
    data["reason"] = "auto_lock_timeout";
    data["timestamp"] = millis();
    sendEvent("auto_locked", data);
  }
}

JsonObject createStatusData() {
  StaticJsonDocument<256> doc;
  JsonObject data = doc.to<JsonObject>();
  
  data["locked"] = isLocked;
  data["door_open"] = doorOpen;
  data["auto_lock_enabled"] = autoLockEnabled;
  data["auto_lock_delay"] = autoLockDelay / 1000; // Convert to seconds
  data["uptime"] = millis();
  data["wifi_rssi"] = WiFi.RSSI();
  
  return data;
}

void sendBirth() {
  StaticJsonDocument<512> doc;
  doc["v"] = 1;
  doc["t"] = millis();
  doc["c"] = "lck";
  doc["e"] = "birth";
  doc["i"] = msgCounter++;
  
  JsonObject data = doc.createNestedObject("d");
  data["hb"] = 30; // Heartbeat interval in seconds
  data["fw"] = "1.0.0";
  
  JsonArray caps = data.createNestedArray("caps");
  JsonObject cap = caps.createNestedObject();
  cap["c"] = "lck";
  cap["cmds"] = "lock,unlock,status,config";
  cap["events"] = "status,door_opened,door_closed,lock_changed,auto_locked,error";
  
  String message;
  serializeJson(doc, message);
  client.publish(birth_topic.c_str(), message.c_str());
  
  Serial.println("Birth sent: " + message);
}

void sendEvent(String event, JsonObject eventData) {
  StaticJsonDocument<512> doc;
  doc["v"] = 1;
  doc["t"] = millis();
  doc["c"] = "lck";
  doc["e"] = event;
  doc["i"] = msgCounter++;
  doc["d"] = eventData;
  
  String message;
  serializeJson(doc, message);
  client.publish(event_topic.c_str(), message.c_str());
  
  Serial.println("Event sent: " + message);
}

void sendAck(int originalMsgId, bool success, String result) {
  StaticJsonDocument<256> doc;
  doc["v"] = 1;
  doc["t"] = millis();
  doc["c"] = "lck";
  doc["e"] = success ? "ack" : "nack";
  doc["i"] = originalMsgId;
  
  JsonObject data = doc.createNestedObject("d");
  data["result"] = result;
  
  String message;
  serializeJson(doc, message);
  client.publish(ack_topic.c_str(), message.c_str());
  
  Serial.println("Ack sent: " + message);
}

void sendHeartbeat() {
  sendEvent("heartbeat", createStatusData());
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    
    if (client.connect(device_id, mqtt_user, mqtt_pass)) {
      Serial.println("connected");
      client.subscribe(cmd_topic.c_str());
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  
  // Check door sensor
  checkDoorSensor();
  
  // Check auto-lock timer
  checkAutoLock();
  
  // Send heartbeat
  if (millis() - lastHeartbeat > HEARTBEAT_INTERVAL) {
    sendHeartbeat();
    lastHeartbeat = millis();
  }
  
  delay(100);
}
