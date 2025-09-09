/*
 * ESP32 Smart Home Hub - Alarm System Controller
 * 
 * This example demonstrates a comprehensive alarm system
 * using the Smart Home Hub MQTT protocol.
 * 
 * Hardware:
 * - ESP32 DevKit
 * - PIR motion sensor on GPIO 19
 * - Door/window sensors on GPIO 20, 21 (digital inputs)
 * - Siren/buzzer on GPIO 25
 * - Status LED on GPIO 2
 * - Optional: Keypad for arming/disarming
 * 
 * Capabilities: "alm" (Alarm)
 * Commands: "arm", "disarm", "status", "config", "test"
 * Events: "status", "armed", "disarmed", "triggered", "motion", "breach", "error"
 */

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// WiFi credentials
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// MQTT settings
const char* mqtt_server = "your-broker.local";
const int mqtt_port = 8883;
const char* mqtt_user = "hub";
const char* mqtt_pass = "supersecret";

// Device settings
const char* device_id = "alarm-001";
const int PIR_PIN = 19;
const int DOOR1_PIN = 20;
const int DOOR2_PIN = 21;
const int SIREN_PIN = 25;
const int STATUS_LED_PIN = 2;
const int HEARTBEAT_INTERVAL = 30000; // 30 seconds

// MQTT topics
String birth_topic = "iot/" + String(device_id) + "/birth";
String event_topic = "iot/" + String(device_id) + "/event";
String cmd_topic = "iot/" + String(device_id) + "/cmd";
String ack_topic = "iot/" + String(device_id) + "/ack";

WiFiClientSecure espClient;
PubSubClient client(espClient);

unsigned long lastHeartbeat = 0;
unsigned long msgCounter = 0;

// Alarm system state
enum AlarmState {
  DISARMED,
  ARMING,
  ARMED_AWAY,
  ARMED_HOME,
  TRIGGERED,
  ALARM
};

AlarmState currentState = DISARMED;
bool motionDetected = false;
bool door1Open = false;
bool door2Open = false;
bool lastDoor1State = false;
bool lastDoor2State = false;
bool lastMotionState = false;

// Configuration
int armingDelay = 30000;      // 30 seconds to leave
int triggerDelay = 10000;     // 10 seconds entry delay
int alarmDuration = 300000;   // 5 minutes alarm duration

// Timers
unsigned long armingTimer = 0;
unsigned long triggerTimer = 0;
unsigned long alarmTimer = 0;
unsigned long sirenTimer = 0;
bool sirenState = false;

void setup() {
  Serial.begin(115200);
  
  // Initialize pins
  pinMode(PIR_PIN, INPUT);
  pinMode(DOOR1_PIN, INPUT_PULLUP);
  pinMode(DOOR2_PIN, INPUT_PULLUP);
  pinMode(SIREN_PIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);
  
  // Turn off siren initially
  digitalWrite(SIREN_PIN, LOW);
  
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
  
  Serial.println("Alarm system controller ready!");
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
  
  if (doc["v"] == 1 && doc["c"] == "alm") {
    String command = doc["e"];
    int msgId = doc["i"];
    JsonObject data = doc["d"];
    
    handleCommand(command, data, msgId);
  }
}

void handleCommand(String command, JsonObject data, int msgId) {
  bool success = true;
  String result = "";
  
  if (command == "arm") {
    String mode = data.containsKey("mode") ? data["mode"].as<String>() : "away";
    if (mode == "away") {
      startArming(ARMED_AWAY);
      result = "Arming system (away mode)";
    } else if (mode == "home") {
      startArming(ARMED_HOME);
      result = "Arming system (home mode)";
    } else {
      success = false;
      result = "Invalid arm mode: " + mode;
    }
    
  } else if (command == "disarm") {
    disarmSystem();
    result = "System disarmed";
    
  } else if (command == "status") {
    result = "Status retrieved";
    
  } else if (command == "config") {
    if (data.containsKey("arming_delay")) {
      armingDelay = data["arming_delay"] * 1000;
      result += "Arming delay: " + String(armingDelay/1000) + "s ";
    }
    if (data.containsKey("trigger_delay")) {
      triggerDelay = data["trigger_delay"] * 1000;
      result += "Trigger delay: " + String(triggerDelay/1000) + "s ";
    }
    if (data.containsKey("alarm_duration")) {
      alarmDuration = data["alarm_duration"] * 1000;
      result += "Alarm duration: " + String(alarmDuration/1000) + "s ";
    }
    if (result == "") {
      result = "Configuration retrieved";
    }
    
  } else if (command == "test") {
    testSiren();
    result = "Siren test completed";
    
  } else {
    success = false;
    result = "Unknown command: " + command;
  }
  
  // Send acknowledgment
  sendAck(msgId, success, result);
  
  // Send status event
  sendEvent("status", createStatusData());
}

void startArming(AlarmState targetState) {
  currentState = ARMING;
  armingTimer = millis() + armingDelay;
  
  // Send arming event
  StaticJsonDocument<256> doc;
  JsonObject data = doc.to<JsonObject>();
  data["target_state"] = stateToString(targetState);
  data["delay"] = armingDelay / 1000;
  sendEvent("arming", data);
  
  Serial.println("System arming... " + String(armingDelay/1000) + " seconds");
}

void completeArming(AlarmState targetState) {
  currentState = targetState;
  
  // Send armed event
  StaticJsonDocument<256> doc;
  JsonObject data = doc.to<JsonObject>();
  data["mode"] = stateToString(currentState);
  data["timestamp"] = millis();
  sendEvent("armed", data);
  
  Serial.println("System armed: " + stateToString(currentState));
}

void disarmSystem() {
  currentState = DISARMED;
  armingTimer = 0;
  triggerTimer = 0;
  alarmTimer = 0;
  stopSiren();
  
  // Send disarmed event
  StaticJsonDocument<256> doc;
  JsonObject data = doc.to<JsonObject>();
  data["timestamp"] = millis();
  sendEvent("disarmed", data);
  
  Serial.println("System disarmed");
}

void triggerAlarm(String reason, String sensor) {
  if (currentState == DISARMED) return;
  
  if (currentState == ARMED_AWAY || currentState == ARMED_HOME) {
    currentState = TRIGGERED;
    triggerTimer = millis() + triggerDelay;
    
    // Send triggered event
    StaticJsonDocument<256> doc;
    JsonObject data = doc.to<JsonObject>();
    data["reason"] = reason;
    data["sensor"] = sensor;
    data["entry_delay"] = triggerDelay / 1000;
    sendEvent("triggered", data);
    
    Serial.println("Alarm triggered: " + reason + " (" + sensor + ")");
  }
}

void startAlarm() {
  currentState = ALARM;
  alarmTimer = millis() + alarmDuration;
  startSiren();
  
  // Send alarm event
  StaticJsonDocument<256> doc;
  JsonObject data = doc.to<JsonObject>();
  data["duration"] = alarmDuration / 1000;
  data["timestamp"] = millis();
  sendEvent("alarm", data);
  
  Serial.println("ALARM ACTIVATED!");
}

void startSiren() {
  sirenTimer = millis();
  sirenState = true;
  digitalWrite(SIREN_PIN, HIGH);
}

void stopSiren() {
  digitalWrite(SIREN_PIN, LOW);
  sirenState = false;
}

void handleSiren() {
  if (currentState == ALARM && sirenState) {
    // Pulse siren every 500ms
    if (millis() - sirenTimer > 500) {
      digitalWrite(SIREN_PIN, !digitalRead(SIREN_PIN));
      sirenTimer = millis();
    }
  }
}

void testSiren() {
  Serial.println("Testing siren...");
  digitalWrite(SIREN_PIN, HIGH);
  delay(2000);
  digitalWrite(SIREN_PIN, LOW);
}

void checkSensors() {
  // Check motion sensor
  bool currentMotion = digitalRead(PIR_PIN) == HIGH;
  if (currentMotion != lastMotionState) {
    motionDetected = currentMotion;
    lastMotionState = currentMotion;
    
    if (motionDetected) {
      // Send motion event
      StaticJsonDocument<256> doc;
      JsonObject data = doc.to<JsonObject>();
      data["timestamp"] = millis();
      sendEvent("motion", data);
      
      // Trigger alarm if armed
      if (currentState == ARMED_AWAY) {
        triggerAlarm("motion_detected", "pir");
      }
    }
  }
  
  // Check door sensors
  bool currentDoor1 = digitalRead(DOOR1_PIN) == LOW; // LOW = door open (pulled down)
  bool currentDoor2 = digitalRead(DOOR2_PIN) == LOW;
  
  if (currentDoor1 != lastDoor1State) {
    door1Open = currentDoor1;
    lastDoor1State = currentDoor1;
    
    if (door1Open) {
      // Send breach event
      StaticJsonDocument<256> doc;
      JsonObject data = doc.to<JsonObject>();
      data["sensor"] = "door1";
      data["timestamp"] = millis();
      sendEvent("breach", data);
      
      // Trigger alarm if armed
      if (currentState == ARMED_AWAY || currentState == ARMED_HOME) {
        triggerAlarm("door_opened", "door1");
      }
    }
  }
  
  if (currentDoor2 != lastDoor2State) {
    door2Open = currentDoor2;
    lastDoor2State = currentDoor2;
    
    if (door2Open) {
      // Send breach event
      StaticJsonDocument<256> doc;
      JsonObject data = doc.to<JsonObject>();
      data["sensor"] = "door2";
      data["timestamp"] = millis();
      sendEvent("breach", data);
      
      // Trigger alarm if armed
      if (currentState == ARMED_AWAY || currentState == ARMED_HOME) {
        triggerAlarm("door_opened", "door2");
      }
    }
  }
}

void updateTimers() {
  unsigned long now = millis();
  
  // Handle arming timer
  if (currentState == ARMING && now > armingTimer) {
    completeArming(ARMED_AWAY); // Default to away mode
  }
  
  // Handle trigger timer
  if (currentState == TRIGGERED && now > triggerTimer) {
    startAlarm();
  }
  
  // Handle alarm timer
  if (currentState == ALARM && now > alarmTimer) {
    disarmSystem();
  }
}

void updateStatusLED() {
  switch (currentState) {
    case DISARMED:
      digitalWrite(STATUS_LED_PIN, LOW);
      break;
    case ARMING:
      // Blink fast during arming
      digitalWrite(STATUS_LED_PIN, (millis() / 250) % 2);
      break;
    case ARMED_AWAY:
    case ARMED_HOME:
      digitalWrite(STATUS_LED_PIN, HIGH);
      break;
    case TRIGGERED:
      // Blink very fast when triggered
      digitalWrite(STATUS_LED_PIN, (millis() / 100) % 2);
      break;
    case ALARM:
      digitalWrite(STATUS_LED_PIN, HIGH);
      break;
  }
}

String stateToString(AlarmState state) {
  switch (state) {
    case DISARMED: return "disarmed";
    case ARMING: return "arming";
    case ARMED_AWAY: return "armed_away";
    case ARMED_HOME: return "armed_home";
    case TRIGGERED: return "triggered";
    case ALARM: return "alarm";
    default: return "unknown";
  }
}

JsonObject createStatusData() {
  StaticJsonDocument<256> doc;
  JsonObject data = doc.to<JsonObject>();
  
  data["state"] = stateToString(currentState);
  data["motion"] = motionDetected;
  data["door1_open"] = door1Open;
  data["door2_open"] = door2Open;
  data["siren_active"] = sirenState;
  data["arming_delay"] = armingDelay / 1000;
  data["trigger_delay"] = triggerDelay / 1000;
  data["alarm_duration"] = alarmDuration / 1000;
  data["uptime"] = millis();
  data["wifi_rssi"] = WiFi.RSSI();
  
  return data;
}

void sendBirth() {
  StaticJsonDocument<512> doc;
  doc["v"] = 1;
  doc["t"] = millis();
  doc["c"] = "alm";
  doc["e"] = "birth";
  doc["i"] = msgCounter++;
  
  JsonObject data = doc.createNestedObject("d");
  data["hb"] = 30; // Heartbeat interval in seconds
  data["fw"] = "1.0.0";
  
  JsonArray caps = data.createNestedArray("caps");
  JsonObject cap = caps.createNestedObject();
  cap["c"] = "alm";
  cap["cmds"] = "arm,disarm,status,config,test";
  cap["events"] = "status,armed,disarmed,triggered,motion,breach,alarm,error";
  
  String message;
  serializeJson(doc, message);
  client.publish(birth_topic.c_str(), message.c_str());
  
  Serial.println("Birth sent: " + message);
}

void sendEvent(String event, JsonObject eventData) {
  StaticJsonDocument<512> doc;
  doc["v"] = 1;
  doc["t"] = millis();
  doc["c"] = "alm";
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
  doc["c"] = "alm";
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
  
  // Check all sensors
  checkSensors();
  
  // Update timers
  updateTimers();
  
  // Handle siren
  handleSiren();
  
  // Update status LED
  updateStatusLED();
  
  // Send heartbeat
  if (millis() - lastHeartbeat > HEARTBEAT_INTERVAL) {
    sendHeartbeat();
    lastHeartbeat = millis();
  }
  
  delay(50);
}
