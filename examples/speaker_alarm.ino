/*
 * ESP32 Smart Home Hub - Speaker Alarm Controller
 * 
 * This example demonstrates controlling a speaker for alarm functionality
 * using the Smart Home Hub MQTT protocol.
 * 
 * Hardware:
 * - ESP32 DevKit
 * - Speaker/Buzzer connected to GPIO 25 (PWM capable pin)
 * - Optional: Amplifier module for better sound quality
 * 
 * Capabilities: "spk" (Speaker)
 * Commands: "on", "off", "beep", "alarm", "tone"
 * Events: "status", "error"
 */

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <esp_task_wdt.h>

// WiFi credentials
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// MQTT settings
const char* mqtt_server = "your-broker.local";
const int mqtt_port = 8883;
const char* mqtt_user = "hub";
const char* mqtt_pass = "supersecret";

// Device settings
const char* device_id = "speaker-001";
const int SPEAKER_PIN = 25;
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
bool isPlaying = false;
int currentFreq = 0;

// Alarm patterns
struct AlarmPattern {
  int freq;
  int duration;
};

AlarmPattern fireAlarm[] = {{800, 500}, {1000, 500}, {800, 500}, {1000, 500}, {0, 0}};
AlarmPattern burglarAlarm[] = {{2000, 200}, {0, 200}, {2000, 200}, {0, 200}, {2000, 200}, {0, 1000}, {0, 0}};

void setup() {
  Serial.begin(115200);
  
  // Initialize speaker pin
  pinMode(SPEAKER_PIN, OUTPUT);
  ledcSetup(0, 1000, 8); // Channel 0, 1000 Hz, 8-bit resolution
  ledcAttachPin(SPEAKER_PIN, 0);
  
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
  
  Serial.println("Speaker alarm controller ready!");
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
  
  if (doc["v"] == 1 && doc["c"] == "spk") {
    String command = doc["e"];
    int msgId = doc["i"];
    JsonObject data = doc["d"];
    
    handleCommand(command, data, msgId);
  }
}

void handleCommand(String command, JsonObject data, int msgId) {
  bool success = true;
  String result = "";
  
  if (command == "on") {
    int freq = data.containsKey("freq") ? data["freq"] : 1000;
    playTone(freq, 0); // Continuous tone
    result = "Speaker on at " + String(freq) + "Hz";
    
  } else if (command == "off") {
    stopTone();
    result = "Speaker off";
    
  } else if (command == "beep") {
    int freq = data.containsKey("freq") ? data["freq"] : 1000;
    int duration = data.containsKey("duration") ? data["duration"] : 500;
    playTone(freq, duration);
    result = "Beep played";
    
  } else if (command == "alarm") {
    String type = data.containsKey("type") ? data["type"].as<String>() : "fire";
    playAlarm(type);
    result = "Alarm " + type + " started";
    
  } else if (command == "tone") {
    int freq = data["freq"];
    int duration = data.containsKey("duration") ? data["duration"] : 1000;
    playTone(freq, duration);
    result = "Tone played";
    
  } else {
    success = false;
    result = "Unknown command: " + command;
  }
  
  // Send acknowledgment
  sendAck(msgId, success, result);
  
  // Send status event
  sendEvent("status", createStatusData());
}

void playTone(int frequency, int duration) {
  if (frequency > 0) {
    ledcWriteTone(0, frequency);
    isPlaying = true;
    currentFreq = frequency;
    
    if (duration > 0) {
      delay(duration);
      stopTone();
    }
  }
}

void stopTone() {
  ledcWriteTone(0, 0);
  isPlaying = false;
  currentFreq = 0;
}

void playAlarm(String type) {
  AlarmPattern* pattern;
  
  if (type == "fire") {
    pattern = fireAlarm;
  } else if (type == "burglar") {
    pattern = burglarAlarm;
  } else {
    // Default pattern
    playTone(1000, 2000);
    return;
  }
  
  // Play pattern in background task
  for (int i = 0; pattern[i].freq != 0 || pattern[i].duration != 0; i++) {
    if (pattern[i].freq > 0) {
      ledcWriteTone(0, pattern[i].freq);
      isPlaying = true;
      currentFreq = pattern[i].freq;
    } else {
      ledcWriteTone(0, 0);
      isPlaying = false;
      currentFreq = 0;
    }
    delay(pattern[i].duration);
  }
  
  stopTone();
}

JsonObject createStatusData() {
  StaticJsonDocument<256> doc;
  JsonObject data = doc.to<JsonObject>();
  
  data["playing"] = isPlaying;
  data["frequency"] = currentFreq;
  data["uptime"] = millis();
  data["wifi_rssi"] = WiFi.RSSI();
  
  return data;
}

void sendBirth() {
  StaticJsonDocument<512> doc;
  doc["v"] = 1;
  doc["t"] = millis();
  doc["c"] = "spk";
  doc["e"] = "birth";
  doc["i"] = msgCounter++;
  
  JsonObject data = doc.createNestedObject("d");
  data["hb"] = 30; // Heartbeat interval in seconds
  data["fw"] = "1.0.0";
  
  JsonArray caps = data.createNestedArray("caps");
  JsonObject cap = caps.createNestedObject();
  cap["c"] = "spk";
  cap["cmds"] = "on,off,beep,alarm,tone";
  cap["events"] = "status,error";
  
  String message;
  serializeJson(doc, message);
  client.publish(birth_topic.c_str(), message.c_str());
  
  Serial.println("Birth sent: " + message);
}

void sendEvent(String event, JsonObject eventData) {
  StaticJsonDocument<512> doc;
  doc["v"] = 1;
  doc["t"] = millis();
  doc["c"] = "spk";
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
  doc["c"] = "spk";
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
  
  // Send heartbeat
  if (millis() - lastHeartbeat > HEARTBEAT_INTERVAL) {
    sendHeartbeat();
    lastHeartbeat = millis();
  }
  
  delay(100);
}
