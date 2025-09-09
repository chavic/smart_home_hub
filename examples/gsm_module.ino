/*
 * ESP32 Smart Home Hub - GSM Module Controller
 * 
 * This example demonstrates GSM/cellular communication for SMS notifications
 * and remote monitoring using the Smart Home Hub MQTT protocol.
 * 
 * Hardware:
 * - ESP32 DevKit
 * - SIM800L GSM module connected via Serial2:
 *   - TX: GPIO 17
 *   - RX: GPIO 16
 *   - Power: 3.7-4.2V (use external power supply)
 *   - RST: GPIO 4 (optional)
 * - Status LED on GPIO 2
 * 
 * Capabilities: "gsm" (GSM Module)
 * Commands: "sms", "call", "status", "config", "signal"
 * Events: "sms_sent", "sms_received", "call_status", "signal_update", "status", "error"
 */

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <HardwareSerial.h>

// WiFi credentials
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// MQTT settings
const char* mqtt_server = "your-broker.local";
const int mqtt_port = 8883;
const char* mqtt_user = "hub";
const char* mqtt_pass = "supersecret";

// Device settings
const char* device_id = "gsm-001";
const int GSM_RST_PIN = 4;
const int STATUS_LED_PIN = 2;
const int HEARTBEAT_INTERVAL = 30000; // 30 seconds

// GSM module serial connection
HardwareSerial gsmSerial(2); // Use Serial2 (GPIO 16=RX, GPIO 17=TX)

// MQTT topics
String birth_topic = "iot/" + String(device_id) + "/birth";
String event_topic = "iot/" + String(device_id) + "/event";
String cmd_topic = "iot/" + String(device_id) + "/cmd";
String ack_topic = "iot/" + String(device_id) + "/ack";

WiFiClientSecure espClient;
PubSubClient client(espClient);

unsigned long lastHeartbeat = 0;
unsigned long lastSignalCheck = 0;
unsigned long msgCounter = 0;

// GSM state
bool gsmReady = false;
int signalStrength = 0;
String networkOperator = "";
String simStatus = "";
bool smsReady = false;

// Configuration
String emergencyNumbers[5] = {"", "", "", "", ""}; // Up to 5 emergency contacts
int emergencyCount = 0;
bool autoForwardSMS = false;
int signalCheckInterval = 60000; // Check signal every minute

void setup() {
  Serial.begin(115200);
  
  // Initialize GSM serial
  gsmSerial.begin(9600, SERIAL_8N1, 16, 17); // RX=16, TX=17
  
  // Initialize pins
  pinMode(GSM_RST_PIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);
  
  // Reset GSM module
  resetGSM();
  
  // Connect to WiFi
  setup_wifi();
  
  // Configure MQTT
  espClient.setInsecure(); // For development - use proper certificates in production
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  
  // Connect to MQTT
  reconnect();
  
  // Initialize GSM module
  initializeGSM();
  
  // Send birth message
  sendBirth();
  
  Serial.println("GSM module controller ready!");
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
  
  if (doc["v"] == 1 && doc["c"] == "gsm") {
    String command = doc["e"];
    int msgId = doc["i"];
    JsonObject data = doc["d"];
    
    handleCommand(command, data, msgId);
  }
}

void handleCommand(String command, JsonObject data, int msgId) {
  bool success = true;
  String result = "";
  
  if (command == "sms") {
    if (data.containsKey("number") && data.containsKey("message")) {
      String number = data["number"];
      String message = data["message"];
      success = sendSMS(number, message);
      result = success ? "SMS sent to " + number : "Failed to send SMS";
    } else {
      success = false;
      result = "Missing number or message parameters";
    }
    
  } else if (command == "call") {
    if (data.containsKey("number")) {
      String number = data["number"];
      success = makeCall(number);
      result = success ? "Call initiated to " + number : "Failed to initiate call";
    } else {
      success = false;
      result = "Missing number parameter";
    }
    
  } else if (command == "status") {
    checkGSMStatus();
    result = "Status updated";
    
  } else if (command == "config") {
    if (data.containsKey("emergency_numbers")) {
      JsonArray numbers = data["emergency_numbers"];
      emergencyCount = 0;
      for (int i = 0; i < min((int)numbers.size(), 5); i++) {
        emergencyNumbers[i] = numbers[i].as<String>();
        if (emergencyNumbers[i] != "") emergencyCount++;
      }
      result += "Emergency numbers updated (" + String(emergencyCount) + " numbers) ";
    }
    if (data.containsKey("auto_forward_sms")) {
      autoForwardSMS = data["auto_forward_sms"];
      result += "Auto-forward SMS: " + String(autoForwardSMS ? "enabled" : "disabled") + " ";
    }
    if (data.containsKey("signal_check_interval")) {
      signalCheckInterval = data["signal_check_interval"] * 1000;
      result += "Signal check interval: " + String(signalCheckInterval/1000) + "s ";
    }
    if (result == "") {
      result = "Configuration retrieved";
    }
    
  } else if (command == "signal") {
    checkSignalStrength();
    result = "Signal strength: " + String(signalStrength) + " dBm";
    
  } else {
    success = false;
    result = "Unknown command: " + command;
  }
  
  // Send acknowledgment
  sendAck(msgId, success, result);
  
  // Send status event
  sendEvent("status", createStatusData());
}

void resetGSM() {
  Serial.println("Resetting GSM module...");
  digitalWrite(GSM_RST_PIN, LOW);
  delay(1000);
  digitalWrite(GSM_RST_PIN, HIGH);
  delay(2000);
}

void initializeGSM() {
  Serial.println("Initializing GSM module...");
  
  // Test AT communication
  for (int i = 0; i < 5; i++) {
    gsmSerial.println("AT");
    if (waitForResponse("OK", 5000)) {
      Serial.println("GSM module responding");
      break;
    }
    delay(1000);
  }
  
  // Disable echo
  gsmSerial.println("ATE0");
  waitForResponse("OK", 5000);
  
  // Check SIM card status
  gsmSerial.println("AT+CPIN?");
  String response = getGSMResponse(5000);
  if (response.indexOf("READY") >= 0) {
    simStatus = "ready";
    Serial.println("SIM card ready");
  } else {
    simStatus = "error";
    Serial.println("SIM card error: " + response);
  }
  
  // Set SMS text mode
  gsmSerial.println("AT+CMGF=1");
  waitForResponse("OK", 5000);
  
  // Enable SMS notifications
  gsmSerial.println("AT+CNMI=1,2,0,0,0");
  waitForResponse("OK", 5000);
  
  // Get network operator
  gsmSerial.println("AT+COPS?");
  response = getGSMResponse(10000);
  int startPos = response.indexOf("\"");
  int endPos = response.indexOf("\"", startPos + 1);
  if (startPos >= 0 && endPos > startPos) {
    networkOperator = response.substring(startPos + 1, endPos);
  }
  
  // Check signal strength
  checkSignalStrength();
  
  gsmReady = (simStatus == "ready");
  smsReady = gsmReady;
  
  Serial.println("GSM initialization complete. Ready: " + String(gsmReady));
}

bool sendSMS(String number, String message) {
  if (!smsReady) {
    Serial.println("SMS not ready");
    return false;
  }
  
  Serial.println("Sending SMS to " + number + ": " + message);
  
  gsmSerial.println("AT+CMGS=\"" + number + "\"");
  delay(1000);
  
  gsmSerial.print(message);
  gsmSerial.write(26); // Ctrl+Z to send
  
  if (waitForResponse("OK", 30000)) {
    Serial.println("SMS sent successfully");
    
    // Send SMS sent event
    StaticJsonDocument<512> doc;
    JsonObject data = doc.to<JsonObject>();
    data["number"] = number;
    data["message"] = message;
    data["timestamp"] = millis();
    sendEvent("sms_sent", data);
    
    return true;
  } else {
    Serial.println("Failed to send SMS");
    return false;
  }
}

bool makeCall(String number) {
  if (!gsmReady) {
    Serial.println("GSM not ready");
    return false;
  }
  
  Serial.println("Calling " + number);
  
  gsmSerial.println("ATD" + number + ";");
  
  if (waitForResponse("OK", 10000)) {
    Serial.println("Call initiated");
    
    // Send call status event
    StaticJsonDocument<256> doc;
    JsonObject data = doc.to<JsonObject>();
    data["number"] = number;
    data["status"] = "initiated";
    data["timestamp"] = millis();
    sendEvent("call_status", data);
    
    return true;
  } else {
    Serial.println("Failed to initiate call");
    return false;
  }
}

void checkSignalStrength() {
  gsmSerial.println("AT+CSQ");
  String response = getGSMResponse(5000);
  
  int startPos = response.indexOf("+CSQ: ");
  if (startPos >= 0) {
    int commaPos = response.indexOf(",", startPos);
    if (commaPos > startPos) {
      int rssi = response.substring(startPos + 6, commaPos).toInt();
      if (rssi != 99) {
        signalStrength = -113 + (rssi * 2); // Convert to dBm
      } else {
        signalStrength = -999; // No signal
      }
    }
  }
  
  Serial.println("Signal strength: " + String(signalStrength) + " dBm");
  
  // Send signal update event
  StaticJsonDocument<256> doc;
  JsonObject data = doc.to<JsonObject>();
  data["signal_dbm"] = signalStrength;
  data["signal_bars"] = getSignalBars(signalStrength);
  data["timestamp"] = millis();
  sendEvent("signal_update", data);
}

int getSignalBars(int dbm) {
  if (dbm >= -70) return 5;
  if (dbm >= -80) return 4;
  if (dbm >= -90) return 3;
  if (dbm >= -100) return 2;
  if (dbm >= -110) return 1;
  return 0;
}

void checkGSMStatus() {
  // Check network registration
  gsmSerial.println("AT+CREG?");
  String response = getGSMResponse(5000);
  
  bool networkRegistered = (response.indexOf(",1") >= 0 || response.indexOf(",5") >= 0);
  
  // Update ready status
  gsmReady = (simStatus == "ready") && networkRegistered;
  smsReady = gsmReady;
  
  // Update status LED
  digitalWrite(STATUS_LED_PIN, gsmReady ? HIGH : LOW);
}

void checkIncomingSMS() {
  if (gsmSerial.available()) {
    String response = gsmSerial.readString();
    
    // Check for incoming SMS notification
    if (response.indexOf("+CMT:") >= 0) {
      Serial.println("Incoming SMS detected");
      
      // Parse SMS details
      int startPos = response.indexOf("+CMT:");
      int endPos = response.indexOf("\n", startPos);
      String smsHeader = response.substring(startPos, endPos);
      
      // Extract sender number
      int numberStart = smsHeader.indexOf("\"") + 1;
      int numberEnd = smsHeader.indexOf("\"", numberStart);
      String senderNumber = smsHeader.substring(numberStart, numberEnd);
      
      // Extract message content (usually on the next line)
      int messageStart = response.indexOf("\n", endPos) + 1;
      int messageEnd = response.indexOf("\n", messageStart);
      if (messageEnd == -1) messageEnd = response.length();
      String messageContent = response.substring(messageStart, messageEnd);
      messageContent.trim();
      
      Serial.println("SMS from " + senderNumber + ": " + messageContent);
      
      // Send SMS received event
      StaticJsonDocument<512> doc;
      JsonObject data = doc.to<JsonObject>();
      data["from"] = senderNumber;
      data["message"] = messageContent;
      data["timestamp"] = millis();
      sendEvent("sms_received", data);
      
      // Auto-forward to emergency contacts if enabled
      if (autoForwardSMS && emergencyCount > 0) {
        String forwardMessage = "SMS from " + senderNumber + ": " + messageContent;
        for (int i = 0; i < emergencyCount; i++) {
          if (emergencyNumbers[i] != senderNumber) { // Don't forward back to sender
            sendSMS(emergencyNumbers[i], forwardMessage);
          }
        }
      }
    }
  }
}

bool waitForResponse(String expected, unsigned long timeout) {
  unsigned long startTime = millis();
  String response = "";
  
  while (millis() - startTime < timeout) {
    if (gsmSerial.available()) {
      response += gsmSerial.readString();
      if (response.indexOf(expected) >= 0) {
        return true;
      }
    }
    delay(100);
  }
  
  return false;
}

String getGSMResponse(unsigned long timeout) {
  unsigned long startTime = millis();
  String response = "";
  
  while (millis() - startTime < timeout) {
    if (gsmSerial.available()) {
      response += gsmSerial.readString();
    }
    delay(100);
  }
  
  return response;
}

JsonObject createStatusData() {
  StaticJsonDocument<512> doc;
  JsonObject data = doc.to<JsonObject>();
  
  data["gsm_ready"] = gsmReady;
  data["sms_ready"] = smsReady;
  data["sim_status"] = simStatus;
  data["network_operator"] = networkOperator;
  data["signal_strength_dbm"] = signalStrength;
  data["signal_bars"] = getSignalBars(signalStrength);
  data["emergency_contacts"] = emergencyCount;
  data["auto_forward_sms"] = autoForwardSMS;
  data["uptime"] = millis();
  data["wifi_rssi"] = WiFi.RSSI();
  
  return data;
}

void sendBirth() {
  StaticJsonDocument<512> doc;
  doc["v"] = 1;
  doc["t"] = millis();
  doc["c"] = "gsm";
  doc["e"] = "birth";
  doc["i"] = msgCounter++;
  
  JsonObject data = doc.createNestedObject("d");
  data["hb"] = 30; // Heartbeat interval in seconds
  data["fw"] = "1.0.0";
  data["module"] = "SIM800L";
  
  JsonArray caps = data.createNestedArray("caps");
  JsonObject cap = caps.createNestedObject();
  cap["c"] = "gsm";
  cap["cmds"] = "sms,call,status,config,signal";
  cap["events"] = "sms_sent,sms_received,call_status,signal_update,status,error";
  
  String message;
  serializeJson(doc, message);
  client.publish(birth_topic.c_str(), message.c_str());
  
  Serial.println("Birth sent: " + message);
}

void sendEvent(String event, JsonObject eventData) {
  StaticJsonDocument<512> doc;
  doc["v"] = 1;
  doc["t"] = millis();
  doc["c"] = "gsm";
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
  doc["c"] = "gsm";
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
  
  // Check for incoming SMS
  checkIncomingSMS();
  
  // Check signal strength periodically
  if (millis() - lastSignalCheck > signalCheckInterval) {
    checkSignalStrength();
    checkGSMStatus();
    lastSignalCheck = millis();
  }
  
  // Send heartbeat
  if (millis() - lastHeartbeat > HEARTBEAT_INTERVAL) {
    sendHeartbeat();
    lastHeartbeat = millis();
  }
  
  delay(1000);
}
