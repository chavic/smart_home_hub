import mqtt from "mqtt";
import * as fs from "node:fs";
import { config } from "dotenv";
config();

export const CLIENT_ID = `hub-${Math.random().toString(16).slice(2)}`;

// Check if TLS files exist
const tlsFilesExist = () => {
  try {
    const caPath = process.env.TLS_CA!;
    const keyPath = process.env.TLS_KEY!;
    const certPath = process.env.TLS_CERT!;
    
    return fs.existsSync(caPath) && fs.existsSync(keyPath) && fs.existsSync(certPath);
  } catch {
    return false;
  }
};

const connectOptions: mqtt.IClientOptions = {
  clientId: CLIENT_ID,
  host: process.env.BROKER_HOST || "localhost",
  port: Number(process.env.BROKER_PORT ?? 8883),
  protocol: "mqtts",
  connectTimeout: 10_000,
  username: process.env.BROKER_USER,
  password: process.env.BROKER_PASS,
  protocolVersion: 5,
  rejectUnauthorized: false, // For development with self-signed certs
};

// Add TLS options if certificates exist
if (tlsFilesExist()) {
  connectOptions.ca = fs.readFileSync(process.env.TLS_CA!);
  connectOptions.key = fs.readFileSync(process.env.TLS_KEY!);
  connectOptions.cert = fs.readFileSync(process.env.TLS_CERT!);
  console.log("Using TLS certificates for MQTT connection");
} else {
  console.warn("TLS certificates not found, MQTT connection may fail");
}

export const mqttClient = mqtt.connect(connectOptions);

mqttClient.on("connect", () => {
  console.log("✅ MQTT connected successfully");
});

mqttClient.on("error", err => {
  console.error("❌ MQTT connection error:", err.message);
});

mqttClient.on("offline", () => {
  console.warn("⚠️  MQTT client offline");
});

mqttClient.on("reconnect", () => {
  console.log("🔄 MQTT reconnecting...");
});
