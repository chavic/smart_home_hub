import mqtt from "mqtt";
import * as fs from "node:fs";
import { config } from "dotenv";
config();

export const CLIENT_ID = `hub-${Math.random().toString(16).slice(2)}`;

export const mqttClient = mqtt.connect({
  clientId: CLIENT_ID,
  host: process.env.BROKER_HOST,     // e.g. broker.local
  port: Number(process.env.BROKER_PORT ?? 8883),
  protocol: "mqtts",
  connectTimeout: 10_000,
  username: process.env.BROKER_USER,
  password: process.env.BROKER_PASS,
  ca:  fs.readFileSync(process.env.TLS_CA!),
  key: fs.readFileSync(process.env.TLS_KEY!),
  cert:fs.readFileSync(process.env.TLS_CERT!),
  protocolVersion: 5,
});

mqttClient.on("connect", () => {
  console.log("MQTT connected");
});

mqttClient.on("error", err => {
  console.error("MQTT error:", err);
});
