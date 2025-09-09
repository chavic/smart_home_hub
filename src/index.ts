import { config } from "dotenv";
import "./mqtt/client.js";
import "./mqtt/ingest.js";
import "./api/http.js";

config();

console.log("Smart Home Hub starting...");
console.log("MQTT client initializing...");
console.log("HTTP API starting on port 8080...");
