import express from "express";
import { config } from "dotenv";
import { registry } from "../registry/registry.js";
import { publishCmd } from "../mqtt/dispatch.js";

config();
const app = express();
app.use(express.json());

app.get("/devices", (_req, res) => {
  res.json(registry.list());
});

app.post("/devices/:id/cmd", async (req, res) => {
  const devId = req.params.id;
  const { cap, cmd, data } = req.body;
  await publishCmd(devId, cap, cmd, data);
  res.status(202).json({ queued: true });
});

app.listen(8080, () => console.log("HTTP API on :8080"));
