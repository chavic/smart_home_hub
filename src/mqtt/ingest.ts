import { mqttClient } from "./client.js";
import { envSchema, Envelope } from "../util/schema.js";
import { registry } from "../registry/registry.js";
import { prisma }   from "../db/client.js";

const EVENT_TOPIC = "$share/hub/iot/+/event";
const BIRTH_TOPIC = "$share/hub/iot/+/birth";
const ACK_TOPIC   = "$share/hub/iot/+/ack";

mqttClient.subscribe([EVENT_TOPIC, BIRTH_TOPIC, ACK_TOPIC], { qos: 1 });

mqttClient.on("message", async (topic, payload) => {
  let env: Envelope;
  try {
    env = envSchema.parse(JSON.parse(payload.toString("utf8")));
  } catch (err) {
    console.warn("invalid payload", err);
    return;
  }

  const [/*root*/, devId/*, ...*/] = topic.split("/");
  if (topic.endsWith("/birth")) {
    registry.upsertDevice(devId, env);
    return;
  }
  if (topic.endsWith("/event")) {
    registry.bumpHeartbeat(devId);
    await prisma.event.upsert({
      where: { devId_i: { devId, i: env.i } },
      update: {},
      create: {
        devId,
        i:       env.i,
        tsMs:    env.t,
        cap:     env.c,
        evt:     env.e,
        payload: JSON.stringify(env.d),
      },
    });
    return;
  }
  if (topic.endsWith("/ack")) {
    registry.handleAck(devId, env);
  }
});
