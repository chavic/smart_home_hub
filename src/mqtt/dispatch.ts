import { mqttClient } from "./client.js";
import { registry }   from "../registry/registry.js";
import { prisma }     from "../db/client.js";

export async function publishCmd(
  devId: string,
  cap: string,
  cmdEvt: string,
  data: Record<string, unknown> = {},
) {
  const msgId = Date.now() & 0xffff;          // simple monotonic id
  const env = { v:1, t:Date.now(), c:cap, e:cmdEvt, i:msgId, d:data };

  mqttClient.publish(
    `iot/${devId}/cmd`,
    JSON.stringify(env),
    { qos: 1 },
    err => {
      if (err) console.error("publish failed:", err);
    },
  );

  await prisma.command.create({
    data: { devId, i: msgId, tsSent: Date.now(), cmd: cmdEvt, status: "sent" },
  });
}
