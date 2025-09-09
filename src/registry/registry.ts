import { Envelope } from "../util/schema.js";

interface Device {
  devId:        string;
  hb:           number;
  lastSeen:     number;
  capabilities: string[];
  fw:           string;
}

class DeviceRegistry {
  private map = new Map<string, Device>();
  private watchdog: NodeJS.Timeout;

  constructor() {
    this.watchdog = setInterval(() => this.scan(), 10_000);
  }

  upsertDevice(devId: string, birth: Envelope) {
    const hb  = birth.d.hb ?? 30;
    const caps= (birth.d.caps as any[]).map(c => c.c);
    const fw  = birth.d.fw;
    this.map.set(devId, { devId, hb, lastSeen: Date.now(), capabilities: caps, fw });
  }

  bumpHeartbeat(devId: string) {
    const d = this.map.get(devId);
    if (d) d.lastSeen = Date.now();
  }

  handleAck(devId: string, ack: Envelope) {
    // mark command resolved: devId + ack.i
    console.log(`Command acknowledged for device ${devId}, message ID: ${ack.i}`);
  }

  list() { return Array.from(this.map.values()); }

  private scan() {
    const now = Date.now();
    for (const d of this.map.values()) {
      if (now - d.lastSeen > d.hb * 1_250) {
        console.warn(`${d.devId} offline`);
      }
    }
  }
}

export const registry = new DeviceRegistry();
