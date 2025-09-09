# Smart Home Hub

A TypeScript-first Smart Home Hub that communicates using JSON/MQTT protocol. Built with Node.js 18 LTS and modern ES modules.

## Features

- **MQTT Integration**: Secure MQTT 5.0 client with TLS support
- **Device Registry**: In-memory device management with heartbeat monitoring
- **Event Storage**: SQLite database with Prisma ORM for event persistence
- **REST API**: Express-based HTTP API for device control and monitoring
- **Type Safety**: Full TypeScript support with Zod schema validation

## Project Structure

```text
hub/
├─ package.json        # scripts + deps
├─ tsconfig.json       # ESM, target ES2022
├─ prisma/
│  └─ schema.prisma    # Database schema
├─ src/
│  ├─ index.ts         # process launcher
│  ├─ mqtt/
│  │   ├─ client.ts    # singleton connection
│  │   ├─ ingest.ts    # message consumer
│  │   └─ dispatch.ts  # command publisher
│  ├─ registry/
│  │   └─ registry.ts  # in-memory device map + HB timers
│  ├─ db/
│  │   └─ client.ts    # Prisma client
│  ├─ api/
│  │   └─ http.ts      # Express entry
│  └─ util/
│      └─ schema.ts    # Zod envelope validator
└─ .env.example        # Environment configuration template
```

## Quick Start

1. **Install dependencies:**
   ```bash
   npm install
   ```

2. **Run setup script (creates .env, certificates, and database):**
   ```bash
   npm run setup
   ```

3. **Start development server:**
   ```bash
   npm run dev
   ```

The server will start on http://localhost:8080 and attempt to connect to MQTT broker on localhost:8883.

## Manual Setup

If you prefer manual setup:

1. **Set up environment:**
   ```bash
   cp .env.example .env
   # Edit .env with your MQTT broker settings and TLS certificate paths
   ```

2. **Generate development certificates:**
   ```bash
   cd certs
   # Follow the certificate generation commands in scripts/dev-setup.js
   ```

3. **Initialize database:**
   ```bash
   npm run generate
   npm run migrate
   ```

## Environment Variables

Copy `.env.example` to `.env` and configure:

- `BROKER_HOST`: MQTT broker hostname
- `BROKER_PORT`: MQTT broker port (default: 8883)
- `BROKER_USER`: MQTT username
- `BROKER_PASS`: MQTT password
- `TLS_CA`: Path to CA certificate
- `TLS_KEY`: Path to client private key
- `TLS_CERT`: Path to client certificate

## API Endpoints

### GET /devices

List all registered devices with their status and capabilities.

### POST /devices/:id/cmd

Send a command to a specific device.

**Request body:**

```json
{
  "cap": "swt",
  "cmd": "on",
  "data": {}
}
```

## MQTT Topics

The hub subscribes to:

- `$share/hub/iot/+/event` - Device events
- `$share/hub/iot/+/birth` - Device registration
- `$share/hub/iot/+/ack` - Command acknowledgments

The hub publishes to:

- `iot/{deviceId}/cmd` - Device commands

## Message Format

All messages use a standardized JSON envelope:

```json
{
  "v": 1,           // Version
  "t": 1234567890,  // Timestamp
  "c": "swt",       // Capability (3 chars)
  "e": "on",        // Event/Command
  "i": 12345,       // Message ID
  "d": {}           // Data payload
}
```

## Development

- `npm run dev` - Start development server with hot reload
- `npm run build` - Build TypeScript to JavaScript
- `npm run start` - Start production server
- `npm run migrate` - Run database migrations
- `npm run generate` - Generate Prisma client

## Testing

You can test the MQTT integration using `mosquitto_pub`:

```bash
mosquitto_pub -h broker.local -p 8883 \
  --cafile certs/ca.pem \
  --cert certs/client.crt \
  --key certs/client.key \
  -t "iot/test-device/birth" \
  -m '{"v":1,"t":1234567890,"c":"swt","e":"birth","i":1,"d":{"hb":30,"caps":[{"c":"swt"}],"fw":"1.0.0"}}'
```
