#!/usr/bin/env node

/**
 * Development setup script
 * This script helps set up the development environment
 */

import { execSync } from 'child_process';
import fs from 'fs';
import path from 'path';

console.log('🚀 Setting up Smart Home Hub for development...\n');

// Check if .env exists
if (!fs.existsSync('.env')) {
  console.log('📝 Creating .env file from template...');
  execSync('cp .env.example .env');
}

// Check if certificates exist
const certsDir = 'certs';
const requiredCerts = ['ca.pem', 'hub.key', 'hub.crt'];
const missingCerts = requiredCerts.filter(cert => 
  !fs.existsSync(path.join(certsDir, cert))
);

if (missingCerts.length > 0) {
  console.log('🔐 Generating development TLS certificates...');
  try {
    execSync(`
      cd certs &&
      openssl genrsa -out ca.key 2048 &&
      openssl req -new -x509 -days 365 -key ca.key -out ca.pem -subj "/C=US/ST=Dev/L=Dev/O=SmartHomeHub/OU=Dev/CN=ca.local" &&
      openssl genrsa -out hub.key 2048 &&
      openssl req -new -key hub.key -out hub.csr -subj "/C=US/ST=Dev/L=Dev/O=SmartHomeHub/OU=Dev/CN=hub.local" &&
      openssl x509 -req -in hub.csr -CA ca.pem -CAkey ca.key -CAcreateserial -out hub.crt -days 365 &&
      rm hub.csr ca.key ca.srl
    `, { stdio: 'inherit' });
  } catch (error) {
    console.warn('⚠️  Could not generate certificates automatically. Please create them manually.');
  }
}

// Check database
if (!fs.existsSync('prisma/data.db')) {
  console.log('🗄️  Initializing database...');
  execSync('npx prisma migrate dev --name init', { stdio: 'inherit' });
}

console.log('\n✅ Development environment ready!');
console.log('\n📋 Next steps:');
console.log('   1. Start a local MQTT broker (optional):');
console.log('      docker run -it -p 1883:1883 -p 8883:8883 eclipse-mosquitto');
console.log('   2. Run the development server:');
console.log('      npm run dev');
console.log('   3. Test the API:');
console.log('      curl http://localhost:8080/devices');
