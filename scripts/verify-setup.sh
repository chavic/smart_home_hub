#!/bin/bash

echo "🔍 Verifying Smart Home Hub setup..."

# Check if .env exists
if [ ! -f ".env" ]; then
    echo "❌ .env file missing"
    exit 1
fi
echo "✅ .env file exists"

# Check if certificates exist
if [ ! -f "certs/ca.pem" ] || [ ! -f "certs/hub.key" ] || [ ! -f "certs/hub.crt" ]; then
    echo "❌ TLS certificates missing"
    exit 1
fi
echo "✅ TLS certificates exist"

# Check if database exists
if [ ! -f "prisma/data.db" ]; then
    echo "❌ Database not initialized"
    exit 1
fi
echo "✅ Database initialized"

# Check if node_modules exists
if [ ! -d "node_modules" ]; then
    echo "❌ Dependencies not installed"
    exit 1
fi
echo "✅ Dependencies installed"

# Try to build the project
echo "🔨 Testing build..."
npm run build > /dev/null 2>&1
if [ $? -ne 0 ]; then
    echo "❌ Build failed"
    exit 1
fi
echo "✅ Build successful"

# Test API (start server briefly)
echo "🌐 Testing API..."
npm run start &
SERVER_PID=$!
sleep 3

# Test the endpoint
RESPONSE=$(curl -s http://localhost:8080/devices)
kill $SERVER_PID 2>/dev/null

if [ "$RESPONSE" = "[]" ]; then
    echo "✅ API working correctly"
else
    echo "❌ API not responding correctly"
    exit 1
fi

echo ""
echo "🎉 All checks passed! Your Smart Home Hub is ready to run."
echo ""
echo "Start the development server with:"
echo "  npm run dev"
