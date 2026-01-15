#!/bin/bash
FILE="docker/Dockerfile"

# 1. Check if the file exists
if [ ! -f "$FILE" ]; then
    echo "❌ Error: $FILE not found!"
    exit 1
fi

# 2. Inject --fix-missing into the apt-get install line
# We look for 'apt-get install -y' and ensure '--fix-missing' follows it
if grep -q "apt-get install -y" "$FILE" && ! grep -q "--fix-missing" "$FILE"; then
    echo "🔧 Adding --fix-missing to Dockerfile for better download stability..."
    sed -i 's/apt-get install -y/apt-get install -y --fix-missing/' "$FILE"
else
    echo "✅ --fix-missing is already present or install line not found."
fi

# 3. Trigger the build with --no-cache
# This forces Docker to ignore the previous failure and try the download again
echo "🚀 Starting rebuild... this will bypass the broken download cache."
docker-compose -f docker/docker-compose.yml build --no-cache basekit

# 4. Final check
if [ $? -eq 0 ]; then
    echo "✨ Build successful! You can now run the robot."
else
    echo "⚠️ Build still failing. The ROS mirrors might be undergoing maintenance."
fi
