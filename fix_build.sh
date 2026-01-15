#!/bin/bash

# Define the fix line
FIX_LINE='RUN pip3 install "setuptools<71.0.0"'

# Check if the fix is already there to avoid duplicates
if grep -q "setuptools<71.0.0" docker/Dockerfile; then
    echo "✅ Fix already applied to Dockerfile."
else
    # Insert the fix before the first pip install line
    # This uses sed to find 'pip3 install' and insert our fix above it
    sed -i '/pip3 install/i '"$FIX_LINE" docker/Dockerfile
    echo "🚀 Fix applied! Downgraded setuptools in Dockerfile."
fi

# Run the build
echo "🔨 Starting build..."
python3 openagbotquick.py --clean
