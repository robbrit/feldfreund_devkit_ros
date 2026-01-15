#!/bin/bash
FILE="docker/Dockerfile"

# 1. Remove the broken lines between section 5 and section 6
# This targets the lines between the "Core Python" and "Workspace Setup" comments
sed -i '/# 5. Core Python/,/# 6. Workspace/{//!d}' "$FILE"

# 2. Define the correct Golden Block
# Note the double backslashes to escape them for the shell
NEW_BLOCK='RUN pip3 install --no-cache-dir --upgrade pip wheel && \\
    pip3 install "setuptools<71.0.0" && \\
    pip3 install --no-cache-dir nicegui pyserial requests pyyaml'

# 3. Insert the Golden Block right after the Section 5 header
sed -i '/# 5. Core Python/a '"$NEW_BLOCK" "$FILE"

echo "✅ Dockerfile repaired with correct syntax."

# 4. Trigger the build
echo "🔨 Starting clean build of basekit..."
docker-compose -f docker/docker-compose.yml build --no-cache basekit
