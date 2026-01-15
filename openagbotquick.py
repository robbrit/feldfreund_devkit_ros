import os
import subprocess
import sys

# --- CONFIGURATION ---
TARGET_DIR = os.path.expanduser("~/open_agbot_ws")
SRC_DIR = os.path.join(TARGET_DIR, "src")
DOCKER_DIR = os.path.join(TARGET_DIR, "src/Open_agbot_devkit_ros/docker")

REPOS = {
    "Open_agbot_devkit_ros": "https://github.com/Agroecology-Lab/Open_agbot_devkit_ros.git",
    "ublox": "https://github.com/KumarRobotics/ublox.git"
}

def run_cmd(cmd, cwd=None):
    print(f"\n>> Executing: {cmd}")
    process = subprocess.Popen(cmd, shell=True, cwd=cwd, stdout=sys.stdout, stderr=sys.stderr)
    process.wait()
    return process.returncode == 0

# 1. Sync Repositories
os.makedirs(SRC_DIR, exist_ok=True)
for folder, url in REPOS.items():
    path = os.path.join(SRC_DIR, folder)
    if not os.path.exists(path):
        branch = "-b ros2" if "ublox" in url else ""
        run_cmd(f"git clone {branch} {url} {folder}", cwd=SRC_DIR)
    else:
        run_cmd("git pull", cwd=path)

# 2. Write Modernized Dockerfile
os.makedirs(DOCKER_DIR, exist_ok=True)
dockerfile_content = r"""FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV PYTHONPATH="/root/.lizard"

# STEP 1: Mirror fix and Tool installation
RUN sed -i 's/[a-z.]*\.ubuntu\.com/azure.archive.ubuntu.com/g' /etc/apt/sources.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends wget curl ca-certificates unzip git nano python3-pip software-properties-common && \
    add-apt-repository universe && \
    apt-get update

# STEP 2: Scraper logic - CORRECTED POOL PATHS
# Iceoryx packages are actually located in the 'i/iceoryx/' subfolder in the ROS pool
RUN mkdir -p /tmp/ros_fix && cd /tmp/ros_fix && \
    BASE_URL="http://packages.ros.org/ros2/ubuntu/pool/main" && \
    set -e; \
    for pkg_path in \
        "r/ros-humble-diagnostic-updater/" \
        "r/ros-humble-rtcm-msgs/" \
        "r/ros-humble-rmw-cyclonedds-cpp/" \
        "i/iceoryx/"; \
    do \
        echo "Searching ${BASE_URL}/${pkg_path}..."; \
        FILES=$(curl -s ${BASE_URL}/${pkg_path} | grep -oP '(?<=>)[^<]+?_amd64\.deb'); \
        for FILE in $FILES; do \
            echo "Fetching $FILE..."; \
            wget -q "${BASE_URL}/${pkg_path}${FILE}"; \
        done; \
    done && \
    apt-get install -y ./*.deb && cd / && rm -rf /tmp/ros_fix

# STEP 3: Remaining ROS/Hardware Dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions python3-serial libasio-dev udev \
    ros-humble-usb-cam ros-humble-xacro ros-humble-nmea-msgs \
    ros-humble-joint-state-publisher ros-humble-robot-state-publisher \
    ros-humble-controller-manager ros-humble-diff-drive-controller \
    ros-humble-twist-mux && \
    rm -rf /var/lib/apt/lists/*

# STEP 4: Lizard & Python (as before)
RUN mkdir -p /root/.lizard && cd /root && \
    LATEST=$(curl -s https://api.github.com/repos/zauberzeug/lizard/releases/latest | grep -oP '"tag_name": "\K(.*)(?=")') && \
    wget -q "https://github.com/zauberzeug/lizard/releases/download/${LATEST}/lizard_firmware_and_devtools_${LATEST}_esp32.zip" && \
    unzip -o *.zip -d /root/.lizard && rm *.zip && \
    pip3 install --no-cache-dir nicegui pyserial requests pyyaml

# STEP 5: Workspace Build
WORKDIR /workspace
COPY ./src ./src
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install --packages-select ublox_serialization && \
    . install/setup.sh && \
    colcon build --symlink-install --packages-select ublox_msgs && \
    . install/setup.sh && \
    colcon build --symlink-install --parallel-workers 1

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /workspace/install/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.sh && source install/setup.bash && ros2 launch basekit_launch basekit.launch.py"]
"""

with open(os.path.join(DOCKER_DIR, "Dockerfile"), "w") as f:
    f.write(dockerfile_content)

# 3. Write docker-compose.yml
print("📝 Writing docker-compose.yml...")
compose_content = """services:
  basekit:
    build:
      context: ../../../
      dockerfile: src/Open_agbot_devkit_ros/docker/Dockerfile
    image: agroecology/open_agbot:latest
    container_name: open_agbot_basekit
    privileged: true
    network_mode: host
    restart: unless-stopped
    volumes:
      - /dev:/dev
    environment:
      - RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
      - PYTHONUNBUFFERED=1
"""
with open(os.path.join(DOCKER_DIR, "docker-compose.yml"), "w") as f:
    f.write(compose_content)

# 4. Build
print("\n🛠️ Starting Docker Build...")
run_cmd(f"docker-compose -f {DOCKER_DIR}/docker-compose.yml up --build -d", cwd=TARGET_DIR)
