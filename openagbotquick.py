import os
import subprocess

# --- CONFIGURATION ---
# This creates a workspace folder in your home directory
TARGET_DIR = os.path.expanduser("~/open_agbot_ws")
SRC_DIR = os.path.join(TARGET_DIR, "src")
DOCKER_DIR = os.path.join(TARGET_DIR, "docker")
IMAGE_NAME = "basekit_ros:latest"

# Repositories required for the AgBot stack
REPOS = {
    "Open_agbot_devkit_ros": "https://github.com/Agroecology-Lab/Open_agbot_devkit_ros.git",
    "ublox": "https://github.com/KumarRobotics/ublox.git"
}

def run_cmd(cmd, cwd=None):
    print(f"\n>> Running: {cmd}")
    # We use a long timeout for the build process
    return subprocess.run(cmd, shell=True, cwd=cwd).returncode == 0

# 1. Setup Directory Structure
os.makedirs(SRC_DIR, exist_ok=True)
os.makedirs(DOCKER_DIR, exist_ok=True)

# 2. Smart Sync: Clone if missing, Pull if exists
for folder, url in REPOS.items():
    path = os.path.join(SRC_DIR, folder)
    if not os.path.exists(path):
        print(f"[📥] Cloning {folder}...")
        # ublox requires the ros2 branch
        branch_flag = "-b ros2" if folder == "ublox" else ""
        run_cmd(f"git clone {branch_flag} {url} {folder}", cwd=SRC_DIR)
    else:
        print(f"[🔄] {folder} already exists. Updating to latest code...")
        run_cmd("git pull", cwd=path)

# 3. Write the Hardened Dockerfile
dockerfile_path = os.path.join(DOCKER_DIR, "Dockerfile")
print(f"[📝] Generating Dockerfile at {dockerfile_path}...")

with open(dockerfile_path, "w") as f:
    f.write(r"""FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV VIRTUAL_ENV=/opt/agbot_venv
ENV PATH="$VIRTUAL_ENV/bin:$PATH"
ENV PYTHONPATH="/root/.lizard"
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Network Resilience
RUN echo 'Acquire::http::Pipeline-Depth "0"; Acquire::Retries "100";' > /etc/apt/apt.conf.d/99-resilience

# 1. System Dependencies (Includes Fix for Mirror Sync errors)
RUN for i in {1..10}; do \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    curl wget ca-certificates \
    python3-pip python3-venv libasio-dev unzip git nano \
    ros-humble-usb-cam ros-humble-xacro ros-humble-nmea-msgs \
    ros-humble-rtcm-msgs ros-humble-diagnostic-updater \
    ros-humble-twist-mux ros-humble-rmw-cyclonedds-cpp \
    ros-humble-domain-bridge ros-humble-tf-transformations \
    ros-humble-image-transport ros-humble-compressed-image-transport && \
    break || (echo "Apt failed, retrying in 10s..." && sleep 10); \
    done && rm -rf /var/lib/apt/lists/*

# 2. Lizard Espresso Setup
RUN mkdir -p /root/.lizard && cd /root && \
    LATEST=$(curl -s https://api.github.com/repos/zauberzeug/lizard/releases/latest | grep -oP '"tag_name": "\K(.*)(?=")') && \
    wget -q https://github.com/zauberzeug/lizard/releases/download/${LATEST}/lizard_firmware_and_devtools_${LATEST}_esp32.zip && \
    unzip -q *.zip -d /root/.lizard && rm *.zip && \
    chmod +x /root/.lizard/espresso.py

# 3. Python Virtual Env (Isolated from System ROS)
RUN python3 -m venv $VIRTUAL_ENV && \
    pip install --no-cache-dir --upgrade pip setuptools wheel && \
    pip install --no-cache-dir nicegui pyserial requests pyyaml && \
    if [ -f /root/.lizard/requirements.txt ]; then pip install -r /root/.lizard/requirements.txt; fi

# 4. Workspace Build
WORKDIR /workspace
COPY ./src ./src
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install --packages-select ublox_serialization && \
    . install/setup.sh && \
    colcon build --symlink-install --packages-select ublox_msgs && \
    . install/setup.sh && \
    colcon build --symlink-install --parallel-workers 1

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.sh && source $VIRTUAL_ENV/bin/activate && source install/setup.bash && ros2 launch basekit_launch basekit.launch.py"]
""")

# 4. Build the Docker Image
print(f"\n[🛠️] Building Docker Image: {IMAGE_NAME}...")
# We build from TARGET_DIR so the 'COPY ./src' command finds the files
if run_cmd(f"docker build -t {IMAGE_NAME} -f docker/Dockerfile .", cwd=TARGET_DIR):
    print("\n" + "="*50)
    print("✅ SUCCESS: Open AgBot Image is Built.")
    print(f"📍 Workspace: {TARGET_DIR}")
    print("🚀 Next step: docker compose up -d")
    print("="*50)
else:
    print("\n❌ ERROR: Docker build failed. Scroll up to see the logs.")
