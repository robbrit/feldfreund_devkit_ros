import os
import subprocess

# --- CONFIGURATION ---
TARGET_DIR = os.path.expanduser("~/open_agbot_ws")
SRC_DIR = os.path.join(TARGET_DIR, "src")
DOCKER_DIR = os.path.join(TARGET_DIR, "docker")
IMAGE_NAME = "basekit_ros:latest"

REPOS = {
    "Open_agbot_devkit_ros": "https://github.com/Agroecology-Lab/Open_agbot_devkit_ros.git",
    "ublox": "https://github.com/KumarRobotics/ublox.git"
}

def run_cmd(cmd, cwd=None):
    print(f"\n>> Running: {cmd}")
    return subprocess.run(cmd, shell=True, cwd=cwd).returncode == 0

os.makedirs(SRC_DIR, exist_ok=True)
os.makedirs(DOCKER_DIR, exist_ok=True)

for folder, url in REPOS.items():
    path = os.path.join(SRC_DIR, folder)
    if not os.path.exists(path):
        branch_flag = "-b ros2" if folder == "ublox" else ""
        run_cmd(f"git clone {branch_flag} {url} {folder}", cwd=SRC_DIR)
    else:
        run_cmd("git pull", cwd=path)

dockerfile_path = os.path.join(DOCKER_DIR, "Dockerfile")
with open(dockerfile_path, "w") as f:
    f.write(r"""FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV PYTHONPATH="/root/.lizard"

# 1. System Tools & ROS 2 Dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl wget ca-certificates unzip git nano \
    python3-pip python3-colcon-common-extensions \
    libasio-dev \
    ros-humble-usb-cam ros-humble-xacro ros-humble-nmea-msgs \
    ros-humble-rtcm-msgs ros-humble-diagnostic-updater \
    ros-humble-twist-mux ros-humble-rmw-cyclonedds-cpp && \
    rm -rf /var/lib/apt/lists/*

# 2. Lizard Espresso Setup
RUN mkdir -p /root/.lizard && cd /root && \
    LATEST=$(curl -s https://api.github.com/repos/zauberzeug/lizard/releases/latest | grep -oP '"tag_name": "\K(.*)(?=")') && \
    wget -q https://github.com/zauberzeug/lizard/releases/download/${LATEST}/lizard_firmware_and_devtools_${LATEST}_esp32.zip && \
    unzip -q *.zip -d /root/.lizard && rm *.zip && \
    chmod +x /root/.lizard/espresso.py

# 3. Direct Python Package Installation (No Venv)
# We use --break-system-packages because this is a dedicated container
RUN pip3 install --no-cache-dir --upgrade pip setuptools wheel && \
    pip3 install --no-cache-dir nicegui pyserial requests pyyaml

# 4. Workspace Build
WORKDIR /workspace
COPY ./src ./src

RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install --packages-select ublox_serialization && \
    . install/setup.sh && \
    colcon build --symlink-install --packages-select ublox_msgs && \
    . install/setup.sh && \
    colcon build --symlink-install --parallel-workers 1

# Auto-source ROS and Workspace for anyone logging in
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /workspace/install/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.sh && source install/setup.bash && ros2 launch basekit_launch basekit.launch.py"]
""")

if run_cmd(f"docker build -t {IMAGE_NAME} -f docker/Dockerfile .", cwd=TARGET_DIR):
    print("\n✅ Build Successful (Simplified No-Venv Version).")
else:
    print("\n❌ Build failed.")
