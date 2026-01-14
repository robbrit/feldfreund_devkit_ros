import os, subprocess, sys, shutil

# --- CONFIGURATION ---
TARGET_DIR = os.path.expanduser("~/open_agbot_ws")
DOCKERFILE_PATH = os.path.join(TARGET_DIR, "docker/Dockerfile")
MAIN_REPO_URL = "https://github.com/Agroecology-Lab/Open_agbot_devkit_ros.git"
GNSS_REPO_URL = "https://github.com/Lemvos/automatepro_gnss_driver"

def run_cmd(cmd, description):
    print(f"\n[🚀] {description}...")
    process = subprocess.Popen(cmd, shell=True)
    process.communicate()
    if process.returncode != 0:
        print(f"\n[❌] FAILED: {description}")
        sys.exit(1)

def main():
    print("=== Open-AgBot Deployment (Final Stability Mode) ===")

    # 1. Setup Workspace & Main Repo
    if not os.path.exists(TARGET_DIR):
        run_cmd(f"git clone {MAIN_REPO_URL} {TARGET_DIR}", "Cloning Main Repository")
    os.chdir(TARGET_DIR)

    # 2. Ensure GNSS Driver is present
    gnss_path = os.path.join(TARGET_DIR, "automatepro_gnss_driver")
    if not os.path.exists(gnss_path):
        run_cmd(f"git clone {GNSS_REPO_URL} {gnss_path}", "Cloning Lemvos GNSS Driver")
    
    os.makedirs("docker", exist_ok=True)

    # 3. Check for requirement files (to avoid COPY errors)
    req_copy = ""
    req_install = ""
    if os.path.exists(os.path.join(TARGET_DIR, "requirements.txt")):
        req_copy += "COPY requirements.txt /root/\n"
        req_install += "RUN pip install -r /root/requirements.txt || true\n"
    if os.path.exists(os.path.join(TARGET_DIR, "requirements-dev.txt")):
        req_copy += "COPY requirements-dev.txt /root/\n"
        req_install += "RUN pip install -r /root/requirements-dev.txt || true\n"

    # 4. Write the Dockerfile
    with open(DOCKERFILE_PATH, "w") as f:
        f.write(r"""FROM ros:humble

# Pre-requisites & Lizard firmware
RUN apt-get update && apt-get install -y wget unzip curl
RUN mkdir -p /root/.lizard && \
    cd /root && \
    LATEST_VERSION=$(curl -s https://api.github.com/repos/zauberzeug/lizard/releases/latest | grep -oP '"tag_name": "\K(.*)(?=")') && \
    wget https://github.com/zauberzeug/lizard/releases/download/${LATEST_VERSION}/lizard_firmware_and_devtools_${LATEST_VERSION}_esp32.zip && \
    unzip lizard_firmware_and_devtools_${LATEST_VERSION}_esp32.zip -d /root/.lizard && \
    rm -f lizard_firmware_and_devtools_${LATEST_VERSION}_esp32.zip

# Repository Fixes (HTTPS & Clean sources)
RUN sed -i 's|http://archive.ubuntu.com/ubuntu/|https://archive.ubuntu.com/ubuntu/|g' /etc/apt/sources.list
RUN apt-get update && apt-get install -y --no-install-recommends ca-certificates gnupg2 lsb-release && update-ca-certificates
RUN rm -f /etc/apt/sources.list.d/ros2.sources
RUN sh -c 'echo "deb [arch=$(dpkg --print-architecture)] https://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list'

# Install ROS 2 Stack
RUN apt-get update && apt-get install -y libvdpau1 || apt-get -f install -y
RUN apt-get install -y --no-install-recommends \
    -o Acquire::https::Verify-Peer=false \
    ros-humble-domain-bridge \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-foxglove-bridge \
    ros-humble-xacro \
    ros-humble-rosbag2-storage-mcap \
    ros-humble-foxglove-msgs \
    ros-humble-tf-transformations \
    ros-humble-septentrio-gnss-driver \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-diagnostic-updater \
    ros-humble-usb-cam \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-image-tools \
    ros-humble-compressed-image-transport \
    ros-humble-axis-camera \
    python3-pip python3-serial python3-requests python3-yaml git nano

# Conditionally add requirements
""" + req_copy + req_install + r"""

# Copy local source code
COPY ./basekit_driver /workspace/src/basekit_driver
COPY ./basekit_launch /workspace/src/basekit_launch
COPY ./basekit_ui /workspace/src/basekit_ui
COPY ./automatepro_gnss_driver /workspace/src/automatepro_gnss_driver

# Build ROS workspace
WORKDIR /workspace
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install
""")

    # 5. Create docker-compose.yml
    with open("docker-compose.yml", "w") as f:
        f.write("""
services:
  open-agbot:
    build:
      context: .
      dockerfile: docker/Dockerfile
    container_name: open-agbot-main
    privileged: true
    network_mode: host
    restart: always
""")

    # 6. Build and Launch
    run_cmd("docker compose build", "Executing Cleaned Build")
    run_cmd("docker compose up -d", "Starting AgBot")

    print("\n" + "="*40)
    print("✨ SUCCESS: AG-BOT IS BUILT AND RUNNING")
    print("="*40)

if __name__ == "__main__":
    main()
