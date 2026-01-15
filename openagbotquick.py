import os
import subprocess
import time
import getpass

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

# 1. Sync Repositories
os.makedirs(SRC_DIR, exist_ok=True)
os.makedirs(DOCKER_DIR, exist_ok=True)
for folder, url in REPOS.items():
    path = os.path.join(SRC_DIR, folder)
    if not os.path.exists(path):
        branch_flag = "-b ros2" if folder == "ublox" else ""
        run_cmd(f"git clone {branch_flag} {url} {folder}", cwd=SRC_DIR)
    else:
        run_cmd("git pull", cwd=path)

# 2. Write Unified Dockerfile (Fixed Syntax & Verbose)
dockerfile_path = os.path.join(DOCKER_DIR, "Dockerfile")
with open(dockerfile_path, "w") as f:
    f.write(r"""FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV PYTHONPATH="/root/.lizard"

# STEP 1: Setup stable mirrors and install tools
RUN sed -i 's/archive.ubuntu.com/azure.archive.ubuntu.com/g' /etc/apt/sources.list && \
    sed -i 's/security.ubuntu.com/azure.archive.ubuntu.com/g' /etc/apt/sources.list && \
    apt-get update && apt-get install -y --no-install-recommends wget curl ca-certificates unzip git nano python3-pip && \
    mkdir -p /tmp/ros_fix && cd /tmp/ros_fix && \
    # Automated 'Latest Version' Finder (Verbose)
    BASE_URL="http://packages.ros.org/ros2/ubuntu/pool/main" && \
    set -e; \
    for pkg_path in \
        "r/ros-humble-diagnostic-updater/" \
        "r/ros-humble-rtcm-msgs/" \
        "r/ros-humble-rmw-cyclonedds-cpp/" \
        "i/iceoryx-binding-c/"; \
    do \
        echo "Searching for latest .deb in ${BASE_URL}/${pkg_path}..."; \
        DEB_FILE=$(curl -s ${BASE_URL}/${pkg_path} | grep -oP 'ros-humble-[^"]+?_amd64\.deb|iceoryx-[^"]+?_amd64\.deb' | tail -n 1); \
        if [ -z "$DEB_FILE" ]; then echo "ERROR: Could not find .deb for ${pkg_path}"; exit 1; fi; \
        echo "Found: $DEB_FILE. Downloading..."; \
        wget --verbose "${BASE_URL}/${pkg_path}${DEB_FILE}"; \
    done && \
    apt-get install -y ./*.deb && \
    cd / && rm -rf /tmp/ros_fix && \
    # STEP 2: Install Remaining Dependencies
    for i in {1..5}; do apt-get update && break || sleep 5; done && \
    apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions python3-serial \
    libasio-dev udev \
    ros-humble-usb-cam ros-humble-xacro ros-humble-nmea-msgs \
    ros-humble-joint-state-publisher ros-humble-robot-state-publisher \
    ros-humble-controller-manager ros-humble-diff-drive-controller \
    ros-humble-twist-mux && \
    rm -rf /var/lib/apt/lists/*

# STEP 3: Lizard Espresso Tooling (Verbose)
RUN mkdir -p /root/.lizard && cd /root && \
    LATEST=$(curl -s https://api.github.com/repos/zauberzeug/lizard/releases/latest | grep -oP '"tag_name": "\K(.*)(?=")') && \
    echo "Latest Lizard found: $LATEST" && \
    wget --verbose "https://github.com/zauberzeug/lizard/releases/download/${LATEST}/lizard_firmware_and_devtools_${LATEST}_esp32.zip" && \
    unzip *.zip -d /root/.lizard && rm *.zip && \
    chmod +x /root/.lizard/espresso.py

# STEP 4: Python Deps
RUN pip3 install --no-cache-dir --upgrade pip setuptools wheel && \
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
""")

# 3. Build Command
print(f"\n[🛠️] Building Image...")
if run_cmd(f"docker build -t {IMAGE_NAME} -f docker/Dockerfile .", cwd=TARGET_DIR):
    print("\n" + "="*60)
    print("✅ SUCCESS: Image built.")
    print(f"Run: docker run -it --rm --privileged --net=host -v /dev:/dev {IMAGE_NAME}")
    print("="*60)
else:
    print("\n❌ Build failed. Check the verbose output above for the error.")
