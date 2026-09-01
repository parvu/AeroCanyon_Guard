# ROS2 Jazzy + Gazebo Harmonic + PX4-Autopilot SITL (with this project's
# tricopter airframe) + Micro-XRCE-DDS-Agent, ready to build and run
# AeroCanyon-Guard. Mirrors the setup steps in README.md exactly -- see it
# for what each stage below is doing and why.
#
# Build:
#   docker build -t aerocanyon .
#
# Run (GUI trials need a working X11 display on the host; on Linux):
#   xhost +local:root
#   docker run -it --rm \
#     --net=host \
#     -e DISPLAY=$DISPLAY \
#     -v /tmp/.X11-unix:/tmp/.X11-unix \
#     aerocanyon
#
# --net=host is what makes the XRCE-DDS agent's UDP:8888, PX4's MAVLink
# ports, and QGroundControl's autoconnect all "just work" without per-port
# mapping. On WSL2/macOS, X11 needs an external server (VcXsrv/X410) and
# DISPLAY pointed at it -- see README.md's note on this. Without a display,
# everything except actually launching `gz sim` (and thus run_trial.py,
# which never launches headless -- see run_trial.py) still works: building,
# unit tests, training the FO-PINN offline, etc.
#
# No GPU passthrough configured here; gz-sim falls back to software
# rendering (llvmpipe) which works but is slow. For host GPU acceleration,
# add the appropriate --gpus/--device flags and NVIDIA/Mesa runtime to the
# `docker run` invocation.

FROM ubuntu:24.04

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ARG ROS_DISTRO=jazzy
# PX4-Autopilot commit the vendored px4_msgs submodule (pinned in
# .gitmodules) was generated against -- keep these in lockstep, otherwise
# the uORB<->ROS2 message bridge can silently mismatch fields.
ARG PX4_REF=1772e24b5f9ef8ae1ec74c5fb451529b8746e10d
ARG PX4_MSGS_REF=52f10548124b94b64f19860388c06179f8d29fcf
ARG UXRCE_REF=v2.4.3
ARG USERNAME=ros

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=${ROS_DISTRO} \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8

# ---------------------------------------------------------------------------
# Base tooling + locale (ROS2 requires a UTF-8 locale)
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        locales curl gnupg2 lsb-release software-properties-common \
        ca-certificates git build-essential cmake ninja-build \
        python3-pip python3-venv python3-dev sudo wget unzip \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# ROS2 Jazzy + Gazebo Harmonic (ros-gz pulls the gz-harmonic binaries
# straight from the ROS apt repo -- no separate OSRF Gazebo repo needed)
# ---------------------------------------------------------------------------
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
        > /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-desktop \
        ros-${ROS_DISTRO}-ros-gz \
        ros-${ROS_DISTRO}-ros-gz-bridge \
        ros-${ROS_DISTRO}-ros-gz-sim \
        ros-${ROS_DISTRO}-ros-gz-interfaces \
        ros-${ROS_DISTRO}-gz-msgs10 \
        ros-${ROS_DISTRO}-gz-transport13 \
        ros-${ROS_DISTRO}-rosidl-default-generators \
        ros-${ROS_DISTRO}-rosidl-default-runtime \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
    && rosdep init \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# Non-root build/run user (PX4's own setup script expects one, and Gazebo
# GUI + X11 forwarding is friendlier without root)
# ---------------------------------------------------------------------------
RUN useradd -m -s /bin/bash -G sudo,dialout,plugdev ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

USER ${USERNAME}
RUN rosdep update
WORKDIR /home/${USERNAME}

# ---------------------------------------------------------------------------
# PX4-Autopilot, pinned to the commit px4_msgs was generated against.
# Shallow-fetch just that one commit rather than a full clone/checkout.
# ---------------------------------------------------------------------------
RUN mkdir PX4-Autopilot && cd PX4-Autopilot \
    && git init -q \
    && git remote add origin https://github.com/PX4/PX4-Autopilot.git \
    && git fetch --depth 1 origin ${PX4_REF} \
    && git checkout -q FETCH_HEAD \
    && git submodule update --init --recursive --depth 1

# --no-nuttx: this project only builds the SITL target, never real hardware.
# --no-sim-tools: Gazebo Harmonic is already installed above via ros-gz;
# this skips PX4's own (redundant) OSRF Gazebo apt repo + install.
RUN cd PX4-Autopilot && bash ./Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools

# Build-only: `make px4_sitl_default` compiles the SITL binary without
# launching a simulator (unlike `make px4_sitl <model>`, which builds *and*
# runs -- that would hang here waiting on a display/Gazebo instance that
# doesn't exist during an image build).
RUN cd PX4-Autopilot && make px4_sitl_default

# ---------------------------------------------------------------------------
# Micro-XRCE-DDS-Agent (bridges ROS2 <-> PX4 over UDP)
# ---------------------------------------------------------------------------
RUN git clone --branch ${UXRCE_REF} --depth 1 \
        https://github.com/eProsima/Micro-XRCE-DDS-Agent.git \
    && cd Micro-XRCE-DDS-Agent \
    && mkdir build && cd build \
    && cmake .. \
    && make -j"$(nproc)" \
    && sudo make install \
    && sudo ldconfig /usr/local/lib/

# ---------------------------------------------------------------------------
# This workspace
# ---------------------------------------------------------------------------
COPY --chown=${USERNAME}:${USERNAME} . ros2_pinn_sim
WORKDIR /home/${USERNAME}/ros2_pinn_sim

# px4_msgs is a git submodule (.gitmodules); fetch it directly by pinned
# commit rather than relying on the build context carrying .git metadata.
RUN rm -rf src/px4_msgs && mkdir src/px4_msgs && cd src/px4_msgs \
    && git init -q \
    && git remote add origin https://github.com/PX4/px4_msgs.git \
    && git fetch --depth 1 origin ${PX4_MSGS_REF} \
    && git checkout -q FETCH_HEAD

# CPU-only torch by default to keep the image size sane; swap the
# --index-url for a CUDA build if the host has a GPU you want to use for
# training the FO-PINN.
RUN python3 -m venv --system-site-packages .venv \
    && source .venv/bin/activate \
    && pip install --upgrade pip \
    && pip install torch --index-url https://download.pytorch.org/whl/cpu \
    && pip install numpy scipy pandas matplotlib

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && source .venv/bin/activate \
    && colcon build --symlink-install

# ---------------------------------------------------------------------------
# Install this project's tricopter model/airframe/world into PX4-Autopilot
# (see README.md's "PX4 setup for the tricopter + wind" for what/why)
# ---------------------------------------------------------------------------
RUN cp -r src/aerocanyon/models/tricopter \
        ~/PX4-Autopilot/Tools/simulation/gz/models/ \
    && cp src/aerocanyon/airframes/4022_gz_tricopter \
        ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/ \
    && chmod 0755 ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4022_gz_tricopter \
    && cp src/aerocanyon/worlds/urban_canyon.sdf \
        ~/PX4-Autopilot/Tools/simulation/gz/worlds/urban_canyon.sdf

# Register the airframe in PX4's ROMFS build (idempotent: fails loudly if
# the stock 4021 entry it anchors on has moved/vanished, rather than
# silently skipping registration -- see History.md on silent failures in
# this stack).
RUN set -e; \
    cmake_file=~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt; \
    if grep -q '4022_gz_tricopter' "$cmake_file"; then \
        echo "4022_gz_tricopter already registered"; \
    elif grep -q '4021_gz_x500_flow' "$cmake_file"; then \
        awk '{print} /4021_gz_x500_flow/{print "\t4022_gz_tricopter"}' \
            "$cmake_file" > "$cmake_file.tmp" \
        && mv "$cmake_file.tmp" "$cmake_file"; \
    else \
        echo "ERROR: anchor entry 4021_gz_x500_flow not found in $cmake_file" \
             "-- PX4's airframes CMakeLists.txt layout changed upstream;" \
             "update the Dockerfile's insertion point." >&2; \
        exit 1; \
    fi

# Rebuild so the new ROMFS entry above is baked into the firmware image
# PX4 boots from (the model and world files copied above are plain runtime
# resources -- gz-sim reads them directly, no rebuild needed for those).
RUN cd ~/PX4-Autopilot && make px4_sitl_default

COPY --chown=${USERNAME}:${USERNAME} docker/entrypoint.sh /home/${USERNAME}/entrypoint.sh
RUN chmod +x /home/${USERNAME}/entrypoint.sh

# ENTRYPOINT's exec form can't expand ${USERNAME}; hardcoded to match the
# USERNAME default above. Update this path too if you override that arg.
ENTRYPOINT ["/home/ros/entrypoint.sh"]
CMD ["bash"]
