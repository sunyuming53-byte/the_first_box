# =============================================================================
# RealMan Robot Arm — Multi-stage Docker build
# =============================================================================
#
# Stages:
#   realman-base-dev   ← osrf/ros:humble-desktop + OpenCV + realsense2 + SDK
#   realman-base       ← ros:humble              + OpenCV + realsense2 + SDK
#   realman-develop    ← base-dev + build tools + dev user
#   realman-runtime    ← base     + supervisor + entrypoint
#
# Build:
#   docker build . --target realman-develop -t realman:develop
#   docker build . --target realman-runtime  -t realman:runtime
#
# Run dev:
#   docker run -it --network host --device /dev \
#       -v $(pwd)/src:/ws/src -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY \
#       realman:develop
#
# Run runtime:
#   docker run -d --restart=always --network host --device /dev \
#       -v $(pwd)/install:/ws/install \
#       realman:runtime

# =============================================================================
# Stage 1a — Dev base (desktop: GUI tools, RViz, display support)
# =============================================================================
FROM osrf/ros:humble-desktop AS realman-base-dev

ENV DEBIAN_FRONTEND=noninteractive

# Use Aliyun mirror for faster apt downloads
RUN sed -i 's@archive.ubuntu.com@mirrors.aliyun.com@g' /etc/apt/sources.list && \
    sed -i 's@security.ubuntu.com@mirrors.aliyun.com@g' /etc/apt/sources.list

# Intel RealSense SDK 2.0 repository (for librealsense2-dev)
RUN apt-get update && apt-get install -y --no-install-recommends gnupg2 && \
    echo "deb [trusted=yes] https://librealsense.intel.com/Debian/apt-repo jammy main" > /etc/apt/sources.list.d/librealsense.list && \
    rm -rf /var/lib/apt/lists/*

# System libraries (dev variants — headers + .so)
RUN apt-get update && apt-get install -y --no-install-recommends \
    libopencv-dev \
    librealsense2-dev \
    && rm -rf /var/lib/apt/lists/*

# RealMan SDK
COPY cmake/RealManSDKConfig.cmake   /opt/realman-sdk/
COPY third_party/RM_API2/C/include  /opt/realman-sdk/include/
COPY third_party/RM_API2/C/linux/linux_x86_c_vv1.1.5/libapi_c.so /opt/realman-sdk/lib/libapi_c.so

# rosdep — install ROS2 deps declared in package.xml without embedding source
RUN --mount=type=bind,source=src,target=/tmp/src,readonly \
    apt-get update && \
    rosdep update && \
    rosdep install --from-paths /tmp/src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# Make SDK discoverable by CMake find_package(RealManSDK)
ENV REALMAN_SDK=/opt/realman-sdk

# =============================================================================
# Stage 1b — Runtime base (ros-base: no GUI, smaller image)
# =============================================================================
FROM ros:humble AS realman-base

ENV DEBIAN_FRONTEND=noninteractive

# Use Aliyun mirror for faster apt downloads
RUN sed -i 's@archive.ubuntu.com@mirrors.aliyun.com@g' /etc/apt/sources.list && \
    sed -i 's@security.ubuntu.com@mirrors.aliyun.com@g' /etc/apt/sources.list

# Intel RealSense SDK 2.0 repository (for librealsense2-dev)
RUN apt-get update && apt-get install -y --no-install-recommends gnupg2 && \
    echo "deb [trusted=yes] https://librealsense.intel.com/Debian/apt-repo jammy main" > /etc/apt/sources.list.d/librealsense.list && \
    rm -rf /var/lib/apt/lists/*

# Runtime .so only — no headers, no cmake configs
RUN apt-get update && apt-get install -y --no-install-recommends \
    libopencv-dev \
    librealsense2-dev \
    && rm -rf /var/lib/apt/lists/*

COPY cmake/RealManSDKConfig.cmake   /opt/realman-sdk/
COPY third_party/RM_API2/C/include  /opt/realman-sdk/include/
COPY third_party/RM_API2/C/linux/linux_x86_c_vv1.1.5/libapi_c.so /opt/realman-sdk/lib/libapi_c.so

RUN --mount=type=bind,source=src,target=/tmp/src,readonly \
    apt-get update && \
    rosdep update && \
    rosdep install --from-paths /tmp/src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

ENV REALMAN_SDK=/opt/realman-sdk

# =============================================================================
# Stage 2 — Develop
# =============================================================================
FROM realman-base-dev AS realman-develop

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    gdb \
    && rm -rf /var/lib/apt/lists/*

# Non-root user matching typical host UID
RUN useradd -m -u 1000 -s /bin/bash ubuntu && \
    mkdir -p /ws && chown ubuntu:ubuntu /ws

# SSH key — generated at build time for remote deployment to runtime container
RUN mkdir -p /home/ubuntu/.ssh && \
    ssh-keygen -t ed25519 -f /home/ubuntu/.ssh/id_rsa -N '' -C "realman-dev" && \
    chown -R ubuntu:ubuntu /home/ubuntu/.ssh

COPY scripts/entrypoint-dev.sh /entrypoint-dev.sh
RUN chmod +x /entrypoint-dev.sh

USER ubuntu
WORKDIR /ws

ENTRYPOINT ["/entrypoint-dev.sh"]

# =============================================================================
# Stage 3 — Runtime
# =============================================================================
FROM realman-base AS realman-runtime

# supervisor + sshd + rsync for remote deployment
RUN apt-get update && apt-get install -y --no-install-recommends \
    supervisor openssh-server rsync \
    && mkdir -p /var/run/sshd \
    && echo 'Port 2022'                       >> /etc/ssh/sshd_config \
    && echo 'PermitRootLogin prohibit-password' >> /etc/ssh/sshd_config \
    && echo 'PasswordAuthentication no'        >> /etc/ssh/sshd_config \
    && rm -rf /var/lib/apt/lists/*

# Authorize dev container's public key for passwordless SSH
RUN mkdir -p /root/.ssh && chmod 700 /root/.ssh
COPY --from=realman-develop --chown=root:root \
    /home/ubuntu/.ssh/id_rsa.pub /root/.ssh/authorized_keys
RUN chmod 600 /root/.ssh/authorized_keys

COPY scripts/entrypoint-runtime.sh /entrypoint-runtime.sh
COPY scripts/supervisord.conf    /etc/supervisor/conf.d/realman.conf
RUN chmod +x /entrypoint-runtime.sh

# Built artifacts are expected at /ws/install (mounted or synced at deploy time)
RUN mkdir -p /ws/install

CMD ["/entrypoint-runtime.sh"]
