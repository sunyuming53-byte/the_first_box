# =============================================================================
# OMRobot — Multi-stage Docker build
# =============================================================================
#
# Stages:
#   omrobot-base-dev   ← osrf/ros:humble-desktop + OpenCV + realsense2
#   omrobot-base       ← ros:humble              + OpenCV + realsense2
#   omrobot-develop    ← base-dev + build tools + dev user
#   omrobot-runtime    ← base     + supervisor + entrypoint
#
# Build:
#   docker build . --target omrobot-develop -t omrobot:develop
#   docker build . --target omrobot-runtime  -t omrobot:runtime
#
# Run dev:
#   docker run -it --network host --device /dev \
#       -v $(pwd)/src:/ws/src -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY \
#       omrobot:develop
#
# Run runtime:
#   docker run -d --restart=always --network host --device /dev \
#       -v $(pwd)/install:/ws/install \
#       omrobot:runtime

# Build-time proxy args — set via docker compose --build-arg or compose build.args
ARG HTTP_PROXY
ARG HTTPS_PROXY

# =============================================================================
# Stage 1a — Dev base (desktop: GUI tools, RViz, display support)
# =============================================================================
FROM osrf/ros:humble-desktop AS omrobot-base-dev

ARG HTTP_PROXY
ARG HTTPS_PROXY
ENV HTTP_PROXY=${HTTP_PROXY} \
    HTTPS_PROXY=${HTTPS_PROXY} \
    http_proxy=${HTTP_PROXY} \
    https_proxy=${HTTPS_PROXY}

ENV DEBIAN_FRONTEND=noninteractive
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Use Aliyun mirror for faster apt downloads
RUN sed -i 's@archive.ubuntu.com@mirrors.aliyun.com@g' /etc/apt/sources.list && \
    sed -i 's@security.ubuntu.com@mirrors.aliyun.com@g' /etc/apt/sources.list

# Intel RealSense SDK 2.0 repository (for librealsense2-dev)
RUN apt-get update && apt-get install -y --no-install-recommends gnupg2 && \
    echo "deb [trusted=yes] https://librealsense.intel.com/Debian/apt-repo jammy main" > /etc/apt/sources.list.d/librealsense.list && \
    rm -rf /var/lib/apt/lists/*

# Use Tsinghua ROS2 mirror for faster downloads (geographic optimization)
# Tsinghua does not mirror source packages — drop deb-src to avoid 404
RUN find /etc/apt/sources.list.d \( -name "*.list" -o -name "*.sources" \) -exec sed -i \
    -e 's|http://packages.ros.org/ros2/ubuntu|https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu|g' \
    -e 's| deb-src||g' {} \;

# System libraries (dev variants — headers + .so)
RUN apt-get update && apt-get install -y --no-install-recommends \
    libeigen3-dev \
    libmodbus-dev \
    libomp-dev \
    libopencv-dev \
    libpcl-dev \
    librealsense2-dev \
    zsh curl git wget gnupg \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rclcpp \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-sim \
    ros-humble-std-srvs \
    ros-humble-tf-transformations \
    ros-humble-tf2-ros \
    ros-humble-geometry-msgs \
    ros-humble-gz-ros2-control \
    ros-humble-hardware-interface \
    ros-humble-pluginlib \
    ros-humble-controller-manager \
    ros-humble-joint-state-broadcaster \
    ros-humble-joint-trajectory-controller \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-sensor-msgs \
    ros-humble-std-msgs \
    ros-humble-visualization-msgs \
    ros-humble-ament-cmake-test \
    ros-humble-ament-index-cpp \
    ros-humble-behaviortree-cpp \
    ros-humble-control-msgs \
    ros-humble-moveit-ros-planning \
    ros-humble-moveit-planners-ompl \
    ros-humble-moveit-ros-move-group \
    ros-humble-moveit-simple-controller-manager \
    ros-humble-moveit-ros-visualization \
    ros-humble-moveit-setup-assistant \
    ros-humble-moveit-ros-planning-interface \
    ros-humble-nav2-bringup \
    ros-humble-nav2-common \
    ros-humble-navigation2 \
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    && rm -rf /var/lib/apt/lists/*

# clangd + clang-tidy + clang-format from LLVM apt repo (latest available for Jammy)
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked,id=apt-cache-dev --mount=type=cache,target=/var/lib/apt,sharing=locked,id=apt-lib-dev \
    wget -qO- https://apt.llvm.org/llvm-snapshot.gpg.key | tee /etc/apt/trusted.gpg.d/apt.llvm.org.asc && \
    echo "deb http://apt.llvm.org/jammy/ llvm-toolchain-jammy-19 main" > /etc/apt/sources.list.d/llvm.list && \
    apt-get update && apt-get install -y --no-install-recommends clangd-19 clang-tidy-19 clang-format-19 libomp-19-dev && \
    ln -sf /usr/bin/clangd-19 /usr/bin/clangd && \
    ln -sf /usr/bin/clang-tidy-19 /usr/bin/clang-tidy && \
    ln -sf /usr/bin/clang-format-19 /usr/bin/clang-format && \
    rm -rf /var/lib/apt/lists/*

# RealMan SDK — copy from submodule to /opt/realman-sdk
RUN mkdir -p /opt/realman-sdk/lib
COPY src/omr_hardware/third_party/realman_arm/third_party/RM_API2/C/include/ /opt/realman-sdk/include/
COPY src/omr_hardware/third_party/realman_arm/third_party/RM_API2/C/linux/linux_x86_c_vv1.1.5/libapi_c.so /opt/realman-sdk/lib/
ENV LD_LIBRARY_PATH=/opt/realman-sdk/lib

# oh-my-zsh + powerlevel10k + plugins (as root; copied to ubuntu user in develop stage)
RUN sh -c "$(curl -fsSL --retry 5 --retry-delay 10 https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" "" --unattended \
    && git clone --depth=1 https://github.com/romkatv/powerlevel10k.git \
        ${ZSH_CUSTOM:-/root/.oh-my-zsh/custom}/themes/powerlevel10k \
    && git clone --depth=1 https://github.com/zsh-users/zsh-syntax-highlighting.git \
        ${ZSH_CUSTOM:-/root/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting \
    && git clone --depth=1 https://github.com/zsh-users/zsh-autosuggestions.git \
        ${ZSH_CUSTOM:-/root/.oh-my-zsh/custom}/plugins/zsh-autosuggestions \
    && test -f /root/.oh-my-zsh/oh-my-zsh.sh \
    || (echo "ERROR: oh-my-zsh install failed (curl timeout?)" >&2 && false)

# =============================================================================
# Stage 1b — Runtime base (ros-base: no GUI, smaller image)
# =============================================================================
FROM ros:humble AS omrobot-base

ARG HTTP_PROXY
ARG HTTPS_PROXY
ENV HTTP_PROXY=${HTTP_PROXY} \
    HTTPS_PROXY=${HTTPS_PROXY} \
    http_proxy=${HTTP_PROXY} \
    https_proxy=${HTTPS_PROXY}

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
    libmodbus-dev \
    zsh curl git \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-ament-cmake-test \
    ros-humble-ament-index-cpp \
    ros-humble-behaviortree-cpp \
    ros-humble-control-msgs \
    ros-humble-moveit-ros-planning \
    ros-humble-moveit-planners-ompl \
    ros-humble-moveit-ros-move-group \
    ros-humble-moveit-simple-controller-manager \
    ros-humble-moveit-ros-visualization \
    ros-humble-moveit-setup-assistant \
    ros-humble-moveit-ros-planning-interface \
    ros-humble-nav2-bringup \
    ros-humble-nav2-common \
    ros-humble-navigation2 \
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    ros-humble-tf-transformations \
    && rm -rf /var/lib/apt/lists/*

# SDK runtime — libapi_c.so for arm control at runtime
RUN mkdir -p /opt/realman-sdk/lib
COPY src/omr_hardware/third_party/realman_arm/third_party/RM_API2/C/linux/linux_x86_c_vv1.1.5/libapi_c.so /opt/realman-sdk/lib/
ENV LD_LIBRARY_PATH=/opt/realman-sdk/lib

RUN --mount=type=bind,source=src,target=/tmp/src,readonly \
    apt-get update && \
    rosdep update && \
    rosdep install --from-paths /tmp/src --ignore-src -r -y --skip-keys realman_arm omr_hardware && \
    rm -rf /var/lib/apt/lists/*

# oh-my-zsh + powerlevel10k + plugins for root (runtime container)
RUN sh -c "$(curl -fsSL --retry 5 --retry-delay 10 https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" "" --unattended \
    && git clone --depth=1 https://github.com/romkatv/powerlevel10k.git \
        ${ZSH_CUSTOM:-/root/.oh-my-zsh/custom}/themes/powerlevel10k \
    && git clone --depth=1 https://github.com/zsh-users/zsh-syntax-highlighting.git \
        ${ZSH_CUSTOM:-/root/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting \
    && git clone --depth=1 https://github.com/zsh-users/zsh-autosuggestions.git \
        ${ZSH_CUSTOM:-/root/.oh-my-zsh/custom}/plugins/zsh-autosuggestions \
    && test -f /root/.oh-my-zsh/oh-my-zsh.sh \
    || (echo "ERROR: oh-my-zsh install failed (curl timeout?)" >&2 && false)

# =============================================================================
# Stage 2 — Develop
# =============================================================================
FROM omrobot-base-dev AS omrobot-develop

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked,id=apt-cache-dev \
    --mount=type=cache,target=/var/lib/apt,sharing=locked,id=apt-lib-dev \
    apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    gdb \
    wget gnupg \
    && rm -rf /var/lib/apt/lists/*

# Non-root user matching typical host UID, with video (camera) and passwordless sudo
RUN useradd -m -u 1000 -s /bin/zsh ubuntu && \
    mkdir -p /ws && chown ubuntu:ubuntu /ws && \
    usermod -aG video ubuntu && \
    usermod -aG sudo ubuntu && \
    echo "ubuntu ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/ubuntu

# SSH key — generated at build time for remote deployment to runtime container
RUN mkdir -p /home/ubuntu/.ssh && \
    ssh-keygen -t ed25519 -f /home/ubuntu/.ssh/id_rsa -N '' -C "omrobot-dev" && \
    chown -R ubuntu:ubuntu /home/ubuntu/.ssh

# oh-my-zsh for ubuntu user (copy from root install in base-dev)
COPY --from=omrobot-base-dev --chown=ubuntu:ubuntu \
    /root/.oh-my-zsh /home/ubuntu/.oh-my-zsh
RUN echo 'export ZSH="$HOME/.oh-my-zsh"' > /home/ubuntu/.zshrc && \
    echo 'ZSH_THEME="powerlevel10k/powerlevel10k"' >> /home/ubuntu/.zshrc && \
    echo 'plugins=(git zsh-syntax-highlighting zsh-autosuggestions extract z)' >> /home/ubuntu/.zshrc && \
    echo 'source $ZSH/oh-my-zsh.sh' >> /home/ubuntu/.zshrc && \
    echo '[[ ! -f ~/.p10k.zsh ]] || source ~/.p10k.zsh' >> /home/ubuntu/.zshrc && \
    chown ubuntu:ubuntu /home/ubuntu/.zshrc

# p10k theme config (generated by `p10k configure`)
COPY --chown=ubuntu:ubuntu config/p10k.zsh /home/ubuntu/.p10k.zsh
COPY .clang-format .clang-tidy /ws/
COPY scripts/entrypoint-dev.sh /entrypoint-dev.sh
RUN chmod +x /entrypoint-dev.sh

COPY scripts/deploy-remote scripts/generate-compile-commands.sh \
     scripts/ssh-remote scripts/sync-remote \
     scripts/build-local \
     /usr/local/bin/
RUN chmod +x /usr/local/bin/deploy-remote \
              /usr/local/bin/generate-compile-commands.sh \
              /usr/local/bin/ssh-remote \
              /usr/local/bin/sync-remote \
              /usr/local/bin/build-local

USER ubuntu
WORKDIR /ws

SHELL ["/bin/zsh", "-c"]
ENTRYPOINT ["/entrypoint-dev.sh"]

# =============================================================================
# Stage 3 — Runtime
# =============================================================================
FROM omrobot-base AS omrobot-runtime

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
COPY --from=omrobot-develop --chown=root:root \
    /home/ubuntu/.ssh/id_rsa.pub /root/.ssh/authorized_keys
RUN chmod 600 /root/.ssh/authorized_keys

# zsh as default shell for root (oh-my-zsh installed in base stage)
RUN echo 'export ZSH="$HOME/.oh-my-zsh"' > /root/.zshrc && \
    echo 'ZSH_THEME="powerlevel10k/powerlevel10k"' >> /root/.zshrc && \
    echo 'plugins=(git zsh-syntax-highlighting zsh-autosuggestions extract z)' >> /root/.zshrc && \
    echo 'source $ZSH/oh-my-zsh.sh' >> /root/.zshrc && \
    chsh -s /bin/zsh

COPY scripts/entrypoint-runtime.sh /entrypoint-runtime.sh
COPY scripts/supervisord.conf    /etc/supervisor/conf.d/omrobot.conf
RUN chmod +x /entrypoint-runtime.sh

SHELL ["/bin/zsh", "-c"]

# Built artifacts are expected at /ws/install (mounted or synced at deploy time)
RUN mkdir -p /ws/install

CMD ["/entrypoint-runtime.sh"]
