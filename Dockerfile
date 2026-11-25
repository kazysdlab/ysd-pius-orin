FROM nvcr.io/nvidia/l4t-jetpack:r36.4.0

ARG TARGETPLATFORM
LABEL maintainer="namazyake"

SHELL ["/bin/bash", "-c"]

# --- User ---
ARG USERNAME=orin
ARG UID=1000
ARG GID=1000

# --- Base ---
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
        wget curl git sudo python3-pip tini \
        build-essential vim nano sudo lsb-release locales \
        bash-completion tzdata terminator \
        python3-dev python3-numpy && \
    apt-get autoclean && \
    apt-get autoremove && \
    rm -rf /var/lib/apt/lists/*

# --- ROS2 ---
ENV ROS_DISTRO=humble
ARG INSTALL_PACKAGE=ros-base

RUN apt-get update -q && \
    apt-get install -y curl gnupg2 lsb-release && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update -q && \
    apt-get install -y ros-${ROS_DISTRO}-${INSTALL_PACKAGE} \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-rosdep python3-vcstool && \
    rosdep init && \
    rm -rf /var/lib/apt/lists/*

RUN rosdep update || true

# --- Upgrade pip ---
RUN pip install --upgrade pip

# --- Jupyter Lab ---
RUN pip install black jupyterlab jupyterlab_code_formatter jupyterlab-git \
    lckr-jupyterlab-variableinspector jupyterlab_widgets ipywidgets import-ipynb

# --- PyTorch ---
ARG CUDA_VERSION=12.1
ARG TORCH_INSTALL=https://developer.download.nvidia.com/compute/redist/jp/v61/pytorch/torch-2.5.0a0+872d972e41.nv24.08.17622132-cp310-cp310-linux_aarch64.whl
RUN apt-get update && apt-get install -y libopenblas-dev
RUN wget raw.githubusercontent.com/pytorch/pytorch/5c6af2b583709f6176898c017424dc9981023c28/.ci/docker/common/install_cusparselt.sh && \
    bash ./install_cusparselt.sh
RUN pip install --upgrade pip && pip install numpy==1.26.1 && pip install --no-cache $TORCH_INSTALL

# --- TorchVision ---
RUN git clone --branch release/0.20 https://github.com/pytorch/vision torchvision
ARG BUILD_VERSION=0.20.0
RUN cd torchvision && \
    pip install --upgrade setuptools wheel && \
    pip install --no-cache-dir .

# --- APT Clean ---
RUN rm /etc/apt/apt.conf.d/docker-clean

# --- Optional install ---
# SMBus2
RUN apt-get update -q && \
    apt-get install -y python3-smbus

# Python library
COPY requirements.txt .
RUN pip install -r requirements.txt && \
    pip install onnxruntime-gpu --index-url https://pypi.jetson-ai-lab.io/jp6/cu126

# Groups
RUN groupadd -g 116 i2c_host

# --- Create User ---
RUN groupadd -g ${GID} ${USERNAME} \
    && useradd -m -u ${UID} -g ${GID} -s /bin/bash ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >/etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

ENV HOME=/home/${USERNAME}

# --- ROS2 Setup for User ---
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ${HOME}/.bashrc

ARG MOUNTED_COLCON_WORKSPACE_DIR=${HOME}/dev/ws/colcon_ws
RUN echo "source ${MOUNTED_COLCON_WORKSPACE_DIR}/install/setup.bash" >> ${HOME}/.bashrc

USER ${USERNAME}
WORKDIR ${HOME}

# --- Running command ---
CMD [ "bash" ]
