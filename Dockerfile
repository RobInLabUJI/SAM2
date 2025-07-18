# Use an NVIDIA CUDA image as the base
FROM nvidia/cuda:12.2.2-devel-ubuntu22.04

# Set up environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV PATH="${PATH}:/home/user/.local/bin"

# We love UTF!
ENV LANG C.UTF-8

RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

# Set the nvidia container runtime environment variables
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
ENV PATH /usr/local/nvidia/bin:/usr/local/cuda/bin:${PATH}
ENV CUDA_HOME="/usr/local/cuda"
ENV TORCH_CUDA_ARCH_LIST="6.0 6.1 7.0 7.5 8.0 8.6+PTX 8.9"

# Install some handy tools. Even Guvcview for webcam support!
RUN set -x \
	&& apt-get update \
	&& apt-get install -y apt-transport-https ca-certificates \
	&& apt-get install -y git vim tmux nano htop sudo curl wget gnupg2 \
	&& apt-get install -y bash-completion \
	&& apt-get install -y guvcview \
	&& rm -rf /var/lib/apt/lists/* \
	&& useradd -ms /bin/bash user \
	&& echo "user:user" | chpasswd && adduser user sudo \
	&& echo "user ALL=(ALL) NOPASSWD: ALL " >> /etc/sudoers

RUN set -x \
    && apt-get update && apt-get install ffmpeg libsm6 libxext6  -y

RUN curl -sS https://bootstrap.pypa.io/get-pip.py | python3

WORKDIR /home/user

RUN git clone https://github.com/facebookresearch/segment-anything-2 --single-branch && \
    cd segment-anything-2 && \
    python3 -m pip install -e . -v
    
RUN  cd segment-anything-2/checkpoints && ./download_ckpts.sh && cd ..

RUN apt-get update \
	&& apt-get install -y python3-opencv python3-matplotlib python3-notebook \
	&& rm -rf /var/lib/apt/lists/*
	
RUN pip uninstall numpy --yes

RUN usermod -aG dialout user
USER user
STOPSIGNAL SIGTERM

COPY build_sam.py sam2_camera_predictor.py /home/user/segment-anything-2/sam2/.

RUN sudo apt-get update && sudo apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    wget \
    && sudo rm -rf /var/lib/apt/lists/*

RUN sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN sudo echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN sudo apt-get update && sudo apt-get install -y \
    ros-humble-desktop \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-dev-tools \
    && sudo rm -rf /var/lib/apt/lists/*
    
RUN sudo rosdep init && rosdep update

RUN sudo apt-get update && sudo apt-get install -y \
    ros-humble-gscam ros-humble-image-transport-plugins \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev \
    gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-tools gstreamer1.0-x \
    gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 \
    gstreamer1.0-pulseaudio \
    && sudo rm -rf /var/lib/apt/lists/*
     
CMD /bin/bash

