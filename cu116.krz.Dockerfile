FROM nvidia/cuda:11.6.2-devel-ubuntu20.04

# Set environment variables
ENV NVENCODE_CFLAGS "-I/usr/local/cuda/include"
ENV CV_VERSION=4.2.0
ENV DEBIAN_FRONTEND=noninteractive

# Get all dependencies
RUN apt-get update && apt-get install -y \
    git zip unzip libssl-dev libcairo2-dev lsb-release libgoogle-glog-dev libgflags-dev libatlas-base-dev libeigen3-dev software-properties-common \
    build-essential cmake pkg-config libapr1-dev autoconf automake libtool curl libc6 libboost-all-dev debconf libomp5 libstdc++6 \
    libqt5core5a libqt5xml5 libqt5gui5 libqt5widgets5 libqt5concurrent5 libqt5opengl5 libcap2 libusb-1.0-0 libatk-adaptor neovim \
    python3-pip python3-tornado python3-dev python3-numpy python3-virtualenv libpcl-dev libgoogle-glog-dev libgflags-dev libatlas-base-dev \
    libsuitesparse-dev python3-pcl pcl-tools libgtk2.0-dev libavcodec-dev libavformat-dev libswscale-dev libtbb2 libtbb-dev libjpeg-dev \
    libpng-dev libtiff-dev libdc1394-22-dev xfce4-terminal &&\
    rm -rf /var/lib/apt/lists/*

# OpenCV with CUDA support
WORKDIR /opencv
RUN git clone https://github.com/opencv/opencv.git -b $CV_VERSION &&\
    git clone https://github.com/opencv/opencv_contrib.git -b $CV_VERSION

# While using OpenCV 4.2.0 we have to apply some fixes to ensure that CUDA is fully supported, thanks @https://github.com/gismo07 for this fix
RUN mkdir opencvfix && cd opencvfix &&\
    git clone https://github.com/opencv/opencv.git -b 4.5.2 &&\
    cd opencv/cmake &&\
    cp -r FindCUDA /opencv/opencv/cmake/ &&\
    cp FindCUDA.cmake /opencv/opencv/cmake/ &&\
    cp FindCUDNN.cmake /opencv/opencv/cmake/ &&\
    cp OpenCVDetectCUDA.cmake /opencv/opencv/cmake/
 
WORKDIR /opencv/opencv/build

RUN cmake -D CMAKE_BUILD_TYPE=RELEASE \
-D CMAKE_INSTALL_PREFIX=/usr/local \
-D OPENCV_GENERATE_PKGCONFIG=ON \
-D BUILD_EXAMPLES=OFF \
-D INSTALL_PYTHON_EXAMPLES=OFF \
-D INSTALL_C_EXAMPLES=OFF \
-D PYTHON_EXECUTABLE=$(which python2) \
-D PYTHON3_EXECUTABLE=$(which python3) \
-D PYTHON3_INCLUDE_DIR=$(python3 -c "from distutils.sysconfig import get_python_inc; print(get_python_inc())") \
-D PYTHON3_PACKAGES_PATH=$(python3 -c "from distutils.sysconfig import get_python_lib; print(get_python_lib())") \
-D BUILD_opencv_python2=ON \
-D BUILD_opencv_python3=ON \
-D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules/ \
-D WITH_GSTREAMER=ON \
-D WITH_CUDA=ON \
-D ENABLE_PRECOMPILED_HEADERS=OFF \
.. &&\
make -j$(nproc) &&\
make install &&\
ldconfig &&\
rm -rf /opencv

WORKDIR /
ENV OpenCV_DIR=/usr/share/OpenCV

# PyTorch for CUDA 11.6
RUN pip3 install typing-extensions==4.12.2 torch==1.13.0+cu116 torchvision==0.14.0+cu116 torchaudio==0.13.1 --extra-index-url https://download.pytorch.org/whl/cu116
ENV TORCH_CUDA_ARCH_LIST="3.5;5.0;6.0;6.1;7.0;7.5;8.0;8.6+PTX"

# OpenPCDet
RUN pip3 install numpy==1.23.0 configobj llvmlite numba tensorboardX easydict pyyaml scikit-image \
    tqdm SharedArray open3d mayavi av2 kornia==0.5.8 pyquaternion

RUN pip3 install spconv-cu116

RUN git clone https://github.com/open-mmlab/OpenPCDet.git

WORKDIR OpenPCDet

RUN python3 setup.py develop

WORKDIR /

ENV NVIDIA_VISIBLE_DEVICES="all" \
    OpenCV_DIR=/usr/share/OpenCV \
    NVIDIA_DRIVER_CAPABILITIES="video,compute,utility,graphics" \
    LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/lib:/usr/lib:/usr/local/lib \
    QT_GRAPHICSSYSTEM="native"

RUN apt-get update -y && apt update -y && apt-get install curl -y && apt-get install linux-headers-generic -y && apt-get install git -y
# RUN bash -c "set -euo pipefail; curl -fsSL https://pixi.sh/install.sh -o install.sh; bash install.sh"
# # set -euo pipefail evita que dockerfile falle silenciosamente
# ENV PATH="/root/.pixi/bin:${PATH}"

# COPY pixi_confs/ros2.pixi.toml /OpenPCDet/pixi.toml

COPY ./packages/ros2_numpy /OpenPCDet/packages/ros2_numpy
COPY pixi_confs/ros2_numpy.pixi.toml /OpenPCDet/packages/ros2_numpy/pixi.toml

COPY ./packages/dynamic_lidar_interpolation /OpenPCDet/packages/dynamic_lidar_interpolation
COPY pixi_confs/dynamic_lidar_interpolation.pixi.toml /OpenPCDet/packages/dynamic_lidar_interpolation/pixi.toml


### ROS2 HUMBLE INSTALLATION
# Install ROS2 Humble from source on Ubuntu 20.04
RUN apt-get update && apt-get install -y \
    locales \
    software-properties-common \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && add-apt-repository universe \
    && rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8

# Add ROS2 apt repository
RUN apt-get update && apt-get install -y \
    curl \
    gnupg \
    lsb-release \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && rm -rf /var/lib/apt/lists/*

# Install development tools and ROS tools
RUN apt-get update && apt-get install -y \
    python3-flake8-docstrings \
    python3-pip \
    python3-pytest-cov \
    ros-dev-tools 

RUN python3 -m pip install -U \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-import-order \
    flake8-quotes \
    "pytest>=5.3" \    
    pytest-repeat \
    pytest-rerunfailures \
    && rm -rf /var/lib/apt/lists/*

# Create workspace for ROS2 Humble
WORKDIR /opt/ros2_humble
RUN mkdir -p src

# Get ROS2 Humble source code
RUN vcs import --input https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos src

# Clone perception_pcl for PCL support
WORKDIR /opt/ros2_humble/src
RUN git clone https://github.com/ros-perception/pcl_msgs.git -b ros2 && \
    git clone https://github.com/ros-perception/perception_pcl.git -b humble

ENV ROS_DISTRO=humble

# Upgrade CMake to 3.27 (compatible with ROS2 Humble)
RUN apt-get update && apt-get install -y wget && \
    wget https://github.com/Kitware/CMake/releases/download/v3.27.9/cmake-3.27.9-linux-x86_64.sh && \
    chmod +x cmake-3.27.9-linux-x86_64.sh && \
    ./cmake-3.27.9-linux-x86_64.sh --prefix=/usr/local --skip-license && \
    rm cmake-3.27.9-linux-x86_64.sh && \
    rm -rf /var/lib/apt/lists/*

# Install dependencies using rosdep
WORKDIR /opt/ros2_humble
RUN apt-get update && \
    rosdep init || true && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers" && \
    rm -rf /var/lib/apt/lists/*

# Build ROS2 Humble with PCL and visualization support
# Building packages needed for PCL and MarkerArray
RUN colcon build --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --packages-up-to \
    rclcpp \
    rclpy \
    std_msgs \
    sensor_msgs \
    geometry_msgs \
    visualization_msgs \
    pcl_conversions \
    pcl_ros \
    tf2 \
    tf2_ros \
    tf2_geometry_msgs 

RUN colcon build --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --packages-up-to \
     sensor_msgs_py

# Setup environment
RUN echo ". /opt/ros2_humble/install/setup.sh" >> ~/.bashrc
ENV AMENT_PREFIX_PATH=/opt/ros2_humble/install
ENV COLCON_PREFIX_PATH=/opt/ros2_humble/install
ENV LD_LIBRARY_PATH=/opt/ros2_humble/install/lib:$LD_LIBRARY_PATH
ENV PATH=/opt/ros2_humble/install/bin:$PATH
ENV PYTHONPATH=/opt/ros2_humble/install/lib/python3.8/site-packages:$PYTHONPATH
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2

WORKDIR /



### ROS2 GALACTIC INSTALLATION
# RUN apt install software-properties-common & add-apt-repository universe && apt update -y
# RUN apt install curl -y

# # set -euo pipefail evita que dockerfile falle silenciosamente
# RUN bash -c "set -euo pipefail; \
#     curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg;"
    
# RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# RUN apt update -y && apt upgrade -y && apt install -y ros-galactic-ros-base && apt install ros-dev-tools -y
# RUN echo ". /opt/ros/galactic/setup.sh" >> ~/.bashrc



WORKDIR OpenPCDet

### GALACTIC
# RUN apt install ros-galactic-pcl-ros -y && apt install libeigen3-dev -y \
#     && apt install ros-galactic-common-interfaces && apt-get install ros-galactic-sensor-msgs-py \
#     && apt-get install -y ros-galactic-rmw-fastrtps-cpp


# RUN . /opt/ros/humble/setup.sh && colcon build --packages-select dynamic_lidar_interpolation --cmake-clean-cache && . /opt/ros/humble/setup.sh
# INSTALL ROS PACKAGES

# RUN pixi install -v --all

# Build instructions: docker build -f minimal.Dockerfile -t openpcdet:cuda11 .
# Start instructions: xhost local:root && docker run -it --rm -e SDL_VIDEODRIVER=x11 -e DISPLAY=$DISPLAY --env='DISPLAY' --gpus all --ipc host --privileged --network host -p 8080:8081 -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v file_locations:/storage -v /weights:/weights openpcdet:cuda11 xfce4-terminal --title=openPCDet
