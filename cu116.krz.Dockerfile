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


# RUN add-apt-repository ppa:deadsnakes/ppa
# RUN apt-get update
# RUN apt-get install -y --no-install-recommends \
#        python3.9 python3.9-dev python3.9-distutils python3.9-venv build-essential \
#     && rm -rf /var/lib/apt/lists/*

#     # Install pip for python3.9 using get-pip.py and upgrade pip/setuptools/wheel
# RUN curl -sS https://bootstrap.pypa.io/get-pip.py -o /tmp/get-pip.py \
#     && python3.9 /tmp/get-pip.py \
#     && python3.9 -m pip install --no-cache-dir --upgrade pip setuptools wheel \
#     && rm -f /tmp/get-pip.py

# # Make /usr/bin/python3 point to python3.9 (so scripts calling `python3` use 3.9)
# RUN update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.9 1

# # Ensure `python3 -m pip` uses the python3.9 pip (install pip for 'python3' name)
# RUN python3 -m pip install --no-cache-dir --upgrade pip

# # Optional: make pip3 point to the python3 pip executable (safe; most images expect pip3)
# RUN ln -sf /usr/bin/python3 /usr/bin/python && ln -sf /usr/bin/python3 /usr/bin/python3.0 

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

RUN apt-get update -y && apt-get install curl -y && apt-get install linux-headers-generic -y && apt-get install git -y
# RUN bash -c "set -euo pipefail; curl -fsSL https://pixi.sh/install.sh -o install.sh; bash install.sh"
# # set -euo pipefail evita que dockerfile falle silenciosamente
# ENV PATH="/root/.pixi/bin:${PATH}"

# COPY pixi_confs/ros2.pixi.toml /OpenPCDet/pixi.toml

COPY ./packages/ros2_numpy /OpenPCDet/packages/ros2_numpy
COPY pixi_confs/ros2_numpy.pixi.toml /OpenPCDet/packages/ros2_numpy/pixi.toml

COPY ./packages/dynamic_lidar_interpolation /OpenPCDet/packages/dynamic_lidar_interpolation
COPY pixi_confs/dynamic_lidar_interpolation.pixi.toml /OpenPCDet/packages/dynamic_lidar_interpolation/pixi.toml


# ROS2 GALACTIC INSTALLATION
RUN apt install software-properties-common & add-apt-repository universe && apt update -y
RUN apt install curl -y


# RUN set -eux; \
#     ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}'); \
#     echo "ROS_APT_SOURCE_VERSION=$ROS_APT_SOURCE_VERSION" >> /etc/environment
# RUN echo "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
# RUN curl -v -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" \
#     && dpkg -i /tmp/ros2-apt-source.deb

# set -euo pipefail evita que dockerfile falle silenciosamente
RUN bash -c "set -euo pipefail; \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg;"
    
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt update -y && apt upgrade -y && apt install -y ros-galactic-ros-base && apt install ros-dev-tools -y
RUN echo ". /opt/ros/galactic/setup.sh" >> ~/.bashrc


RUN apt update -y && apt upgrade -y && apt install -y ros-galactic-ros-base && apt install ros-dev-tools -y
RUN echo ". /opt/ros/galactic/setup.sh" >> ~/.bashrc



WORKDIR OpenPCDet
RUN apt install ros-galactic-pcl-ros -y && apt install libeigen3-dev -y \
    && apt install ros-galactic-common-interfaces && apt-get install ros-galactic-sensor-msgs-py

# RUN . /opt/ros/humble/setup.sh && colcon build --packages-select dynamic_lidar_interpolation --cmake-clean-cache && . /opt/ros/humble/setup.sh
# INSTALL ROS PACKAGES

# RUN pixi install -v --all

# Build instructions: docker build -f minimal.Dockerfile -t openpcdet:cuda11 .
# Start instructions: xhost local:root && docker run -it --rm -e SDL_VIDEODRIVER=x11 -e DISPLAY=$DISPLAY --env='DISPLAY' --gpus all --ipc host --privileged --network host -p 8080:8081 -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v file_locations:/storage -v /weights:/weights openpcdet:cuda11 xfce4-terminal --title=openPCDet
