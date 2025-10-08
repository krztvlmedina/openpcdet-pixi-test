# syntax=docker/dockerfile:1.7-labs
FROM ubuntu:22.04 AS pixi-image

RUN apt-get update -y && apt-get install curl -y && apt-get install linux-headers-generic -y && apt-get install git -y
RUN apt-get install -y libgl1-mesa-dev && apt-get install -y libglib2.0-0
RUN bash -c "set -euo pipefail; curl -fsSL https://pixi.sh/install.sh -o install.sh; bash install.sh"
# set -euo pipefail evita que dockerfile falle silenciosamente
ENV PATH="/root/.pixi/bin:${PATH}"

FROM pixi-image
COPY ./pixi_confs/pixi.toml /project/pixi.toml

COPY ./src/openpcdet /project/src/openpcdet/
COPY ./pixi_confs/openpcdet.pyproject.toml /project/src/openpcdet/pyproject.toml

COPY ./packages/ros2_numpy /project/packages/ros2_numpy/
COPY ./pixi_confs/ros2_numpy.pixi.toml /project/packages/ros2_numpy/pixi.toml

COPY ./packages/pcdet_ros2 /project/packages/pcdet_ros2/
COPY ./pixi_confs/pcdet_ros2.pixi.toml /project/packages/pcdet_ros2/pixi.toml

WORKDIR /project
ENV PATH="/root/.pixi/bin:${PATH}"
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES all
RUN pixi install -v
# RUN pixi run check_cuda