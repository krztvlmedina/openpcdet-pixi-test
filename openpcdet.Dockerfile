# syntax=docker/dockerfile:1.7-labs
FROM ubuntu:22.04 AS pixi-image

RUN apt-get update -y && apt-get install curl -y && apt-get install linux-headers-generic -y 
RUN bash -c "set -euo pipefail; curl -fsSL https://pixi.sh/install.sh -o install.sh; bash install.sh"
# set -euo pipefail evita que dockerfile falle silenciosamente
ENV PATH="/root/.pixi/bin:${PATH}"

FROM pixi-image
COPY pixi.toml /project/pixi.toml
COPY openpcdet.pyproject.toml /project/src/openpcdet/pyproject.toml
COPY ./src/openpcdet /project/src/openpcdet/
COPY ./src/ros2_numpy /project/src/ros2_numpy/
COPY ./src/pcdet_ros2 /project/src/pcdet_ros2/
WORKDIR /project
ENV PATH="/root/.pixi/bin:${PATH}"
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES all
RUN pixi install -v
# RUN pixi run check_cuda