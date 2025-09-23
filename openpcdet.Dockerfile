# syntax=docker/dockerfile:1.7-labs
FROM ubuntu:20.04 AS build-spconv
COPY pixi.toml /project/pixi.toml
COPY pyproject.toml /project/pyproject.toml
COPY ./src/openpcdet /project/src/openpcdet/

RUN apt-get update -y && apt-get install curl -y && apt-get install linux-headers-$(uname -r) -y 
RUN bash -c "set -euo pipefail; curl -fsSL https://pixi.sh/install.sh -o install.sh; bash install.sh"
# set -euo pipefail evita que dockerfile falle silenciosamente
ENV PATH="/root/.pixi/bin:${PATH}"

WORKDIR /project

ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES all
RUN pixi install -vv && pixi run check_cuda