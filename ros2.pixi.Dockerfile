# syntax=docker/dockerfile:1.7-labs
#
FROM ghcr.io/prefix-dev/pixi:0.59.0 AS build
COPY pixi_confs/ros2.pixi.toml /app/pixi.toml

COPY packages/ros2_numpy /app/packages/ros2_numpy
COPY pixi_confs/ros2_numpy.pixi.toml /app/packages/ros2_numpy/pixi.toml

COPY packages/dynamic_lidar_interpolation /app/packages/dynamic_lidar_interpolation
COPY pixi_confs/dynamic_lidar_interpolation.pixi.toml /app/packages/dynamic_lidar_interpolation/pixi.toml
RUN apt-get update -y && apt-get install xorg openbox -y
# copy source code, pixi.toml and pixi.lock to the container
WORKDIR /app
# install dependencies to `/app/.pixi/envs/prod`
# use `--locked` to ensure the lockfile is up to date with pixi.toml
# increase file descriptor limit to avoid "No file descriptors available" error
RUN ulimit -n 65536 && pixi install --all \
# create the shell-hook bash script to activate the environment
&& pixi shell-hook -e default -s bash > /shell-hook
RUN echo "#!/bin/bash" > /app/entrypoint.sh
RUN cat /shell-hook >> /app/entrypoint.sh
# extend the shell-hook script to run the command passed to the container
RUN echo 'exec "$@"' >> /app/entrypoint.sh && chmod 0755 /app/entrypoint.sh
# RUN echo 'eval "$(pixi completion --shell bash)"' >> /app/entrypoint.sh'
# RUN pixi add ros-humble-desktop-full ros-humble-turtlesim colcon-common-extensions \
# 	"setuptools<=58.2.0" \
# 	ros-humble-joint-state-publisher \
# 	ros-humble-xacro \
# 	ros-humble-ros-ign-bridge \
# 	ros-humble-ros-ign-gazebo \
# 	ros-humble-ros-ign-image \
# 	ros-humble-ros-ign-interfaces \
# 	ros-humble-slam-toolbox \
# 	ros-humble-nav2-bringup \
# 	ros-humble-navigation2 \
# 	ros-humble-rviz2


ENTRYPOINT [ "/app/entrypoint.sh" ]
CMD ["pixi", "shell"]


# FROM ubuntu:24.04 AS production
# WORKDIR /app
# # only copy the production environment into prod container
# # please note that the "prefix" (path) needs to stay the same as in the build container
# COPY --from=build /app/.pixi/envs/prod /app/.pixi/envs/prod
# COPY --from=build --chmod=0755 /app/entrypoint.sh /app/entrypoint.sh

# # # copy your project code into the container as well
# # COPY ./my_project /app/my_project

# ENTRYPOINT [ "/app/entrypoint.sh" ]
# # run your app inside the pixi environment

# RUN pixi shell && ros2 run turtlesim turtlesim_node
# # CMD [ "uvicorn", "my_project:app", "--host", "0.0.0.0" ]

