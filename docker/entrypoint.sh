#!/bin/bash
# Entrypoint контейнера ar3-dev.
# Работает и как docker-entrypoint (`docker run`), и как источник окружения
# для `docker compose exec` (см. docker-compose.yml: command).

set -e

source /opt/ros/noetic/setup.bash

# Overlay workspace поверх ROS: после первой сборки catkin_make
WS=/home/albert/catkin_ws
if [ -f "$WS/devel/setup.bash" ]; then
    source "$WS/devel/setup.bash"
fi

cd "$WS"

# docker compose exec запускает эту команду как PID внутри контейнера,
# чтобы shell получил окружение ROS; при docker run $@ = CMD (bash)
exec "$@"
