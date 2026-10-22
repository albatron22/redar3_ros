# Среда разработки AR3 (ROS 1 Noetic).
# База — официальный образ (robot-вариант): ros-base + build-essential + инициализированный rosdep.
# https://hub.docker.com/_/ros
FROM ros:noetic-robot-focal

# ros-noetic-desktop: rviz, rqt, joint_state_publisher, robot_state_publisher, xacro
#   (рецепт официального desktop-образа; метапакет robot уже в базе)
# moveit / moveit-visual-tools / pcl-ros / pcl-conversions — зависимости сборки
#   ar3_control и конфигурации ar3_moveit_config (в desktop НЕ входят)
# ros-control / ros-controllers — стек ros_control (controller_manager,
#   transmission_interface, joint_limits_interface) для ar3_hardware_interface
#   (в desktop не входят; rosdep их не ставит — noetic EOL, индекс пропускается)
# Для симуляции ar3_gazebo раскомментировать gazebo-ros-pkgs (тянет gazebo11)
#   и пересобрать образ: docker compose build --no-cache
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-noetic-desktop \
        ros-noetic-moveit \
        ros-noetic-moveit-visual-tools \
        ros-noetic-pcl-ros \
        ros-noetic-pcl-conversions \
        ros-noetic-ros-control \
        ros-noetic-ros-controllers \
        # ros-noetic-gazebo-ros-pkgs \
    && rm -rf /var/lib/apt/lists/*

# Пользователь с UID/GID хоста: артефакты catkin (build/, devel/) в смонтированном
# workspace принадлежат хостовому пользователю, а не root.
# USER_UID/USER_GID передаются при сборке (docker compose build --build-arg);
# значения по умолчанию 1000/1000 соответствуют первому пользователю Ubuntu-хоста
ARG USERNAME=albert
ARG USER_UID=1000
ARG USER_GID=1000
RUN if ! getent group ${USER_GID} >/dev/null; then groupadd --gid ${USER_GID} ${USERNAME}; fi \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME} \
    && usermod -aG dialout ${USERNAME} \
    && apt-get update -qq && apt-get install -y -qq --no-install-recommends sudo \
    && rm -rf /var/lib/apt/lists/* \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

# Окружение ROS в интерактивных shell (в т.ч. `docker compose exec ar3 bash`,
# который НЕ проходит через ENTRYPOINT)
RUN echo "source /opt/ros/noetic/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo '[ -f ~/catkin_ws/devel/setup.bash ] && source ~/catkin_ws/devel/setup.bash' >> /home/${USERNAME}/.bashrc

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

USER ${USERNAME}
WORKDIR /home/${USERNAME}/catkin_ws
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
