# ИЗМЕНЕНИЯ В ВЕРСИИ ПАКЕТА ДЛЯ МАНИПУЛЯТОРА REDAR3 С ПРОШИВКОЙ V4.4
* Манпулятор REDAR3
* Контроллер манпулятора: Teensy 4.1
* Версия прошивки: AR4_teensy41_sketch_v4.4 (в прошивку внесены небольшие изменения см. в Obsidian).

*Данная версия пакета основовыается на репозитории https://github.com/ongdexter/ar3_core.git.*

Оснновные изменения коснулись драйвера контроллера Teensу 4.1
В файле [TeensyDriver.cpp](/ar3_hardware_drivers/src/TeensyDriver.cpp) внесены следующие изменения:

1. Startup процедура. Команды начальной инициализации Teensy 4.1 взяты из анализа протокола HMI при включении (см. в Obsidian). Оно включает основную команду `"UP"` - UPDATE PARAMS. На этой драйвер ожидает положительный ответ "Done\n", после прихода которой связь считается успешной установленной. Остальный команды процедру инициализации (send pos, calib enc) игнорируются за ненадобностью.
2. Чтения текущей позиции звеньеы (углы поворота Joint1-6). Команда запрос "RP\n". Ответ со стороны Teensy 4.1 не включает заголовка, а 
сразу начинается с индентификатора первого звена "A...". Значения углов передаются в градусах в вещественном формате.
3. Измененена команда установки требуемого положения звеньев в `update()`. Она начинается с заголовка `"RJ"`.  Включаеьтся в себя требуемые углы в градусах, максимальную скорость, ускорения разгона и торможения и др. параметры (котоорые лучше не трогать, например Loop mode, указывающий на работу без энкодеров)
4. Убрана команда установки скоростей вращения звеньев (оставлена пустай функция-заглушка)
5. Внесены изменения в функциях пересчета значений импульсов с энкодера и углы и наборотв. Теперь этого не требуется, так как энкодеры отсуствуют, а обратная связь по углам и требуемые углы передаются сразу в градусах

Содержание конфигурационных файлов также было изменено в соответсвии с новыми параметрами. Главные изменения:
1. `ar3_hardware_interface/config/hardware_driver.yaml`: имя serial порта и скорость соединения. Прописанные значения количества импульсов на градус в дальнейшем не используются.
2. `ar3_hardware_interface/config/hardware_interface.yaml`: повышена частота дискретизации контроллера управления траекторией. Таким образом достигается плавность движения.
3. `ar3_hardware_interface/config/joint_offsets.yaml`: значения offsets это не калибпровочные значения смещений угла как HMI. Данные значения подобраны, чтоюы модель в rviz совпадало с реальным роботом.
4. `ar3_moveit_config/config/joint_limits.yaml`: корректировки в димитах по осям (но лучше их не трогать, потому что когда стал менять J2 модель в rviz сломалась...)

# ЗАПУСК В DOCKER (ROS Noetic)

Пакеты разрабатывались под ROS Melodic/Noetic (ROS 1). На хост-машине установлен только ROS 2 Jazzy, поэтому сборка и запуск выполняются в Docker-контейнере на базе официального образа `ros:noetic-robot-focal`. Конфигурация находится в каталоге [`docker/`](docker): `Dockerfile`, `docker-compose.yml`, `entrypoint.sh`.

## Состав образа

Поверх официального образа установлены (см. `docker/Dockerfile`):
* `ros-noetic-desktop` — RViz, rqt, joint_state_publisher, robot_state_publisher, xacro
* `ros-noetic-moveit`, `ros-noetic-moveit-visual-tools` — MoveIt для планирования движений
* `ros-noetic-pcl-ros`, `ros-noetic-pcl-conversions` — зависимости сборки `ar3_control`
* `ros-noetic-ros-control`, `ros-noetic-ros-controllers` — стек ros_control для `ar3_hardware_interface`

Симуляция Gazebo не установлена (для неё раскомментируйте `ros-noetic-gazebo-ros-pkgs` в `docker/Dockerfile` и пересоберите образ).

**Важно:** `rosdep` для ROS Noetic не работает (дистрибутив EOL с мая 2025, индекс rosdep его пропускает), поэтому зависимости фиксируются явным списком пакетов в `Dockerfile`. При добавлении новых зависимостей в `package.xml` добавляйте соответствующий apt-пакет в `Dockerfile` вручную.

## Возможности конфигурации

* **Сеть общая с хостом** (`network_mode: host`) — ROS-мастер и ноды доступны без проброса портов.
* **USB serial** (`privileged: true`) — контейнеру доступны все устройства `/dev/ttyACM*`, `/dev/ttyUSB*`; переподключение платы не требует перезапуска контейнера.
* **Графика** — RViz и rqt работают на дисплее хоста (проброшен X11-сокет).
* **Код на хосте** — монтируется весь workspace `~/ar3_ros1_ws`, артефакты сборки (`build/`, `devel/`) остаются на хосте в корне workspace; пересборка образа при изменении кода пакетов не требуется.
* Внутри контейнера создан пользователь `albert` (UID/GID совпадают с хостовыми), артефакты сборки не ломают права на файлы. `sudo` работает без пароля.

## Сборка и запуск

Требуется установленный Docker с compose-плагином.

```bash
# 1. Сборка образа (один раз, ~5 минут; повторно — секунды за счёт кэша)
cd ~/ar3_ros1_ws/src/docker
docker compose build

# 2. Запуск контейнера в фоне
docker compose up -d

# 3. Вход в контейнер
docker compose exec ar3 bash
```

Внутри контейнера:
```bash
# сборка workspace (при первом запуске и после изменения кода)
catkin_make

# пример запуска: демо MoveIt + RViz без реального манипулятора
roslaunch ar3_moveit_config demo.launch

# работа с реальным манипулятором (два терминала внутри контейнера)
roslaunch ar3_hardware_interface ar3_hardware_bringup.launch use_existing_calibrations:=true
roslaunch ar3_moveit_config ar3_moveit_bringup.launch
```

## Графика (RViz) на хосте

X11-авторизация для контейнера включается на хосте **один раз за X-сессию** (после перезагрузки — снова):
```bash
xhost +local:
```
Без этого RViz в контейнере упадёт с ошибкой подключения к дисплею.

## Предупреждение о калибровке

При старте `ar3_hardware_bringup.launch` **без** аргумента `use_existing_calibrations:=true` выполняется калибровка суставов — **манипулятор физически движется**. Убедитесь, что вокруг него нет препятствий и людей. Аргумент пропускает калибровку и использует сохранённые значения.

## Администрирование контейнера

```bash
docker ps                                          # статус контейнера ar3-dev
docker compose -f ~/ar3_ros1_ws/src/docker/docker-compose.yml stop    # остановить
docker compose -f ~/ar3_ros1_ws/src/docker/docker-compose.yml start   # запустить снова
docker compose -f ~/ar3_ros1_ws/src/docker/docker-compose.yml down    # удалить контейнер
docker exec -u root ar3-dev bash -c '<команда>'    # разовое действие от root
docker exec ar3-dev bash -c '<команда>'            # разовое действие от albert
```
Контейнер настроен с `restart: unless-stopped` — после перезагрузки хоста Docker запустит его автоматически (графику не забудьте: `xhost +local:`).

## Типичные проблемы

| Симптом | Причина и решение |
|---|---|
| `rviz: cannot connect to display` | Не выданы права X11: выполните `xhost +local:` на хосте |
| `Failed to connect to serial port /dev/ttyACM0` | Плата не подключена или занята другим процессом; проверьте `ls /dev/ttyACM*` |
| `catkin_make: command not found` | Не выполнен `source /opt/ros/noetic/setup.bash` (в интерактивном shell `.bashrc` делает это сам) |
| Сборка не видит зависимости | Пакет не добавлен в `docker/Dockerfile` (rosdep для noetic не работает) |
| `docker: permission denied` на хосте | Добавьте пользователя в группу `docker`: `sudo usermod -aG docker $USER` (перелогин) |


# AR3 Core Software
This repository provides the software for control of the AR3 arm with ros_control and MoveIt. I hope to provide fellow robotic arm enthusiasts with a starting point to explore using ROS for controlling the arm. The baseline implementation is designed to accomodate the original hardware and firmware, including the message structures for communication. Moving forward, I will do my best to continue to keep it accessible. I plan to explore some interesting concepts that I have in mind for on-arm vision, grippers and human-robot interaction and I will share my projects here if possible.

You may also wish to check out some hardware modifications that I've made for joint encoders at [ar3_hardware_mods](https://github.com/ongdexter/ar3_hardware_mods).

* [Overview](#Overview)
* [Installation](#Installation)
* [Usage](#Usage)

[![](http://img.youtube.com/vi/6D4vdhJlLsQ/0.jpg)](http://www.youtube.com/watch?v=6D4vdhJlLsQ "AR3 with ROS and MoveIt")

## Overview
+ **ar3_control**
    + Controlling the arm through the MoveIt user interfaces
    + Provides demo for the move group interface
    + Provides joint sweep demo (all six joints through their range of motion)
    
+ **ar3_description**
    + Hardware description of arm, urdf etc.
    
+ **ar3_hardware_interface**
    + ROS interface for the hardware driver, built on the ros_control framework
    + Manages joint offsets, limits and conversion between joint and actuator messages
    
+ **ar3_hardware_drivers**
    + Handles communication with the motor controllers

+ **ar3_microcontrollers**
    + Firmware for the motor controller ie. Teensy

+ **ar3_moveit_config**
    + MoveIt module for motion planning
    + Controlling the arm through Rviz

+ **ar3_gazebo**
    + Simulation on Gazebo


## Installation
*Actively developed and tested on ROS Melodic on Windows 10 with limited testing on Ubuntu 18.04.*

* [Windows 10](#Windows)
* [Ubuntu 18.04](#Ubuntu)


### Windows
* Install [ROS Melodic and MoveIt](https://moveit.ros.org/install/) for Windows 10
* Create the ROS workspace `ie. ar3_ws`
  ```
  md C:\ar3_ws\src && C:\ar3_ws\src
  ```

* Clone this repository into the workspace `src`:
  ```
  git clone https://github.com/ongdexter/ar3_core.git .
  ```
  The workspace directory should be as such:
  ```
  ar_ws
  +-- src
  |   +-- ar3_control
  |   +-- ar3_description
  |   +-- ...
  ```
* Build the workspace:
  ```
  cd ~/ar3_ws
  catkin_make
  ```
* Source the workspace

  ```
  call C:\ar3_ws\setup.bat
  ```
  You can add this to the ROS Command Window shortcut that you created during the ROS installation, by appending `&& c:\ar3_ws\devel\setup.bat` to the shortcut target, so that it is automatically run each time a new terminal is opened.

-----

### Ubuntu
* Install [ROS Melodic and MoveIt](https://moveit.ros.org/install/) for Ubuntu 18.04
* Create the ROS workspace `ie. ar3_ws`
  ```
  mkdir -p ~/ar3_ws/src && cd "$_"
  ```
  
* Clone this repository into the workspace `src`:
  ```
  git clone https://github.com/ongdexter/ar3_core.git .
  ```
  The workspace directory should be as such:
  ```
  ar_ws
  +-- src
  |   +-- ar3_control
  |   +-- ar3_description
  |   +-- ...
  ```
* Install workspace dependencies:
  ```
  cd ~/ar3_ws
  rosdep install --from-paths src --ignore-src -r -y
  ```
* Build the workspace:
  ```
  catkin_make
  ```
* Source the workspace:
  ```
  source ~/ar3_ws/devel/setup.bash
  ```
  You can add this to your .bashrc so that it is automatically run each time a new terminal is opened:
  ```
  echo "source ~/ar3_ws/devel/setup.bash" >> ~/.bashrc
  ```
* Enable serial port access if you haven't already done so:
  ```
  sudo addgroup $USER dialout
  ```
  You will need to log out and back in for changes to take effect.

## Setup
* **Hardware interface**  
  - Set the serial port and baudrate in `ar3_hardware_interface/config/hardware_driver.yaml`
  
* **Teensy Sketch**  
  - Both teensy sketches provided are compatible with the default hardware. Refer to the module for more information.

## Usage
There are two modules that you will always need to run:

1. **Arm module** - this can be for either a real-world or simulated arm
     + For controlling the real-world arm, you will need to run the `ar3_hardware_interface` module 
     + For the simulated arm, you will need to run the `ar3_gazebo` module
     + Either of the modules will load the necessary hardware descriptions for MoveIt

2. **MoveIt module** - the `ar3_moveit_config` module provides the MoveIt interface and RViz GUI, and the `ar3_control` module provides the MoveIt user interface for programmatically setting goals

The various use cases of the modules and instructions to run them are described below:

-----

### MoveIt Demo in RViz
If you are unfamiliar with MoveIt, it is recommended to start with this to explore planning with MoveIt in RViz. This contains neither a real-world nor a simulated arm but just a model loaded within RViz for visualisation.
* The robot description, moveit interface and RViz will all be loaded in the single demo launch file
  ```
  roslaunch ar3_moveit_config demo.launch
  ```

-----  

### Control real-world arm with MoveIt in RViz

* Start the `ar3_hardware_interface` module, which will load configs and the robot description
  ```
  roslaunch ar3_hardware_interface ar3_hardware_bringup.launch
  ```
  The hardware interface will also start the hardware driver and initialise communication with the Teensy. You can skip the joint encoder calibration sequence with the `use_existing_calibrations` argument when starting the node  
  ie. `roslaunch ar3_hardware_interface ar3_hardware_bringup.launch use_existing_calibrations:=true`.
  
  
* Start MoveIt and RViz
  ```
  roslaunch ar3_moveit_config ar3_moveit_bringup.launch
  ```
You can now plan in RViz and control the real-world arm. Joint commands and joint states will be updated through the hardware interface.

-----

### Control simulated arm in Gazebo with MoveIt in RViz

* Start the `ar3_gazebo` module, which will start the Gazebo simulator and load the robot description
  ```
  roslaunch ar3_gazebo ar3_gazebo_bringup.launch
  ```
* Start Moveit and RViz
  ```
  roslaunch ar3_moveit_config ar3_moveit_bringup.launch
  ```
You can now plan in RViz and control the simulated arm.

-----
### Control arm with Move Group Interface

**It is recommended to run this demo with the simulated arm first to make sure that the programmed goals are safe for your environment (and your arm). Needless to say, the same applies when programming your own tasks.**

This is a demo modified from the official MoveIt tutorials. As opposed to manually setting goals through RViz, the move group interface allows us to programmatically plan and execute movements, and provides more functionality such as specifying path constraints and planning Cartesian movements. This also enables much more complex tasks, planning around obstacles etc.

* Start the `ar3_gazebo` module, which will start the Gazebo simulator and load the robot description  
*For controlling the real-world arm, you will just need to run `ar3_hardware_interface` instead of `ar3_gazebo` as described above.*
  ```
  roslaunch ar3_gazebo ar3_gazebo_bringup.launch
  ```
* Start Moveit and RViz
  ```
  roslaunch ar3_moveit_config ar3_moveit_bringup.launch
  ```
* Start the move group demo
  ```
  roslaunch ar3_control move_group_demo.launch
  ```
  Follow the command-line instructions to step through the demo. See `ar3_control` for more details.

-----

### Joint Sweep Demo (all six joints)

Demonstrates the range of motion of the manipulator: each joint from 1 to 6 is swept to both sides of the home pose (`ready` from the SRDF) and returned back, one joint at a time. Targets are clipped to the real joint limits and to an optional home-relative window, failed steps are retried with a reduced amplitude.

**It is recommended to run this demo on the simulated arm or in RViz-only demo first, and start on the real arm with a reduced `range_fraction`/`amplitude_scale`.**

* Run any bringup layer first:
  * RViz-only demo (no arm):
    ```
    roslaunch ar3_moveit_config demo.launch
    ```
  * Simulated arm in Gazebo:
    ```
    roslaunch ar3_gazebo ar3_gazebo_bringup.launch
    roslaunch ar3_moveit_config ar3_moveit_bringup.launch
    ```
  * Real arm (skip the calibration motion if calibrations already exist):
    ```
    roslaunch ar3_hardware_interface ar3_hardware_bringup.launch use_existing_calibrations:=true
    roslaunch ar3_moveit_config ar3_moveit_bringup.launch
    ```
* Start the joint sweep demo:
  ```
  roslaunch ar3_control joint_sweep_demo.launch
  ```
  In the default step mode, press `next` in the RvizVisualToolsGui panel in RViz before every motion. To run the whole sequence automatically:
  ```
  roslaunch ar3_control joint_sweep_demo.launch step_mode:=false
  ```
* Optional arguments (see `ar3_control` for the full list):
  * `range_fraction:=0.3` — limit each joint to a 30%-of-range window around home
  * `amplitude_scale:=0.3` — smaller sweep amplitude
  * `velocity_scale:=0.2` — slower trajectories

-----