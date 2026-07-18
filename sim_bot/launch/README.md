# Launch files

Esta pasta contem os arquivos de inicializacao ROS 2 do pacote `sim_bot`.

No fluxo atual, o `sim_bot` ficou focado na simulacao do `palmares_bot`. Os
arquivos antigos do `diff_bot` e `four_wheel` foram removidos.

## Fluxo Principal

```text
sim_manager.launch.py
    -> sim_essentials.launch.py
    -> sim_navigation.launch.py
            -> nav_hub/launch/route_graph/sim_nav_graph.launch.py
```

## sim_manager.launch.py

E o orquestrador principal.

Ele le os valores padrao de:

```text
config/launch_params.yaml
```

E repassa os argumentos para as camadas de simulacao e navegacao.

Uso padrao:

```bash
ros2 launch sim_bot sim_manager.launch.py
```

## sim_essentials.launch.py

Sobe a parte basica da simulacao:

```text
Gazebo
robot_state_publisher
spawn do robo no Gazebo
ros_gz_bridge
RViz opcional
teleoperacao opcional
```

O `robot_state_publisher` e iniciado diretamente neste arquivo usando o
argumento:

```text
robot_urdf:=...
```

Por padrao, ele usa:

```text
description/palmares_bot.urdf.xacro
```

## sim_navigation.launch.py

Sobe a camada de navegacao da simulacao:

```text
slam.launch.py, se slam:=True
nav_hub/launch/route_graph/sim_nav_graph.launch.py, se nav:=True
```

Por padrao, o arquivo de parametros Nav2 vem do pacote `nav_hub`:

```text
nav_hub/config/sim_nav_params.yaml
```

## nav_hub/launch/route_graph/sim_nav_graph.launch.py

Sobe a pilha Nav2 adaptada para simulacao:

```text
map_server
amcl
controller_server
smoother_server
planner_server
behavior_server
bt_navigator
waypoint_follower
velocity_smoother
route_server, se route:=True
speed_filter, se speed_filter:=True
lifecycle managers
```

Mapa, grafo, mascara de velocidade e Behavior Tree ficam definidos no arquivo
de parametros do `nav_hub`.

O `sim_bot/launch/nav.launch.py` foi removido. O fluxo principal usa o launch
do `nav_hub`.

## slam.launch.py

Sobe o `slam_toolbox` para casos em que ainda nao existe mapa pronto.

No fluxo atual Palmares/Oregon, o uso normal e:

```text
slam:=False
nav:=True
```

## teleop.launch.py

Sobe teleoperacao por joystick:

```text
joy_node
teleop_twist_joy
```

Como o Nav2 publica diretamente em `/cmd_vel`, evite usar teleoperacao e Nav2
ao mesmo tempo. Para navegacao autonoma, use:

```text
joy:=False
```
