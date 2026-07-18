# Sim Bot

Pacote ROS 2 para simular no Gazebo qualquer robo declarado no perfil de
launch. A navegacao da simulacao ainda segue o modelo adaptado da Oregon por
meio do pacote `nav_hub`.

O fluxo atual separa as responsabilidades assim:

```text
sim_bot
  -> Gazebo
  -> robot_state_publisher
  -> spawn do robo
  -> ros_gz_bridge
  -> RViz
  -> joystick opcional
  -> scripts auxiliares de teste

nav_hub
  -> parametros Nav2
  -> mapa
  -> speed mask
  -> grafo
  -> Behavior Tree
  -> launch de navegacao da simulacao
```

## Requisitos

```text
Ubuntu 24.04
ROS 2 Jazzy
Gazebo Sim 8
```

Sempre carregue o ROS 2 antes de compilar ou executar:

```bash
source /opt/ros/jazzy/setup.bash
```

## Compilar

Na raiz do workspace:

```bash
cd ~/sim_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select nav_hub sim_bot
source install/setup.bash
```

Se for compilar tudo:

```bash
colcon build
source install/setup.bash
```

## Fluxo De Launch

O ponto de entrada principal e:

```text
sim_bot/launch/sim_manager.launch.py
```

Fluxo executado:

```text
sim_manager.launch.py
  -> sim_essentials.launch.py
  -> sim_navigation.launch.py
      -> nav_hub/launch/route_graph/sim_nav_graph.launch.py
```

`sim_manager.launch.py` le os valores padrao de:

```text
sim_bot/config/launch_params.yaml
```

O arquivo atual:

```yaml
mode: simulation

world: worlds/aceleradora.world
robot_name: palmares_bot
robot_urdf: description/palmares_bot.urdf.xacro
spawn_x: 18.34406852722168
spawn_y: 22.93574333190918
spawn_z: 0.15
bridge_config: config/gz_bridge.yaml
rviz_config: rviz/bot.rviz
rviz: true
joy: false

slam: false
nav: true
route: true
speed_filter: true
```

Neste exemplo, o perfil usa o Xacro `palmares_bot.urdf.xacro`, mas o simulador
nao e fixo nesse robo. Para testar outro robo, altere principalmente:

```yaml
robot_name: nome_do_robo
robot_urdf: description/outro_robo.urdf.xacro
```

Os caminhos nesse arquivo podem ser absolutos ou relativos ao pacote `sim_bot`.

## Executar Simulacao

Em um terminal:

```bash
cd ~/sim_ws
source install/setup.bash
ros2 launch sim_bot sim_manager.launch.py
```

Esse comando sobe:

```text
Gazebo
robot_state_publisher
spawn do robo configurado em robot_name/robot_urdf
ros_gz_bridge
RViz
Nav2
AMCL
route_server
speed_filter
```

Tambem e possivel sobrescrever parametros no terminal:

```bash
ros2 launch sim_bot sim_manager.launch.py rviz:=False
```

```bash
ros2 launch sim_bot sim_manager.launch.py joy:=True nav:=False
```

```bash
ros2 launch sim_bot sim_manager.launch.py headless:=True
```

## Arquivos De Navegacao

A navegacao da simulacao fica centralizada no `nav_hub`.

Arquivos principais:

```text
nav_hub/launch/essentials.launch.py
nav_hub/launch/route_graph/sim_nav_graph.launch.py
nav_hub/config/sim_nav_params.yaml
nav_hub/maps/aceleradora.yaml
nav_hub/maps/aceleradora.pgm
nav_hub/maps/aceleradora_speed_mask.yaml
nav_hub/maps/aceleradora_speed_mask.pgm
nav_hub/graphs/aceleradoras.json
nav_hub/btree/nav_on_route_graph_oregon.xml
```

O `sim_bot` chama `nav_hub/launch/essentials.launch.py` com
`mode:=simulation`. Esse launch, por sua vez, chama
`sim_nav_graph.launch.py`, que sobe a pilha Nav2 para simulacao:

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
route_server
speed_filter_mask_server
speed_costmap_filter_info_server
lifecycle managers
```

O `sim_bot` nao mantem mais copia local de mapa, grafo, speed mask ou Behavior
Tree. Esses arquivos ficam no `nav_hub`.

## Teste Pelo RViz

Depois de subir:

```bash
ros2 launch sim_bot sim_manager.launch.py
```

No RViz:

```text
1. Confira se o mapa aparece.
2. Confira se o robo aparece no mapa.
3. Use 2D Pose Estimate se precisar ajustar a pose inicial.
4. Use 2D Goal Pose para enviar um destino.
```

Fluxo esperado ao enviar um goal:

```text
RViz
  -> bt_navigator
  -> nav_hub/btree/nav_on_route_graph_oregon.xml
  -> ComputeRoute
  -> route_server
  -> nav_hub/graphs/aceleradoras.json
  -> SmoothPath
  -> FollowPath
  -> /cmd_vel
  -> Gazebo
```

Fluxo de velocidade:

```text
controller_server
  -> /controller_cmd_vel
  -> velocity_smoother
  -> /cmd_vel
  -> ros_gz_bridge
  -> Gazebo
```

Esse remap aproxima a simulacao do padrao usado na Oregon: o controller nao
publica diretamente no topico final do robo, ele passa primeiro pelo
`velocity_smoother`.

Se o goal for aceito, o robo deve seguir o caminho do grafo. Se houver obstaculo
sobre a rota, o comportamento esperado e reduzir/parar/abortar conforme os
parametros de costmap, controller e collision checks.

## Visualizar O Grafo

Em outro terminal:

```bash
cd ~/sim_ws
source install/setup.bash
ros2 run sim_bot graph_visualizer
```

No RViz:

```text
Add -> By topic -> /route_graph_markers -> MarkerArray
```

O visualizador publica:

```text
linhas azuis: arestas do grafo
esferas amarelas: nos do grafo
texto branco: ID dos nos
```

Por padrao, o script usa:

```text
nav_hub/graphs/aceleradoras.json
```

Para usar outro grafo:

```bash
ros2 run sim_bot graph_visualizer --ros-args \
  -p graph_file:=$PWD/src/nav_hub/graphs/aceleradoras.json
```

## Ir De Um Ponto A Outro Pelo Grafo

O script `route_to_poses` chama o `route_server`, calcula uma rota por IDs do
grafo e envia o caminho para o Nav2 seguir.

Exemplo:

```bash
cd ~/sim_ws
source install/setup.bash

ros2 run sim_bot route_to_poses --ros-args \
  -p use_start:=False \
  -p start_id:=1 \
  -p goal_id:=17
```

Parametros principais:

```text
start_id       ID inicial no grafo.
goal_id        ID final no grafo.
use_start      Se False, usa start_id. Se True, o route_server pode usar a pose atual.
use_poses      Mantido False para rota por IDs.
mode           follow_path ou navigate_through_poses.
max_poses      Limita a quantidade de poses enviadas. 0 nao limita.
```

Modos:

```bash
ros2 run sim_bot route_to_poses --ros-args \
  -p use_start:=False \
  -p start_id:=1 \
  -p goal_id:=17 \
  -p mode:=follow_path
```

```bash
ros2 run sim_bot route_to_poses --ros-args \
  -p use_start:=False \
  -p start_id:=1 \
  -p goal_id:=17 \
  -p mode:=navigate_through_poses
```

Para esse teste ficar coerente, o robo precisa estar perto do no inicial usado
em `start_id`, ou a rota pode tentar comecar de um ponto distante da pose real.

## Testes Recomendados

### 1. Launch Principal

```bash
ros2 launch sim_bot sim_manager.launch.py
```

Validar:

```text
Gazebo abriu
RViz abriu
mapa apareceu
robo apareceu
AMCL ativo
Nav2 ativo
route_server ativo
speed_filter ativo
```

### 2. Goal Pelo RViz

Enviar `2D Goal Pose`.

Validar:

```text
robo se move
rota segue o grafo
nao aparece erro de BT XML
nao aparece erro de mapa
nao aparece erro de speed mask
/controller_cmd_vel recebe comandos do controller
/cmd_vel recebe comandos suavizados
```

### 3. Grafo No RViz

```bash
ros2 run sim_bot graph_visualizer
```

Validar:

```text
/route_graph_markers aparece no RViz
nos e arestas ficam alinhados ao mapa
IDs aparecem corretamente
```

### 4. Rota Por ID

```bash
ros2 run sim_bot route_to_poses --ros-args \
  -p use_start:=False \
  -p start_id:=1 \
  -p goal_id:=17
```

Validar:

```text
route_server retorna caminho
FollowPath recebe o caminho
robo executa a rota
```

### 5. Speed Filter

Enviar o robo para passar pela area pintada da speed mask.

Validar:

```text
velocidade reduz na area marcada
fora da area, velocidade volta ao normal
```

## Estrutura Atual Do Pacote

```text
sim_bot/
  config/
    launch_params.yaml     Perfil principal da simulacao.
    gz_bridge.yaml         Bridge ROS 2 <-> Gazebo.
    joy_params.yaml        Parametros do joystick.
    slam_params.yaml       Parametros do slam_toolbox.
    nav_params.yaml        Referencia antiga/base simples de Nav2.

  description/
    palmares_bot.urdf.xacro
    palmares_lidar.xacro
    common.xacro
    sensores auxiliares

  launch/
    sim_manager.launch.py      Orquestrador principal.
    sim_essentials.launch.py   Gazebo, spawn, bridge, RViz e robot_state_publisher.
    sim_navigation.launch.py   Chama SLAM e/ou nav_hub sim_nav_graph.
    slam.launch.py             SLAM opcional.
    teleop.launch.py           Joystick opcional.

  scripts/
    graph_visualizer.py    Publica markers do grafo no RViz.
    route_to_poses.py      Calcula rota por ID e envia ao Nav2.

  worlds/
    aceleradora.world

  models/
    aceleradora_world/

  rviz/
    bot.rviz
```

Estrutura usada do `nav_hub`:

```text
nav_hub/
  launch/route_graph/sim_nav_graph.launch.py
  config/sim_nav_params.yaml
  maps/aceleradora.yaml
  maps/aceleradora.pgm
  maps/aceleradora_speed_mask.yaml
  maps/aceleradora_speed_mask.pgm
  graphs/aceleradoras.json
  btree/nav_on_route_graph_oregon.xml
```

## Observacoes

- O comando principal deve ser executado pelo `sim_manager.launch.py`.
- `config/launch_params.yaml` controla o perfil padrao da simulacao.
- Mapa, grafo, speed mask e BT ficam no `nav_hub`.
- O `sim_bot` e a camada de simulacao; o `nav_hub` e a camada de navegacao.
- Para Nav2, mantenha `joy: false` para evitar comando manual e autonomo ao
  mesmo tempo em `/cmd_vel`.
- Se alterar arquivos de launch, config ou mapa, recompile e rode `source`:

```bash
cd ~/sim_ws
colcon build --packages-select nav_hub sim_bot
source install/setup.bash
```
