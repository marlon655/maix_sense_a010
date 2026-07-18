# nav_hub

Pacote ROS 2 que concentra a parte de navegacao baseada no projeto Oregon.

No fluxo atual, o `nav_hub` e usado pelo simulador `sim_bot` como a camada de
navegacao. O `sim_bot` sobe Gazebo, robo, bridge e RViz; o `nav_hub` fornece
Nav2, mapa, grafo, Behavior Tree e speed filter.

Para esse fluxo funcionar, este pacote deve estar no mesmo workspace ROS 2 do
`sim_bot`, por exemplo:

```text
~/sim_ws/src/sim_bot
~/sim_ws/src/nav_hub
```

```text
sim_bot
  -> simulacao
  -> Gazebo
  -> robot_state_publisher
  -> spawn do robo
  -> ros_gz_bridge
  -> RViz

nav_hub
  -> navegacao
  -> Nav2 params
  -> mapa
  -> speed mask
  -> grafo
  -> Behavior Tree
  -> route_server
  -> speed_filter
```

## Integracao Com O sim_bot

O fluxo principal no simulador e:

```text
sim_bot/launch/sim_manager.launch.py
  -> sim_bot/launch/sim_essentials.launch.py
  -> sim_bot/launch/sim_navigation.launch.py
      -> nav_hub/launch/essentials.launch.py mode:=simulation
          -> nav_hub/launch/route_graph/sim_nav_graph.launch.py
```

Ou seja, o `sim_bot` nao chama mais um launch proprio de Nav2. Ele chama o
`essentials.launch.py` deste pacote em modo `simulation`.

No robo real, o mesmo arquivo pode ser usado em modo `hardware`:

```bash
ros2 launch nav_hub essentials.launch.py mode:=hardware
```

No modo `hardware`, ele preserva o fluxo original da Oregon:

```text
robot06_description/display.launch.py
ldlidar_stl_ros2/stl27l.launch.py
canopen_ros/canopen.launch.py
nav_hub/launch/route_graph/nav_graph.launch.py
```

No modo `simulation`, ele evita dependencias de hardware real e chama:

```text
nav_hub/launch/route_graph/sim_nav_graph.launch.py
```

O arquivo de parametros usado por esse launch e:

```text
nav_hub/config/sim_nav_params.yaml
```

## Arquivos Principais

### Launch De Simulacao

```text
launch/essentials.launch.py
```

Ponto de entrada comum para hardware e simulacao:

```text
mode:=hardware
  -> display do robo real
  -> lidar real
  -> canopen real
  -> route_graph/nav_graph.launch.py

mode:=simulation
  -> route_graph/sim_nav_graph.launch.py
```

Os defaults desse launch ficam em:

```text
config/launch_params.yaml
```

```text
launch/route_graph/sim_nav_graph.launch.py
```

Sobe a pilha Nav2 adaptada para Gazebo:

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

Esse launch e inspirado no `launch/route_graph/nav_graph.launch.py`, mas evita
dependencias de hardware real e usa `use_sim_time:=true`.

### Lifecycle Manager

O `sim_nav_graph.launch.py` usa um lifecycle manager unico, seguindo o padrao
mais proximo da Oregon:

```text
lifecycle_manager_navigation
  -> map_server
  -> speed_filter_mask_server
  -> speed_costmap_filter_info_server
  -> amcl
  -> controller_server
  -> smoother_server
  -> planner_server
  -> behavior_server
  -> route_server
  -> bt_navigator
  -> waypoint_follower
  -> velocity_smoother
```

Para esse modelo funcionar, mantenha:

```text
route:=true
speed_filter:=true
```

Assim todos os nodes esperados pelo lifecycle manager unico sao iniciados.

### Main Route Graph Simulado

```text
src/sim_main_route_graph.cpp
launch/route_graph/sim_main_route_graph.launch.py
```

Versao de simulacao do `main_route_graph`.

O arquivo original da Oregon continua preservado:

```text
src/main_route_graph.cpp
```

A versao `sim_` existe para testar o fluxo operacional sem depender dos
caminhos fixos e do ambiente real da Oregon. Ela recebe um destino por ID no
topico:

```text
destination
```

Busca esse ID no JSON configurado por `route_file` e publica o objetivo em:

```text
/goal_pose
```

Tambem publica status basico em:

```text
has_reached
navegando
```

Por padrao, a versao de simulacao nao publica `initialpose`. Isso evita que o
AMCL reposicione o robo para o ponto `initial` do JSON e tire o robo da pose
correta definida pelo Gazebo/parametros de navegacao. Se quiser testar esse
comportamento explicitamente:

```bash
ros2 launch nav_hub sim_main_route_graph.launch.py publish_initial_pose:=true
```

O launch de teste e:

```bash
ros2 launch nav_hub sim_main_route_graph.launch.py
```

Com um arquivo de destinos especifico:

```bash
ros2 launch nav_hub sim_main_route_graph.launch.py \
  route_file:=$PWD/src/nav_hub/route/route_graph/goals_aceleradoras.json
```

Para enviar um destino operacional:

```bash
ros2 topic pub --once /destination std_msgs/msg/Int32 "{data: 3}" \
  --qos-durability transient_local \
  --qos-reliability reliable
```

Tambem existe um painel no pacote `sim_rviz_plugins` que substitui esse
comando por botoes:

```bash
colcon build --packages-select nav_hub sim_rviz_plugins
source install/setup.bash
rviz2
```

No RViz2, abra:

```text
Panels -> Add New Panel -> sim_rviz_plugins/SimDestinationPanel
```

O painel mostra o titulo `Destinos` e botoes de `1` ate `18`. Ao clicar em um
botao, ele publica `std_msgs/msg/Int32` em `/destination` com QoS:

```cpp
rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local()
```

O QoS e necessario apenas no teste manual com `ros2 topic pub`. Na Oregon, o
destino e publicado por outro node do proprio sistema usando QoS compativel com
o `main_route_graph`. O comando manual do ROS 2 usa QoS padrao, entao precisa
declarar `transient_local` e `reliable` para combinar com o fluxo original.

Esse node nao substitui o `route_server`. Ele publica o `/goal_pose`; o
`bt_navigator`, usando a Behavior Tree de rota, continua chamando o
`route_server` para calcular o caminho no grafo.

### Parametros Nav2

```text
config/sim_nav_params.yaml
```

Define:

```text
BT Navigator
AMCL
map_server
costmaps
controller_server
planner_server
smoother_server
velocity_smoother
route_server
speed_filter
```

Tambem aponta para os arquivos de mapa, grafo e Behavior Tree:

```yaml
default_nav_to_pose_bt_xml: "$(find-pkg-share nav_hub)/btree/nav_on_route_graph_oregon.xml"
yaml_filename: "$(find-pkg-share nav_hub)/maps/aceleradora.yaml"
graph_filepath: "$(find-pkg-share nav_hub)/graphs/aceleradoras.json"
yaml_filename: "$(find-pkg-share nav_hub)/maps/aceleradora_speed_mask.yaml"
```

### Mapa E Mascara De Velocidade

```text
maps/aceleradora.yaml
maps/aceleradora.pgm
maps/aceleradora_speed_mask.yaml
maps/aceleradora_speed_mask.pgm
```

O mapa e usado pelo `map_server`.

A speed mask e usada pelo speed filter do Nav2 para limitar velocidade em regioes
especificas do mapa.

### Grafo De Rotas

```text
graphs/aceleradoras.json
```

Grafo usado pelo `nav2_route/route_server`.

### Destinos Operacionais

```text
route/route_graph/goals_aceleradoras.json
```

Arquivo usado pelo `sim_main_route_graph` para converter um ID operacional em
um `/goal_pose`.

Esse arquivo nao e o grafo do `route_server`. A separacao atual e:

```text
graphs/aceleradoras.json
  -> grafo de navegacao usado pelo route_server

route/route_graph/goals_aceleradoras.json
  -> lista de destinos por ID usada pelo sim_main_route_graph
```

A pasta `route/route_graph` foi limpa para manter apenas o arquivo de destinos
da aceleradora e o script auxiliar `graph_to_goals.py`. Os JSONs antigos da
Oregon e arquivos de teste foram removidos desta versao do pacote para evitar
confusao durante a simulacao.

O arquivo original `src/main_route_graph.cpp` ainda aponta para caminhos da
Oregon. Para simulacao, use `src/sim_main_route_graph.cpp`.

Esse arquivo define os nos e arestas que o robo deve seguir quando o Behavior
Tree usa rota por grafo.

### Behavior Tree

```text
btree/nav_on_route_graph_oregon.xml
```

Behavior Tree usada pelo `bt_navigator` para calcular rota pelo grafo, suavizar
o caminho e mandar o controller seguir.

Fluxo esperado:

```text
RViz 2D Goal Pose
  -> bt_navigator
  -> ComputeRoute
  -> route_server
  -> graphs/aceleradoras.json
  -> SmoothPath
  -> FollowPath
  -> /cmd_vel
```

## Fluxo De Velocidade

O fluxo atual segue o padrao usado na Oregon, passando pelo
`velocity_smoother`:

```text
controller_server
  -> /controller_cmd_vel
  -> velocity_smoother
  -> /cmd_vel
  -> sim_bot/Gazebo
```

No launch:

```text
controller_server
  cmd_vel -> controller_cmd_vel

velocity_smoother
  cmd_vel -> controller_cmd_vel
  cmd_vel_smoothed -> cmd_vel
```

Assim, o controller nao publica direto no comando final do robo. O comando final
em `/cmd_vel` vem do `velocity_smoother`.

## Como Compilar

Na raiz do workspace:

```bash
cd ~/sim_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select nav_hub sim_bot
source install/setup.bash
```

Esse build assume que os dois pacotes estao no mesmo workspace:

```text
src/sim_bot
src/nav_hub
```

## Como Executar Pelo sim_bot

O uso normal deste pacote e por meio do `sim_bot`:

```bash
cd ~/sim_ws
source install/setup.bash
ros2 launch sim_bot sim_manager.launch.py
```

Esse comando sobe o Gazebo pelo `sim_bot` e a navegacao pelo `nav_hub`.

## Como Testar O Launch Do nav_hub Isolado

Para listar os argumentos do launch de navegacao:

```bash
cd ~/sim_ws
source install/setup.bash
ros2 launch ~/sim_ws/install/nav_hub/share/nav_hub/launch/route_graph/sim_nav_graph.launch.py --show-args
```

Observacao: esse launch isolado sobe apenas a navegacao. Para o robo existir no
Gazebo, use o fluxo completo pelo `sim_bot`.

## Visualizar O Grafo No RViz

O visualizador fica no pacote `sim_bot`, mas usa o grafo deste pacote:

```bash
cd ~/sim_ws
source install/setup.bash
ros2 run sim_bot graph_visualizer
```

No RViz:

```text
Add -> By topic -> /route_graph_markers -> MarkerArray
```

Por padrao, o visualizador usa:

```text
nav_hub/graphs/aceleradoras.json
```

## Testar Rota Por ID

O script tambem fica no `sim_bot`, mas chama o `route_server` configurado pelo
`nav_hub`:

```bash
cd ~/sim_ws
source install/setup.bash

ros2 run sim_bot route_to_poses --ros-args \
  -p use_start:=False \
  -p start_id:=1 \
  -p goal_id:=17
```

Esse teste chama:

```text
/compute_route
/follow_path
```

## Estrutura Relevante

```text
nav_hub/
  launch/
    route_graph/
      sim_nav_graph.launch.py   Launch Nav2 adaptado para simulacao.
      nav_graph.launch.py       Launch original/base da Oregon.

  config/
    sim_nav_params.yaml         Parametros Nav2 usados na simulacao.
    nav_routegraph.yaml         Referencia da Oregon.

  maps/
    aceleradora.yaml
    aceleradora.pgm
    aceleradora_speed_mask.yaml
    aceleradora_speed_mask.pgm

  graphs/
    aceleradoras.json

  btree/
    nav_on_route_graph_oregon.xml

  src/
    main_route_graph.cpp
    botoeira_ponto_linear.cpp
    stop_obstacle.cpp
    outros nos de referencia/importacao da Oregon
```

## O Que Ainda Nao Esta Integrado Na Simulacao

Alguns arquivos vieram da estrutura real da Oregon, mas ainda nao sao usados no
fluxo principal da simulacao:

```text
main_route_graph
botoeira_ponto_linear
collision_monitor
controller custom
ground_segmentation_ros2
launches de hardware real
ToF real
canopen real
ldlidar real
```

Esses itens devem ser migrados por etapas, sempre mantendo o fluxo do simulador
funcionando.

## Ordem Recomendada Para Evolucao

1. Manter `sim_nav_graph.launch.py` funcionando como baseline.
2. Comparar `sim_nav_graph.launch.py` com `nav_graph.launch.py`.
3. Avaliar integracao do `main_route_graph`.
4. Avaliar `collision_monitor`.
5. Avaliar controller custom.
6. Avaliar `ground_segmentation_ros2` apenas se houver sensor/pointcloud que
   justifique isso na simulacao.
