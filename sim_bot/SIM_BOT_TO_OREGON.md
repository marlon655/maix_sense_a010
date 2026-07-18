# Sim Bot To Oregon

Este documento organiza as etapas para adaptar a simulacao atual do `sim_bot`
para usar configuracoes inspiradas no projeto Oregon.

O arquivo principal de teste no fluxo atual e:

```bash
src/nav_hub/config/sim_nav_params.yaml
```

Ele foi criado a partir do `src/sim_bot/config/oregon_nav_params.yaml` usado
nos primeiros testes, mas agora fica no pacote `nav_hub` para aproximar a
simulacao da estrutura real da Oregon.

## 1. Confirmar o Frame Base do Robo

Antes de testar os parametros da Oregon, confirme qual frame o robo usa no Xacro:

```text
base_link
base_footprint
```

O arquivo `oregon_nav_params.yaml` esta usando `base_link`.

Se o Xacro usar `base_footprint`, ajuste todos os pontos equivalentes:

```text
bt_navigator.robot_base_frame
amcl.base_frame_id
local_costmap.robot_base_frame
global_costmap.robot_base_frame
behavior_server.robot_base_frame
collision_monitor.base_frame_id, se for ativado depois
```

Comando util para conferir TF:

```bash
ros2 run tf2_tools view_frames
```

## 2. Testar Primeiro com Plugins Padrao

Primeiro teste apenas os plugins comuns do Nav2 que ja funcionam no simulador:

```text
DWBLocalPlanner
NavfnPlanner
ObstacleLayer
InflationLayer
StaticLayer
SimpleSmoother
AMCL
```

Comando de teste:

```bash
ros2 launch sim_bot sim_manager.launch.py \
  world:=$PWD/src/sim_bot/worlds/aceleradora.world \
  slam:=False \
  nav:=True \
  rviz:=True \
  joy:=False
```

Se esse teste nao rodar limpo, nao avance para os plugins especificos da Oregon.

## 3. Ajustar o Footprint

O simulador esta usando raio simples:

```yaml
robot_radius: 0.35
```

A Oregon usava um footprint com o formato real do robo:

```yaml
footprint: "[[0.26, -0.25], [0.26, 0.25], [-0.26, 0.25], [-0.26, 0.085], [-0.54, 0.085], [-0.54, -0.085], [-0.26, -0.085], [-0.26, -0.25]]"
```

Para usar o footprint, remova ou comente o `robot_radius` e ative o
`footprint` em:

```text
local_costmap
global_costmap
```

## 4. Ajustar Velocidades

O simulador usa velocidades conservadoras:

```yaml
max_vel_x: 0.26
max_vel_theta: 1.0
```

A Oregon usava valores mais agressivos em alguns controladores, por exemplo:

```yaml
desired_linear_vel: 0.7
```

Suba as velocidades aos poucos. Primeiro valide navegacao lenta, depois ajuste
para se aproximar do comportamento da Oregon.

## 5. Avaliar o Controller da Oregon

A Oregon usava um controller customizado:

```yaml
nav2_regulated_pure_pursuit_controller::CustomRegulatedPurePursuitController
```

Antes de ativar, confirme se esse plugin existe no ambiente:

```bash
ros2 pkg list | grep pure
ros2 pkg prefix nav2_regulated_pure_pursuit_controller
```

Se o plugin customizado nao existir, mantenha o controller padrao do simulador:

```yaml
dwb_core::DWBLocalPlanner
```

## 6. Avaliar o Route Server

A Oregon usava navegacao por grafo:

```text
route_server
graph_filepath
nav2_route
```

Para ativar isso no simulador, sera necessario:

```text
ter o pacote nav2_route instalado ou compilado
copiar/adaptar o arquivo .json do grafo para o sim_bot
adaptar um launch para iniciar o route_server
usar uma Behavior Tree compativel com rota
```

Essa etapa deve ser feita depois que a navegacao basica com mapa estiver
funcionando.

## 7. Avaliar o Speed Filter

A Oregon usava filtro de velocidade no costmap:

```text
speed_filter_mask_server
speed_costmap_filter_info_server
speed_filter
```

Para usar no simulador, sera necessario:

```text
ter um mapa de mascara de velocidade .yaml/.pgm
iniciar os servidores do filtro
ativar filters: ["speed_filter"] no global_costmap
validar o topico /speed_limit
```

Deixe essa parte comentada ate o Nav2 basico estar estavel.

## 8. Deixar STVL e ToF para o Final

A Oregon tambem tinha configuracoes para sensor 3D/ToF:

```text
spatio_temporal_voxel_layer
/ground_segmentation/obstacle_points
PointCloud2
```

No simulador atual, o sensor principal e o LaserScan em:

```text
/scan
```

Por isso, mantenha STVL/ToF comentado ate existir uma simulacao real de sensor
3D ou uma ponte publicando `PointCloud2`.

## Comandos Uteis de Diagnostico

```bash
ros2 topic list
ros2 topic echo /scan --once
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo map base_link
ros2 lifecycle nodes
ros2 node list
```

## Sequencia Recomendada

```text
1. Rodar o oregon_nav_params.yaml como esta
2. Conferir /scan, /odom, /tf e /map
3. Ajustar base_link/base_footprint
4. Ajustar footprint
5. Ajustar velocidades
6. Testar controller Oregon, se o plugin existir
7. Testar route_server
8. Testar speed_filter ou ToF apenas no final
```

## Sequencia Atual Para Aproximar Da Oregon

Depois dos testes iniciais, a meta passou a ser deixar o simulador mais fiel ao
modelo da Oregon, mas em um formato generico para reaproveitar com outros
robos.

A ideia nao e copiar caminhos absolutos, nomes especificos ou dependencias do
NUC. O objetivo e manter a mesma arquitetura:

```text
robo simulado
  -> Nav2
  -> route_server
  -> grafo
  -> speed_filter
  -> camada tipo nav_hub / Behavior Tree
```

## Estado Atual Da Migracao

Ja foi validado no simulador:

```text
palmares_bot no Gazebo
mapa aceleradora com AMCL
oregon_nav_params.yaml
controller Regulated Pure Pursuit padrao
footprint do palmares
route_server com graph aceleradoras.json
route_to_poses chamando /compute_route e /follow_path
grafo sequencial e limpo
speed_filter ativo
mascara de velocidade alinhada ao mapa
area de reducao de velocidade na curva id5 -> id6 -> id7
Behavior Tree de rota por grafo adaptada para o sim_bot
2D Goal Pose do RViz seguindo o grafo
visualizacao do grafo com /route_graph_markers
```

O `route_to_poses.py` continua sendo uma ponte de teste. Ele prova que o grafo
e o controller funcionam de forma direta.

A aproximacao mais fiel da Oregon agora passa pela BT:

```text
btree/nav_on_route_graph_sim.xml
```

Ela permite que o `2D Goal Pose` do RViz chame o `route_server`, use o grafo e
depois envie o caminho suavizado para o controller.

O fluxo ja foi validado:

```text
RViz 2D Goal Pose
  -> bt_navigator
  -> ComputeRoute
  -> route_server
  -> graphs/aceleradoras.json
  -> SmoothPath
  -> FollowPath
```

A BT foi ajustada para ficar mais parecida com a Oregon. A estrutura atual usa:

```text
NavigateWithRoutes
PlanningRecovery
ComputeFullRoute
EnsureGoalOnPath
SmoothPath
FollowPath
```

As diferenças mantidas sao especificas do simulador:

```text
global_frame=map
robot_base_frame=base_link
```

## Arquitetura Desejada

Formato desejado para ficar mais proximo da Oregon:

```text
RViz / cliente / operador
  -> camada de decisao de rota
  -> route_server
  -> rota no grafo
  -> Nav2 / controller
  -> speed_filter
  -> /cmd_vel
```

Na Oregon, essa camada de decisao provavelmente envolve:

```text
nav_hub
main_route_graph
Behavior Tree
btree/nav_on_route_graph_oregon.xml
```

No simulador, essa camada deve ser adaptada para algo generico, evitando
dependencias diretas de nomes como `oregon`, caminhos em `/home/ubuntu` ou
pacotes que nao fazem parte do `sim_bot`.

## Proximos Passos Para Um Modelo Generico

### 1. Mapear A Arquitetura Da Oregon

Objetivo:

```text
descobrir quem substitui o route_to_poses.py na Oregon
```

Procurar e documentar:

```text
nav_hub
main_route_graph
btree/nav_on_route_graph_oregon.xml
actions usadas
services usados
topicos de goal
como chama /compute_route
como entrega a rota para o Nav2
como trata falha e replanejamento
```

Resultado esperado:

```text
lista de arquivos importantes da Oregon
dependencias obrigatorias
dependencias que podem ser removidas
fluxo real entre goal, grafo e Nav2
```

### 2. Separar O Que E Generico Do Que E Especifico Do Robo

Generico:

```text
route_server
grafo .json
speed_filter
mascara de velocidade
costmap filters
cliente que transforma destino em rota
Behavior Tree de navegacao por grafo
```

Especifico do robo:

```text
URDF/Xacro
footprint
frames base_link/base_footprint
topicos de sensores
limites de velocidade
controller tuning
mapa e mascara usados no ambiente
```

O modelo final deve permitir trocar o robo mantendo a estrutura.

### 3. Evoluir O route_to_poses Para Um Node Mais Generico

Nome possivel:

```text
route_graph_navigator
sim_route_graph_navigator
```

Responsabilidades:

```text
receber um destino por ID ou pose
encontrar o no mais adequado do grafo
chamar /compute_route
enviar o path para /follow_path ou para uma action Nav2 equivalente
monitorar resultado
publicar logs claros
tratar falha simples
permitir reuso em outros robos
```

Esse node deve substituir o uso manual de:

```bash
ros2 run sim_bot route_to_poses --ros-args ...
```

### 4. Integrar Com Entrada De Goal

Opcoes de entrada:

```text
goal por ID do grafo
goal por pose clicada no RViz
goal por topico custom
goal por service/action propria
```

Primeiro passo recomendado:

```text
manter goal por ID
depois adicionar conversao de clique no RViz para no mais proximo do grafo
```

Isso deixa o uso mais pratico sem trazer toda a complexidade do `nav_hub` de
uma vez.

### 5. Adaptar Behavior Tree

Depois que grafo, route_server e speed_filter foram validados, foi criada uma
BT semelhante a:

```text
nav_on_route_graph_oregon.xml
```

Arquivo no simulador:

```text
btree/nav_on_route_graph_sim.xml
```

Objetivo da BT:

```text
orquestrar planejamento por grafo
acionar navegacao
reagir a falhas
permitir replanejamento
aproximar o simulador da arquitetura real
```

Fluxo desejado:

```text
RViz 2D Goal Pose
  -> bt_navigator
  -> ComputeRoute
  -> route_server
  -> grafo
  -> SmoothPath
  -> FollowPath
```

Esse modo exige:

```text
route:=True
speed_filter:=True, quando o YAML estiver com filters: ["speed_filter"]
```

O `route_to_poses.py` permanece como ferramenta de diagnostico para testar
`/compute_route` e `/follow_path` sem depender do RViz.

### 6. Remover Caminhos Absolutos E Nomes Fixos

Todo arquivo importado da Oregon precisa ser adaptado para usar:

```text
$(find-pkg-share sim_bot)
LaunchConfiguration
parametros YAML
nomes genericos
```

Evitar:

```text
/home/ubuntu/...
/home/ros_estudo/Downloads/...
nomes fixos de robo dentro de configs genericas
```

### 7. Documentar Cada Camada Validada

Manter dois documentos:

```text
SIM_BOT_TO_OREGON.md
OREGON_INTEGRATION.md
```

Uso recomendado:

```text
SIM_BOT_TO_OREGON.md      roteiro geral e estrategia
OREGON_INTEGRATION.md     detalhes tecnicos do que foi implementado
```

Quando uma etapa funcionar no simulador:

```text
documentar comando de teste
documentar arquivos alterados
documentar topicos/nos esperados
documentar diferencas em relacao a Oregon
documentar o que ficou generico para outros robos
```

## Ordem Recomendada A Partir De Agora

```text
1. Manter o baseline atual funcionando como referencia
2. Validar novamente a BT ajustada com 2D Goal Pose no RViz
3. Comparar diferencas restantes com nav_on_route_graph_oregon.xml
4. Manter route_to_poses.py e graph_visualizer como diagnostico
5. Parametrizar grafo/mapa/mascara/BT para virar modelo generico
6. Identificar dependencias ausentes do nav_hub/main_route_graph
7. Separar parametros genericos dos parametros especificos do Palmares
8. Manter speed_filter e grafo como recursos genericos configuraveis
```

## Upgrades Depois Da Parametrizacao

Depois que os argumentos de launch estiverem mais organizados e o fluxo puder
trocar mapa, grafo, mascara e parametros sem editar YAML manualmente, avaliar:

```text
nav_hub/main_route_graph
entrada por destino ID operacional
status de navegacao
rotas alternativas com custo por aresta
ToF/STVL
controller custom
```

Esses itens aproximam mais o simulador da operacao real da Oregon, mas devem
entrar depois que o baseline atual estiver facil de iniciar e reaproveitar.

Proximo foco imediato:

```text
melhorar e organizar os parametros de inicializacao dos launch files
```

## Parametros De Inicializacao Do Launch

Foi criado um arquivo para centralizar os valores padrao do
`sim_manager.launch.py`:

```text
config/launch_params.yaml
```

Com isso, o baseline Palmares/Oregon pode subir com:

```bash
ros2 launch sim_bot sim_manager.launch.py
```

Sem precisar repetir no terminal:

```text
mode
world
robot_name
robot_urdf
spawn_x
spawn_y
spawn_z
bridge_config
rviz_config
slam
nav
rviz
joy
route
speed_filter
```

Mapa, grafo de rota e mascara de velocidade ficam no arquivo de parametros
Nav2 usado no fluxo atual:

```text
nav_hub/config/sim_nav_params.yaml
  -> map_server.yaml_filename
  -> route_server.graph_filepath
  -> speed_filter_mask_server.yaml_filename
```

Assim o `sim_bot` nao precisa injetar esses caminhos pelo launch. Ele apenas
chama o `nav_hub` como pacote de navegacao e usa o arquivo de parametros
principal do proprio `nav_hub`.

O perfil foi separado em secoes comentadas:

```text
Mode
Simulation essentials
Navigation
```

Os campos de `Simulation essentials` controlam Gazebo, spawn do robo, xacro,
bridge e RViz. Os campos de `Navigation` controlam se SLAM, Nav2, route_server
e speed_filter sobem. Os caminhos internos de mapa/grafo/mascara ficam no YAML
do Nav2, para facilitar reaproveitar arquivos do robo real.

Essa abordagem deve ser usada como base para outros robos/clientes:

```text
criar ou ajustar um arquivo de parametros Nav2
apontar mapa, grafo e mascara dentro desse arquivo
manter o launch simples para o operador
```

## Camadas De Launch Da Simulacao

O `sim_manager.launch.py` foi reorganizado para se aproximar da separacao da
Oregon, mas mantendo o baseline Gazebo:

```text
launch/sim_manager.launch.py
  -> le config/launch_params.yaml
  -> se mode:=simulation
    -> launch/sim_essentials.launch.py
    -> launch/sim_navigation.launch.py
  -> se mode:=hardware
    -> modo reservado para integracao futura
```

Camada de essenciais da simulacao:

```text
launch/sim_essentials.launch.py
  -> Gazebo server/client
  -> robot_state_publisher
  -> spawn do palmares_bot
  -> ros_gz_bridge
  -> joystick opcional
  -> RViz opcional
```

Camada de navegacao da simulacao:

```text
launch/sim_navigation.launch.py
  -> slam.launch.py, se slam:=True
  -> nav_hub/launch/route_graph/sim_nav_graph.launch.py, se nav:=True
    -> map_server + AMCL
    -> Nav2
    -> route_server, se route:=True
    -> speed_filter, se speed_filter:=True
```

Regra importante:

```text
config/launch_params.yaml e lido pelo sim_manager.launch.py
sim_essentials.launch.py recebe argumentos do sim_manager.launch.py
sim_navigation.launch.py recebe argumentos do sim_manager.launch.py
rodar sim_navigation.launch.py direto usa apenas os defaults internos dele
```

Com isso, o comando principal continua o mesmo:

```bash
ros2 launch sim_bot sim_manager.launch.py
```

Mas a estrutura ficou preparada para, no futuro, trocar a camada de simulacao
por uma camada de hardware real.

## Arquitetura Oregon No Hardware

A estrutura observada na Oregon real esta separada em dois servicos principais:

```text
essentials.service
  -> essentials.sh
    -> essentials.launch.py (nav_hub)
      -> display.launch.py (robot06_description)
        -> robot06.urdf.xacro
        -> robot_state_publisher
        -> joint_state_publisher
      -> stl27l.launch.py (ldlidar_stl_ros2)
        -> ldlidar_node
      -> sipeed_tof_node (sipeed_tof_ms_a010_ros)
      -> canopen.launch.py (canopen_ros)
        -> motor_control_node
        -> odometry_node
      -> nav_graph.launch.py (nav_hub/launch/route_graph)
        -> config/nav_routegraph.yaml
        -> remappings
        -> composable_nodes
        -> ground_segmentation_ros2
        -> launch_amcl.launch.py
        -> lifecycle_nodes

navigation.service
  -> navigation.sh
    -> navigation.launch.py (nav_hub)
      -> feedback.launch.py (mobby_feedback)
        -> fitaled_node
        -> controla_som_node
      -> main_route_graph
      -> botoeira_ponto_ponto_linear
```

No simulador atual, a equivalencia ainda nao e completa. O objetivo agora e
manter funcionando primeiro:

```text
Gazebo
robot_state_publisher
bridge Gazebo/ROS
map_server + AMCL
Nav2
route_server
BT de rota por grafo
speed_filter
graph_visualizer
```

Depois que essa base estiver estavel, a arquitetura pode evoluir para uma
separacao parecida com a Oregon:

```text
launch de essenciais da simulacao
  -> robo, mundo, sensores simulados, bridge, TF, odometria

launch de navegacao
  -> Nav2, route_server, BT, speed_filter, clientes operacionais
```

## Perfil Simulation/Hardware Futuro

A ideia futura e usar o `config/launch_params.yaml` como perfil operacional.
Hoje ele representa o perfil de simulacao e ja possui um campo de modo:

```yaml
mode: simulation
```

ou:

```yaml
mode: hardware
```

Com `mode: simulation`, o launch usaria:

```text
Gazebo
world
mapa do simulador
grafo do simulador
mascara de velocidade do simulador
sensores simulados via xacro/Gazebo
bridge ros_gz
```

Com `mode: hardware`, o launch usaria:

```text
URDF/Xacro do robo real
driver real do lidar
driver real de ToF
canopen_ros
odometria real
mapa/grafo/mascara do ambiente real
feedback fisico, som, leds e botoeira
```

Essa mudanca deve ficar para depois. Por enquanto, a regra e:

```text
nao misturar hardware real no baseline da simulacao
validar primeiro o fluxo Gazebo/Oregon atual
deixar os nomes dos parametros genericos para facilitar a troca depois
```

## Decisao Atual Sobre AMCL E Ground Segmentation

Apesar da Oregon possuir `launch_amcl.launch.py` separado, no simulador atual o
AMCL permanece dentro do `nav_hub/launch/route_graph/sim_nav_graph.launch.py`.

Motivo:

```text
o simulador esta usando LaserScan 2D
map_server + AMCL + Nav2 ja estao funcionando
separar AMCL agora nao traz ganho imediato
```

O `ground_segmentation_ros2` tambem fica fora por enquanto.

Motivo:

```text
nao ha camera/sensor 3D ativo no robo atual
nao ha PointCloud2 sendo usado pelo Nav2 neste baseline
o foco atual e manter o grafo, AMCL, speed_filter e Nav2 estaveis
```

Reavaliar essa decisao somente depois de adicionar sensor 3D ou nuvem de pontos
no simulador.
