# Sequencia de start do ToF

Este guia resume os comandos para iniciar o ToF na simulacao e no robo real.

## Simulacao com sim_bot

Fluxo:

```text
Gazebo/sim_bot -> /cloud
tof_sim.launch.py -> /tof/obstacle_points
Nav2 local_costmap -> STVL
```

No estado atual da simulacao, o `local_costmap` esta usando apenas:

```yaml
plugins: ["stvl_layer", "inflation_layer"]
```

Ou seja, o lidar 2D `/scan` nao entra no `local_costmap`; os obstaculos vem do ToF.

### Opcao A: mapa normal da aceleradora

### Terminal 1: simulador, bridge, RViz e Nav2

```bash
cd ~/sim_ws
source setup_sim.bash
ros2 launch sim_bot sim_manager.launch.py
```

### Opcao B: circuito isolado de testes ToF

Use quando quiser testar deteccao e costmap em um mundo separado, sem misturar
com o mapa da aceleradora. Este comando usa o Nav2 completo, mas troca o mapa
para um mapa vazio proprio do circuito:

```bash
cd ~/sim_ws
source setup_sim.bash
ros2 launch sim_bot sim_manager.launch.py \
  world:=/home/marlon/sim_ws/sim_bot/worlds/tof_test_circuit.world \
  spawn_x:=0.0 \
  spawn_y:=0.0 \
  spawn_z:=0.15 \
  nav:=true \
  slam:=false \
  nav_params_file:=/home/marlon/sim_ws/nav_hub/config/sim_nav_params_tof_circuit.yaml
```

O circuito usa um corredor de 1,80 m de largura, pensando no `palmares_bot`
com corpo aproximado de 0,53 x 0,46 m. Os alvos ficam separados ao longo do
eixo X para testar cada caso de forma mais limpa:

```text
Faixa central de obstaculos:
x=1.20  guia baixa de 3 cm
x=2.35  cubo de 10 cm
x=3.60  caixa de 30 cm
x=4.90  painel alto
x=6.20  painel largo
x=7.65  rampa de 10 graus
x=9.20  painel final

Faixa lateral de rampas, em y=3.00:
x=1.40  rampa 5 graus, baixa
x=3.00  rampa 10 graus, media
x=5.65  rampa 15 graus, 1,20 m de largura, subida + plato de 60 cm + descida
x=8.15  rampa 20 graus, inclinada
```

Neste perfil, o spawn do Gazebo e a pose inicial do AMCL ficam alinhados em:

```text
x=0.0
y=0.0
yaw=0.0
base_frame: base_footprint
```

### Terminal 2: pre-processador ToF em modo simulacao

```bash
cd ~/sim_ws
source setup_sim.bash
ros2 launch tof_stvl_robot tof_sim.launch.py
```

O `tof_sim.launch.py` usa:

```text
simulation: true
driver A010: desligado
input_topic: /cloud
target_frame: base_footprint
output_frame: tof_lidar_frame
temporal_required_frames: 1
debug clouds: ligado
```

### Terminal 3: grafo de rotas

Use apenas se for navegar por `/destination`:

```bash
cd ~/sim_ws
source setup_sim.bash
ros2 launch nav_hub sim_main_route_graph.launch.py
```

### Terminal opcional: obstaculos de teste do ToF

Use para inserir objetos de varios tamanhos no Gazebo e validar o que o ToF
consegue marcar no RViz/costmap:

```bash
cd ~/sim_ws
source setup_sim.bash
ros2 launch sim_bot tof_test_obstacles.launch.py
```

O launch insere:

```text
tof_low_curb_3cm    50 x 8 x 3 cm
tof_cube_10cm       10 x 10 x 10 cm
tof_box_30cm        30 x 30 x 30 cm
tof_tall_panel      8 x 60 x 80 cm
tof_wide_panel      10 x 100 x 50 cm
tof_ramp_10deg      rampa/plano inclinado de 10 graus
```

O `tof_low_curb_3cm` pode ficar abaixo de `height_min: 0.04`, entao ele serve
justamente para confirmar o limite inferior do filtro. A rampa pode aparecer
parcialmente, porque o filtro atual foi feito para obstaculos, nao para
classificar inclinacao de piso.

### Terminal 4: validacao

```bash
cd ~/sim_ws
source setup_sim.bash

ros2 topic info /tof/obstacle_points
ros2 topic echo /tof/obstacle_points --once --field width
ros2 topic list | grep tof_filters
```

Resultado esperado quando o STVL estiver conectado:

```text
Publisher count: 1
Subscription count: 2
```

Um subscriber normalmente e o RViz, e o outro e o `local_costmap` via STVL.

Para mandar um destino pelo grafo:

```bash
ros2 topic pub --once /destination std_msgs/msg/Int32 "{data: 99}" \
  --qos-reliability reliable \
  --qos-durability transient_local
```

## Robo real

Fluxo:

```text
A010 fisico -> /cloud
tof_real.launch.py -> /tof/obstacle_points
Nav2 local_costmap -> STVL
```

Antes de iniciar, o robo real precisa fornecer:

```text
/dev/tof
odom -> base_footprint
base_footprint -> tof
Nav2/STVL consumindo /tof/obstacle_points
```

### Terminal 1: bringup oficial do robo

Inicie primeiro o bringup normal do robo real, ou seja, odometria, TF, Nav2 e
os drivers principais do robo.

### Terminal 2: ToF real

```bash
cd ~/sim_ws
source setup_sim.bash
ros2 launch tof_stvl_robot tof_real.launch.py
```

O `tof_real.launch.py` usa:

```text
simulation: false
driver A010: ligado
input_topic: /cloud
target_frame: base_footprint
output_frame: tof
temporal_required_frames: 1
debug clouds: desligado
```

### Validacao no robo real

```bash
ros2 topic hz /cloud
ros2 topic hz /tof/obstacle_points
ros2 topic info /tof/obstacle_points
ros2 topic echo /tof/obstacle_points --once --field header
ros2 run tf2_ros tf2_echo base_footprint tof
```

O STVL do Nav2 real deve ter uma fonte apontando para:

```yaml
tof_obstacles:
  topic: /tof/obstacle_points
  data_type: PointCloud2
  marking: true
  clearing: false
```

O nome `tof_obstacles` pode ser outro, mas precisa bater com
`observation_sources`.
