# MaixSense A010 para ROS 2 Jazzy

Este repositorio fornece dois pacotes para uso do MaixSense A010 no robo real:

- `sipeed_tof_ms_a010`: driver serial, `/depth` e `/cloud`;
- `tof_stvl_robot`: pre-processamento e publicacao da nuvem filtrada.

O repositorio nao inicia Nav2, costmaps, STVL, odometria, URDF ou TFs.

## Uso no robo real com workspace separado

Arquitetura:

```text
tof_robot_ws                                      robot_ws
MaixSense A010                                    odometria real
  -> sipeed_tof_node                                -> odom -> base_footprint
  -> /cloud                                       URDF oficial
  -> tof_pointcloud_preprocessor                    -> base_footprint -> tof
  -> /ground_segmentation/obstacle_points         Nav2 oficial
                                                    -> STVL/local costmap
```

| Responsabilidade | tof_robot_ws | robot_ws |
|---|---:|---:|
| Driver A010 | sim | nao |
| Pre-processamento | sim | nao |
| Odometria | nao | sim |
| URDF/TF do sensor | nao | sim |
| STVL | nao | sim |
| Local costmap | nao | sim |
| Nav2 | nao | sim |

### Instalacao e compilacao

```bash
mkdir -p ~/tof_robot_ws/src
cd ~/tof_robot_ws/src

git clone \
  --branch tof_robot \
  --single-branch \
  https://github.com/marlon655/maix_sense_a010.git

cd ~/tof_robot_ws
source /opt/ros/jazzy/setup.bash

rosdep install --from-paths src --ignore-src -r -y

colcon build \
  --packages-select sipeed_tof_ms_a010 tof_stvl_robot \
  --symlink-install
```

### Ordem dos overlays e execucao

Use sempre esta ordem:

```text
ROS 2 Jazzy -> robot_ws -> tof_robot_ws
```

Com o bringup oficial do robo ativo em outro terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/robot_ws/install/setup.bash
source ~/tof_robot_ws/install/setup.bash

ros2 launch tof_stvl_robot tof_robot_bringup.launch.py \
  device:=/dev/tof
```

O launch inicia exclusivamente:

- `sipeed_tof_ms_a010/sipeed_tof_node`;
- `tof_stvl_robot/pointcloud_preprocessor`.

### TFs obrigatorias

O `robot_ws` deve fornecer:

```text
odom -> base_footprint       dinamica, publicada pela odometria
base_footprint -> tof        fixa, publicada pelo URDF oficial
```

Valide:

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_footprint tof
ros2 run tf2_tools view_frames
```

A altura fisica aproximada de `0,22 m` pertence ao URDF. `height_min` e
`height_max` sao limites de obstaculos e nao corrigem a pose do sensor.

### Integracao com Nav2

O Nav2 oficial deve consumir somente:

```text
/ground_segmentation/obstacle_points
```

Use `tof_stvl_robot/config/nav2_stvl_a010_example.yaml` apenas como referencia
para mesclar a fonte ao STVL existente. Nao crie um segundo local costmap e
nunca adicione `/tof_filters/*` a `observation_sources`.

### Debug

O perfil de producao desliga por padrao:

```yaml
publish_intermediate_clouds: false
publish_filter_bounds: false
publish_projected_wall: false
```

Para diagnostico, altere as flags no YAML e reinicie o no. Quando habilitados,
ficam disponiveis `/tof_filters/distance`, `/height`, `/lateral`, `/spatial`,
`/filter_bounds` e `/projected_wall`. A nuvem final permanece ativa em ambos os
perfis.

### Validacao rapida

```bash
ros2 node list
ros2 topic hz /cloud
ros2 topic hz /ground_segmentation/obstacle_points
ros2 topic echo /ground_segmentation/obstacle_points --once --field header
ros2 param dump /tof_pointcloud_preprocessor
```

### Problemas comuns

- `/cloud` ausente: confira driver, dispositivo e permissao serial.
- `/dev/tof` ausente: localize a interface real em `/dev/ttyUSB*`.
- permissao negada: confira o grupo `dialout` e refaca login.
- TF `base_footprint -> tof` ausente: corrija o URDF/bringup oficial.
- TF `odom -> base_footprint` ausente: inicie a odometria real.
- topico final sem subscriber: confirme o YAML efetivo do Nav2.
- `/tof_filters/*` ausentes: o debug vem desligado em producao.
- duas autoridades TF: remova o publicador duplicado.
- pacote nao encontrado: respeite a ordem dos overlays.

## Documentacao completa

Veja [instalacao e validacao do tof_robot_ws](docs/tof_robot_ws_installation.md)
para o procedimento desde zero, dispositivo serial, testes de hardware,
integracao STVL e checklist de seguranca.

Detalhes do driver permanecem em
[src/sipeed_tof_ms_a010/USAGE.md](src/sipeed_tof_ms_a010/USAGE.md).
