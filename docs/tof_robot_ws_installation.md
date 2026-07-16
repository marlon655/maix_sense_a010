# Instalacao e validacao do `tof_robot_ws`

Este guia instala o A010 em um workspace separado do Nav2 oficial. O launch
de producao inicia somente o driver e o pre-processador. O `robot_ws` continua
responsavel por odometria, URDF, TFs, STVL, costmaps e Nav2.

## 1. Arquitetura e responsabilidades

```text
tof_robot_ws
MaixSense A010 -> sipeed_tof_node -> /cloud
  -> tof_pointcloud_preprocessor
  -> /ground_segmentation/obstacle_points

robot_ws
odometria -> odom -> base_footprint
URDF      -> base_footprint -> tof
/ground_segmentation/obstacle_points -> STVL -> local costmap -> Nav2
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

## 2. Instalacao desde zero

```bash
mkdir -p ~/tof_robot_ws/src
cd ~/tof_robot_ws/src

git clone \
  --branch tof_robot \
  --single-branch \
  https://github.com/marlon655/maix_sense_a010.git

cd ~/tof_robot_ws
source /opt/ros/jazzy/setup.bash

rosdep install \
  --from-paths src \
  --ignore-src \
  -r \
  -y
```

O clone fica em `~/tof_robot_ws/src/maix_sense_a010`. O colcon percorre esse
repositorio e encontra os pacotes dentro da pasta `src` interna. Confirme antes
do build:

```bash
colcon list | grep -E \
  'sipeed_tof_ms_a010|tof_stvl_robot'
```

O resultado deve conter exatamente os dois nomes. Se a instalacao local do
colcon tiver sido configurada para nao percorrer subdiretorios, use uma destas
alternativas validas, sem copiar arquivos manualmente:

```bash
colcon list --base-paths ~/tof_robot_ws/src/maix_sense_a010/src
```

ou clone o repositorio como raiz do workspace e use seu `src` diretamente.
Nao prossiga se `colcon list` nao encontrar ambos os pacotes.

Compile somente o necessario:

```bash
cd ~/tof_robot_ws
source /opt/ros/jazzy/setup.bash

colcon build \
  --packages-select \
    sipeed_tof_ms_a010 \
    tof_stvl_robot \
  --symlink-install

source ~/tof_robot_ws/install/setup.bash

ros2 pkg prefix sipeed_tof_ms_a010
ros2 pkg prefix tof_stvl_robot
```

## 3. Ordem dos overlays

A ordem recomendada e:

```text
ROS 2 Jazzy -> robot_ws -> tof_robot_ws
```

No terminal da infraestrutura principal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/robot_ws/install/setup.bash

ros2 launch <pacote_do_robo> <bringup_oficial>.launch.py
```

Substitua os placeholders pelos nomes reais do robo.

Em outro terminal, para o A010:

```bash
source /opt/ros/jazzy/setup.bash
source ~/robot_ws/install/setup.bash
source ~/tof_robot_ws/install/setup.bash

ros2 launch tof_stvl_robot tof_robot_bringup.launch.py \
  device:=/dev/tof
```

O launch do A010 nao publica TF, URDF ou odometria. Ele depende das mensagens e
da arvore TF fornecidas pelo `robot_ws`.

## 4. Dispositivo serial

Localize a interface real:

```bash
ls -l /dev/tof /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
readlink -f /dev/tof
```

Para descobrir atributos de uma interface candidata:

```bash
udevadm info -q property -n /dev/ttyUSB0 | \
  grep -E \
  'ID_SERIAL|ID_MODEL|ID_VENDOR|ID_USB_INTERFACE_NUM|ID_PATH'
```

Crie uma regra udev para `/dev/tof` somente depois de obter os IDs reais do
hardware. Nao use IDs copiados de outro equipamento.

Em caso de permissao negada, confira:

```bash
groups
ls -l /dev/tof
```

Se necessario, adicione o usuario a `dialout` e refaca o login:

```bash
sudo usermod -aG dialout "$USER"
```

O arquivo `sipeed_tof_ms_a010/config/maixsense_params.yaml` e carregado pelo
launch. O argumento `device` sobrescreve apenas o dispositivo. Nao altere
`sensor_baud_code`, `sensor_unit`, `sensor_fps` ou `sensor_binn` sem validar a
necessidade e a compatibilidade com o driver.

## 5. Launch de producao

Argumentos disponiveis:

```bash
ros2 launch tof_stvl_robot tof_robot_bringup.launch.py --show-args
```

- `device`: `/dev/tof` por padrao;
- `driver_params_file`: YAML instalado do driver;
- `preprocessor_params_file`: YAML instalado do pre-processador;
- `log_level`: `info` por padrao.

O launch inicia exclusivamente:

```text
/sipeed_tof_ms_a010
/tof_pointcloud_preprocessor
```

Ele nao inicia TF, URDF, `robot_state_publisher`, costmap, STVL, Inflation
Layer, Lifecycle Manager, Nav2 ou simulacao. O driver ja possui watchdog e
reconexao serial; por isso o launch nao habilita `respawn` externo.

## 6. TFs externas obrigatorias

O pre-processador consome:

```text
odom -> base_footprint       dinamica, fornecida pela odometria real
base_footprint -> tof        fixa, fornecida pelo URDF oficial
```

Confirme:

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_footprint tof
ros2 run tf2_tools view_frames
```

`odom -> base_footprint` deve mudar quando o robo se move.
`base_footprint -> tof` deve permanecer fixa e ter autoridade unica. O centro
optico medido fica aproximadamente a `0,22 m`, mas a fonte correta da pose e o
URDF/Xacro oficial.

Nao confunda essa altura fisica com:

```yaml
height_min: 0.04
height_max: 1.20
```

Esses parametros apenas selecionam a altura dos obstaculos no
`base_footprint`.

Se uma TF estiver temporariamente ausente, o no registra warning limitado,
nao usa a transformacao mais recente como substituta e preserva a politica de
limpeza do historico temporal.

## 7. Integracao com a STVL oficial

O arquivo instalado
`tof_stvl_robot/config/nav2_stvl_a010_example.yaml` e somente referencia. Ele
nao e carregado pelo bringup. Mescle a fonte no `local_costmap` existente do
robo, respeitando o nome e os demais parametros da camada oficial.

O topico deve ser exatamente:

```text
/ground_segmentation/obstacle_points
```

Nunca use `/tof_filters/*` em `observation_sources`. Nao crie outro local
costmap neste workspace.

Exemplo da fonte validado contra a STVL instalada no ROS 2 Jazzy deste
ambiente:

```yaml
stvl_layer:
  observation_sources: pointcloud
  pointcloud:
    topic: /ground_segmentation/obstacle_points
    data_type: PointCloud2
    marking: true
    clearing: false
    observation_persistence: 0.0
    obstacle_range: 2.0
    min_obstacle_height: 0.04
    max_obstacle_height: 1.20
    expected_update_rate: 0.0
    inf_is_valid: false
    clear_after_reading: true
    sensor_frame: tof
```

## 8. Validacoes antes de mover o robo

### 8.1 Nos

```bash
ros2 node list | grep -E \
  'sipeed_tof|tof_pointcloud_preprocessor|local_costmap|robot_state_publisher'
```

O launch A010 cria apenas `/sipeed_tof_ms_a010` e
`/tof_pointcloud_preprocessor`. Costmap e `robot_state_publisher` devem ser os
ja existentes no bringup oficial.

### 8.2 Nuvem bruta

```bash
ros2 topic info /cloud -v
ros2 topic hz /cloud
ros2 topic echo /cloud --once --field header
```

Confirme driver como publisher, `frame_id: tof`, timestamp crescente e taxa
estavel.

### 8.3 Nuvem filtrada

```bash
ros2 topic info /ground_segmentation/obstacle_points -v
ros2 topic hz /ground_segmentation/obstacle_points
ros2 topic echo \
  /ground_segmentation/obstacle_points \
  --once \
  --field header
```

Confirme o pre-processador como unico publisher, `frame_id: tof`, timestamp
preservado e o costmap oficial como subscriber quando o Nav2 estiver ativo.

### 8.4 Parametros carregados

```bash
ros2 param dump /tof_pointcloud_preprocessor
```

Confira `input_topic`, `output_topic`, `target_frame`, `output_frame`,
`temporal_reference_frame`, `temporal_match_radius`,
`temporal_required_frames` e flags de debug.

### 8.5 Debug desligado

O YAML de producao usa:

```yaml
publish_intermediate_clouds: false
publish_filter_bounds: false
publish_projected_wall: false
```

Nesse perfil, publishers e timer de debug nao sao criados, a parede nao e
calculada e as mensagens intermediarias nao sao serializadas. A saida final
continua ativa. Compare CPU apenas com medicoes reais usando `top`, `htop` e
`ros2 topic hz`; nao presuma ganhos numericos.

### 8.6 Debug ligado

Altere temporariamente as tres flags para `true` e reinicie o launch. Mudancas
dinamicas dessas flags sao rejeitadas para evitar publishers parcialmente
configurados.

```bash
ros2 topic hz /tof_filters/distance
ros2 topic hz /tof_filters/height
ros2 topic hz /tof_filters/lateral
ros2 topic hz /tof_filters/spatial
ros2 topic echo /tof_filters/filter_bounds --once
ros2 topic hz /tof_filters/projected_wall
```

No RViz, use Fixed Frame `odom` e adicione `/cloud`, os topicos de debug,
`/ground_segmentation/obstacle_points`, TF, RobotModel e o mapa do costmap
oficial.

### 8.7 STVL efetivamente carregada

```bash
ros2 topic info /ground_segmentation/obstacle_points -v
ros2 param dump /local_costmap/local_costmap
```

Procure a fonte pelo topico, sem presumir que a camada se chama exatamente
`stvl_layer`. Confirme que nenhum topico de debug e consumido pela camada.

### 8.8 Filtro temporal com odometria

Com o robo parado, coloque um objeto estatico e confirme a publicacao depois
dos frames exigidos. Em velocidade baixa, confirme que o objeto permanece
estavel em `odom` com `temporal_match_radius: 0.05`. Depois de reset ou falha de
odometria, o primeiro frame apenas repovoa o historico; pontos antigos nao
podem ser reutilizados em outro referencial.

## 9. Checklist fisico de seguranca

Antes de navegacao autonoma normal, teste:

1. objeto baixo pouco acima de `height_min`;
2. caixa opaca;
3. obstaculo proximo;
4. obstaculo perto de `distance_max`;
5. obstaculo nas bordas laterais;
6. ambiente vazio;
7. robo parado;
8. robo avancando lentamente;
9. robo girando;
10. remocao do obstaculo e limpeza do costmap.

Comece com o robo suspenso ou em velocidade muito baixa e mantenha parada de
emergencia disponivel. O costmap nao substitui um dispositivo de seguranca
certificado.

## 10. Solucao de problemas

### `/cloud` nao publica

Confira dispositivo, permissao, logs do watchdog e se o driver e o unico dono
da porta serial.

### `/dev/tof` nao existe

Use `ls`, `readlink` e `udevadm` da secao de dispositivo. Passe temporariamente
`device:=/dev/ttyUSBX` somente depois de identificar a interface correta.

### Permissao serial negada

Confira o grupo `dialout`, as permissoes da porta e refaca login apos alterar
grupos.

### TF `base_footprint -> tof` ausente

Corrija o URDF/Xacro e o `robot_state_publisher` oficial. Nao adicione TF
estatica neste launch.

### TF `odom -> base_footprint` ausente

Inicie a odometria real. O filtro temporal nao deve usar TF antiga ou a mais
recente fora do timestamp da nuvem.

### Topico final sem subscriber

Confira o YAML realmente carregado pelo Nav2 e procure
`/ground_segmentation/obstacle_points` no dump do local costmap.

### Topicos `/tof_filters/*` ausentes

Isso e esperado no perfil de producao. Habilite as flags e reinicie o no.

### Dois publicadores para a mesma TF

Use `view_frames` e remova o publicador duplicado. Este pacote nao publica TF.

### Pacote ou launch nao encontrado

Confirme `colcon list`, o build dos dois pacotes e a ordem Jazzy, `robot_ws`,
`tof_robot_ws` nos comandos `source`.

## 11. Testes que exigem hardware

Build, lint, testes unitarios e `--show-args` nao validam o sensor fisico.
Continuam obrigatorios no robo:

- abertura e reconexao real de `/dev/tof`;
- taxa e timestamps reais de `/cloud`;
- TF dinamica da odometria;
- TF do URDF oficial;
- consumo pelo STVL/Nav2 oficial;
- CPU medida com debug ligado e desligado;
- todos os testes fisicos de seguranca.
