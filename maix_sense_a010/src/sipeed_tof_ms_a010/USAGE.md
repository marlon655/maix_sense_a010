# Uso do pacote `sipeed_tof_ms_a010`

Para usar este driver em outro workspace ROS 2, copie somente a pasta:

```text
src/sipeed_tof_ms_a010
```

Depois compile o pacote:

```bash
cd ~/seu_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select sipeed_tof_ms_a010
source install/setup.bash
```

## Usar o arquivo de parametros no launch

O arquivo de parametros fica em:

```text
config/maixsense_params.yaml
```

No launch do robo, carregue esse YAML assim:

```python
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

tof_params = os.path.join(
    get_package_share_directory('sipeed_tof_ms_a010'),
    'config',
    'maixsense_params.yaml'
)

tof_node = Node(
    package='sipeed_tof_ms_a010',
    executable='sipeed_tof_node',
    name='sipeed_tof_ms_a010',
    output='screen',
    parameters=[tof_params],
)
```

Assim o driver inicia automaticamente com os parametros definidos no YAML.

## Rodar manualmente

Depois do build, tambem e possivel rodar manualmente:

```bash
ros2 run sipeed_tof_ms_a010 sipeed_tof_node --ros-args \
  --params-file install/sipeed_tof_ms_a010/share/sipeed_tof_ms_a010/config/maixsense_params.yaml
```

## Topicos publicados

```text
/cloud
/depth
```

## Observacao sobre porta serial

No arquivo `config/maixsense_params.yaml`, confira o parametro:

```yaml
device: /dev/tof
```

No robo final, recomenda-se criar uma regra udev para `/dev/tof` apontar para a
interface de dados correta do MaixSense A010.
