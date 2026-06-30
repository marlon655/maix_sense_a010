# Tutorial de Testes - MaixSense A010 no `tof_test_ws`

Este tutorial descreve a sequencia de testes usada para validar o MaixSense A010 com o driver `sipeed_tof_ms_a010`, evitando o problema `ISP is busy`.

A versao experimental do driver neste workspace foi alterada para:

- nao usar `AT+ISP=0` / `AT+ISP=1`;
- parar stream antigo com `AT+DISP=1` no inicio;
- drenar dados binarios pendentes antes de mandar `AT`;
- usar retries para `AT`, `AT+COEFF?` e `AT+DISP=3`;
- encerrar melhor com `Ctrl+C`;
- nao quebrar se a serial nao abrir;
- limitar o loop de leitura para nao travar o processo dentro do callback;
- adicionar watchdog de `/cloud`;
- reabrir a serial se a USB cair e voltar como outra `/dev/ttyUSB*`;
- aceitar stream binario ja ativo depois de reconectar USB;
- resetar o buffer interno do parser quando reinicia/reconecta;
- descartar cabecalho invalido no parser, evitando ficar preso em lixo de frame.

Resumo do driver atual:

```text
Inicializacao: AT+DISP=1 -> drain -> AT -> AT+COEFF? -> AT+DISP=3
Watchdog: se /cloud parar -> reabre serial -> procura ttyUSB -> reseta parser -> volta a publicar
Nao usa: AT+ISP=0 / AT+ISP=1
```

Workspace usado:

```bash
cd ~/tof_test_ws
```

## Importante sobre RMW

No notebook, o `.bashrc` pode estar configurado para usar Zenoh apontando para o robo:

```bash
RMW_IMPLEMENTATION=rmw_zenoh_cpp
ZENOH_CONFIG_OVERRIDE=...
```

Para estes testes locais, use FastDDS em todos os terminais:

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ZENOH_CONFIG_OVERRIDE
unset ZENOH_SHM_ALLOC_SIZE
```

Se aparecer erro parecido com:

```text
Unable to connect to tcp/192.168.1.118:7447
Error setting up zenoh session
```

significa que aquele terminal ainda esta usando Zenoh. Rode novamente os exports acima.

## Preparar cada terminal

Em todo terminal usado nos testes, rode:

```bash
cd ~/tof_test_ws
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ZENOH_CONFIG_OVERRIDE
unset ZENOH_SHM_ALLOC_SIZE
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Para conferir:

```bash
echo $RMW_IMPLEMENTATION
echo $ZENOH_CONFIG_OVERRIDE
```

Esperado:

```text
rmw_fastrtps_cpp
```

E `ZENOH_CONFIG_OVERRIDE` deve sair vazio.

## Verificar portas Serial
```bash
ls -l /dev/tof /dev/ttyUSB* /dev/ttyACM*
```

```bash
udevadm info -q property -n /dev/ttyUSB0 | grep -E 'ID_SERIAL|ID_MODEL|ID_VENDOR|ID_USB_INTERFACE_NUM|ID_PATH'
udevadm info -q property -n /dev/ttyUSB1 | grep -E 'ID_SERIAL|ID_MODEL|ID_VENDOR|ID_USB_INTERFACE_NUM|ID_PATH'
```

## Teste 1 - Verificar porta serial correta

Terminal unico:

```bash
cd ~/tof_test_ws
python3 - <<'PY'
import serial, time

for dev in ["/dev/tof", "/dev/ttyUSB0", "/dev/ttyUSB1"]:
    print("\n###", dev)
    try:
        ser = serial.Serial(dev, 115200, timeout=2, write_timeout=1)
        ser.reset_input_buffer()
        ser.write(b"AT\r")
        ser.flush()
        time.sleep(1)
        print(ser.read(1000))
        ser.close()
    except Exception as e:
        print(e)
PY
```

Resultado bom:

```text
/dev/ttyUSB0 -> b'OK\r\n'
```

Se aparecer:

```text
ISP is busy
Dragonfly ISP stop failed
```

entao o sensor entrou em estado ruim. Nesse estado, geralmente precisa desligar/ligar fisicamente o A010.

## Teste 2 - Rodar somente o driver

Terminal 1 - driver:

```bash
cd ~/tof_test_ws
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ZENOH_CONFIG_OVERRIDE
unset ZENOH_SHM_ALLOC_SIZE
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run sipeed_tof_ms_a010 sipeed_tof_node --ros-args -p device:=/dev/ttyUSB0
```

Inicializacao esperada:

```text
use device: /dev/ttyUSB0
finish: AT OK
finish: AT+COEFF? ... JSON ...
parse json
fx: 75
fy: 75
u0: 47.3667
v0: 53.4833
finish: AT+DISP=3: read chunk: OK
```

Nao deve aparecer:

```text
ISP is busy
Dragonfly ISP stop failed
```

Terminal 2 - monitorar `/cloud`:

```bash
cd ~/tof_test_ws
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ZENOH_CONFIG_OVERRIDE
unset ZENOH_SHM_ALLOC_SIZE
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 topic hz /cloud
```

Resultado esperado:

```text
average rate: ~6.8 a 7.1
```

Deixe rodando 20 a 30 minutos sem abrir RViz e sem STVL. Se nao travar, o driver/sensor isolado esta estavel.

## Teste 3 - Parar e iniciar manualmente

Terminal 1:

1. Inicie o driver.
2. Confirme no Terminal 2 que `/cloud` publica.
3. Aperte `Ctrl+C` no driver.
4. Espere 2 segundos.
5. Inicie de novo.

Comando do driver:

```bash
ros2 run sipeed_tof_ms_a010 sipeed_tof_node --ros-args -p device:=/dev/ttyUSB0
```

Repita 5 vezes.

Resultado esperado:

- `Ctrl+C` encerra rapido;
- proxima inicializacao volta com `AT OK`;
- `/cloud` volta a publicar;
- nao aparece `ISP is busy`.

## Teste 4 - Stress de restart automatico

Terminal unico:

```bash
cd ~/tof_test_ws
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ZENOH_CONFIG_OVERRIDE
unset ZENOH_SHM_ALLOC_SIZE
source /opt/ros/jazzy/setup.bash
source install/setup.bash

for i in $(seq 1 10); do
  echo "===== RUN $i ====="
  timeout --signal=INT 20s ros2 run sipeed_tof_ms_a010 sipeed_tof_node --ros-args -p device:=/dev/ttyUSB0
  sleep 1
done
```

Resultado esperado em cada ciclo:

```text
finish: AT OK
finish: AT+COEFF? ... JSON ...
finish: AT+DISP=3: read chunk: OK
signal_handler(SIGINT/SIGTERM)
```

Nao deve aparecer:

```text
ISP is busy
Dragonfly ISP stop failed
```

Observacao: se usar `timeout` muito curto, ele pode matar o node durante a inicializacao. Use `20s` ou mais.

## Teste 5 - Watchdog de queda/reconexao USB

Este teste valida a recuperacao automatica quando o cabo de dados USB cai e volta. Mantenha a alimentacao separada do A010 ligada; desconecte apenas os dados USB.

Terminal 1 - driver:

```bash
cd ~/tof_test_ws
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ZENOH_CONFIG_OVERRIDE
unset ZENOH_SHM_ALLOC_SIZE
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run sipeed_tof_ms_a010 sipeed_tof_node --ros-args \
  -p device:=/dev/ttyUSB0 \
  -p watchdog_timeout_sec:=3.0 \
  -p watchdog_cooldown_sec:=1.0
```

Terminal 2 - monitorar `/cloud`:

```bash
cd ~/tof_test_ws
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ZENOH_CONFIG_OVERRIDE
unset ZENOH_SHM_ALLOC_SIZE
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 topic hz /cloud
```

Sequencia:

1. Aguarde `/cloud` publicar perto de `6 a 7 Hz`.
2. Desconecte somente o cabo de dados USB.
3. Aguarde o watchdog reclamar.
4. Reconecte o cabo de dados USB, na mesma porta ou em outra porta.
5. Confirme se `/cloud` volta a publicar sem reiniciar o node.

Logs esperados no driver:

```text
No /cloud frames for 3.0s. Restarting MaixSense stream.
Watchdog reopening serial.
watchdog probe device: /dev/ttyUSB0
watchdog using active stream device: /dev/ttyUSB0 drained_bytes: ...
MaixSense stream was already active after serial reopen.
```

Se o Linux recriar o dispositivo com outro numero, tambem e aceitavel:

```text
watchdog probe device: /dev/ttyUSB1
watchdog using active stream device: /dev/ttyUSB1 drained_bytes: ...
```

Resultado esperado:

- sem `Segmentation fault`;
- sem `ISP is busy`;
- sem precisar parar o node manualmente;
- `/cloud` volta sozinho no `ros2 topic hz /cloud`.

## Teste 6 - STVL sem RViz

Este teste sobe o driver real, TF fixo e um `local_costmap` com STVL lendo `/cloud`.

Terminal 1:

```bash
cd ~/tof_test_ws
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ZENOH_CONFIG_OVERRIDE
unset ZENOH_SHM_ALLOC_SIZE
source /opt/ros/jazzy/setup.bash
source install/setup.bash

./run_maixsense_stvl_test.sh /dev/ttyUSB0
```

Terminal 2:

```bash
cd ~/tof_test_ws
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ZENOH_CONFIG_OVERRIDE
unset ZENOH_SHM_ALLOC_SIZE
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 topic hz /cloud
```

Terminal 3:

```bash
cd ~/tof_test_ws
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ZENOH_CONFIG_OVERRIDE
unset ZENOH_SHM_ALLOC_SIZE
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 topic list | grep -E "cloud|costmap|stvl|voxel"
```

Topicos esperados:

```text
/cloud
/local_costmap/costmap
/local_costmap/stvl_layer
/local_costmap/costmap_raw
```

Deixe 20 minutos. Se `/cloud` continuar publicando, teste passou.

## Teste 7 - RViz leve

Primeiro suba o teste MaixSense + STVL. Ele publica o driver, TF e costmap usados pelo RViz.

Terminal 1:

```bash
cd ~/tof_test_ws
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ZENOH_CONFIG_OVERRIDE
unset ZENOH_SHM_ALLOC_SIZE
source /opt/ros/jazzy/setup.bash
source install/setup.bash

./run_maixsense_stvl_test.sh /dev/ttyUSB0
```

Depois abra o RViz em outro terminal.

Terminal 2:

```bash
cd ~/tof_test_ws
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ZENOH_CONFIG_OVERRIDE
unset ZENOH_SHM_ALLOC_SIZE
source /opt/ros/jazzy/setup.bash
source install/setup.bash

rviz2
```

No RViz:

```text
Fixed Frame: odom
```

Adicione:

```text
TF
PointCloud2 -> /cloud
Map -> /local_costmap/costmap
Map -> /local_costmap/stvl_layer
```

Configuracao recomendada para `/cloud`:

```text
Style: Points
Size: 0.01 ou 0.02
```

Nao adicione `/local_costmap/voxel_grid` no primeiro teste. Esse topico pode pesar e travar o RViz.

## Como parar tudo

```bash
pkill -f sipeed_tof_node
pkill -f run_maixsense_stvl_test.sh
pkill -f stvl_maixsense_test.launch.py
pkill -f nav2_costmap_2d
pkill -f lifecycle_manager_costmap_test
pkill -f static_transform_publisher
```

Verificar se sobrou processo:

```bash
ps -ef | grep -E "sipeed_tof|tof_stvl|maixsense|nav2_costmap|static_transform" | grep -v grep
```

Verificar se a porta serial esta livre:

```bash
fuser -v /dev/tof /dev/ttyUSB0 /dev/ttyUSB1
```

Se nao imprimir nada, ninguem esta usando a serial.

## Interpretacao dos erros

### `/dev/tof` apontando para porta errada

O MaixSense A010 cria duas interfaces seriais, normalmente `/dev/ttyUSB0` e `/dev/ttyUSB1`. Em alguns testes, `/dev/tof` apontou para a interface errada.

Para conferir:

```bash
readlink -f /dev/tof
udevadm info -q property -n /dev/ttyUSB0 | grep ID_USB_INTERFACE_NUM
udevadm info -q property -n /dev/ttyUSB1 | grep ID_USB_INTERFACE_NUM
```

No PC final/robo, crie regra udev para a interface correta. Durante estes testes, pode usar direto `/dev/ttyUSB0` ou a porta que estiver publicando.

### `ISP is busy`

Falha ruim do sensor/firmware. Reiniciar o node pode nao resolver.

```text
ISP is busy
Dragonfly ISP stop failed
```

Se aparecer, pare tudo e tente power-cycle fisico do A010.

### Erro Zenoh

```text
Unable to connect to tcp/192.168.1.118:7447
Error setting up zenoh session
```

O terminal esta usando Zenoh. Rode:

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ZENOH_CONFIG_OVERRIDE
unset ZENOH_SHM_ALLOC_SIZE
```

### `/cloud` com pausa grande

Se voce parou/reiniciou o driver enquanto `ros2 topic hz /cloud` continuou rodando, e normal aparecer `max` grande. Para testar estabilidade, nao mexa no driver durante o `topic hz`.
