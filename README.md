# MaixSense A010 ROS 2 Driver Test Workspace

Workspace isolado para testar o driver ROS 2 do Sipeed MaixSense A010 e validar
publicacao de nuvem de pontos, STVL e recuperacao de falhas de USB.

## Principais ajustes deste driver

- Remove a sequencia `AT+ISP=0` / `AT+ISP=1`, que podia deixar o sensor em
  `ISP is busy` / `Dragonfly ISP stop failed`.
- Para stream antigo com `AT+DISP=1` antes de iniciar.
- Drena dados binarios pendentes antes de mandar comandos AT.
- Aceita resposta JSON direta de `AT+COEFF?`.
- Limita o loop de leitura para evitar travar o node dentro do callback.
- Adiciona watchdog de `/cloud`.
- Reabre a serial se a USB cair e voltar como outra `/dev/ttyUSB*`.
- Aceita stream binario ja ativo apos reconexao USB.
- Reseta o buffer interno do parser apos reiniciar/reconectar.
- Descarta cabecalho invalido no parser para nao ficar preso em lixo de frame.

Fluxo atual:

```text
Inicializacao: AT+DISP=1 -> drain -> AT -> AT+COEFF? -> AT+DISP=3
Watchdog: se /cloud parar -> reabre serial -> procura ttyUSB -> reseta parser -> volta a publicar
Nao usa: AT+ISP=0 / AT+ISP=1
```

## Build

```bash
cd ~/tof_test_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select sipeed_tof_ms_a010
source install/setup.bash
```

Para buildar tambem os testes STVL:

```bash
colcon build
source install/setup.bash
```

## Descobrir dispositivo

```bash
ls -l /dev/tof /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

Se nao existir `/dev/tof`, use o `/dev/ttyUSB0` ou `/dev/ttyACM0` correspondente.

## Rodar driver isolado

```bash
ros2 run sipeed_tof_ms_a010 sipeed_tof_node --ros-args -p device:=/dev/tof
```

Ou:

```bash
ros2 run sipeed_tof_ms_a010 sipeed_tof_node --ros-args \
  -p device:=/dev/ttyUSB0 \
  -p watchdog_timeout_sec:=3.0 \
  -p watchdog_cooldown_sec:=1.0
```

## Conferir topicos

Em outro terminal:

```bash
cd ~/tof_test_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic list | grep -E "cloud|depth"
ros2 topic echo /cloud --once
```

Para acompanhar a taxa:

```bash
ros2 topic hz /cloud
```

Valor esperado nos testes locais: perto de `6 a 7 Hz`.

## Se der permissao negada

```bash
sudo usermod -aG dialout $USER
```

Depois saia e entre novamente na sessao.

## Observacao sobre AT+COEFF

Este workspace usa a versao local do driver que aceita a resposta JSON direta do
sensor no comando `AT+COEFF?`, alem do formato antigo `+COEFF=1`.

## Tutorial completo

Veja [tutorial_testes_maixsense_a010.md](tutorial_testes_maixsense_a010.md) para
os testes de stress, watchdog, STVL e RViz.
