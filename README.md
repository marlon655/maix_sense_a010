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

## Usar em outro workspace ROS 2

Para uso real em outro workspace, o pacote essencial e apenas:

```text
src/sipeed_tof_ms_a010
```

O restante do repositorio e material de teste/documentacao:

```text
src/tof_stvl_test              # testes locais com STVL/costmap
run_*.sh                       # scripts de teste
tutorial_testes_maixsense_*.md # roteiro detalhado dos testes
maixsense_a010_falha_*.md      # anotacoes da investigacao da falha
```

Exemplo para copiar/clonar o driver em outro workspace:

```bash
cd ~/seu_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select sipeed_tof_ms_a010
source install/setup.bash
```

Executavel principal:

```bash
ros2 run sipeed_tof_ms_a010 sipeed_tof_node --ros-args \
  -p device:=/dev/tof \
  -p watchdog_timeout_sec:=3.0 \
  -p watchdog_cooldown_sec:=1.0
```

Topicos publicados:

```text
/cloud
/depth
```

## Descobrir dispositivo

```bash
ls -l /dev/tof /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

O `/dev/ttyUSB0` usado nos exemplos e apenas um exemplo. Antes de rodar o
driver, confirme em qual porta a interface de dados do MaixSense apareceu.

O A010 normalmente cria duas seriais, por exemplo `/dev/ttyUSB0` e
`/dev/ttyUSB1`. A porta correta e a que responde `AT` ou ja esta enviando o
stream binario do sensor.

Para inspecionar as interfaces:

```bash
udevadm info -q property -n /dev/ttyUSB0 | grep -E 'ID_SERIAL|ID_MODEL|ID_VENDOR|ID_USB_INTERFACE_NUM|ID_PATH'
udevadm info -q property -n /dev/ttyUSB1 | grep -E 'ID_SERIAL|ID_MODEL|ID_VENDOR|ID_USB_INTERFACE_NUM|ID_PATH'
```

Se nao existir `/dev/tof`, use a `/dev/ttyUSB*` correta. Se `/dev/tof` existir,
confira para onde aponta:

```bash
readlink -f /dev/tof
```

No PC final, recomenda-se criar regra udev para `/dev/tof` apontar sempre para a
interface de dados correta.

## Rodar driver isolado

Troque `/dev/ttyUSB0` pela porta correta encontrada na etapa anterior, se
necessario.

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

## Configurar parametros AT manualmente

O driver ja configura o necessario para publicar `/cloud`: ele para stream antigo,
consulta `AT+COEFF?` e inicia com `AT+DISP=3`. Normalmente nao precisa alterar os
parametros internos do sensor.

Se precisar testar configuracoes como `FPS`, `BINN`, `UNIT` ou `BAUD`, use o
helper interativo:

```bash
cd ~/tof_test_ws
python3 tools/maixsense_at_config.py --device /dev/ttyUSB0
```

Troque `/dev/ttyUSB0` pela porta correta do sensor.

O script:

- para o stream com `AT+DISP=1` antes de consultar/enviar comandos;
- mostra valores atuais com `AT+FPS?`, `AT+BAUD?`, `AT+UNIT?`, `AT+BINN?` e `AT+DISP?`;
- permite deixar cada campo em branco para manter o valor atual;
- salva o plano em `maixsense_at_configs/*.json` antes de enviar;
- exige confirmacao extra para alterar `BAUD`.

Na parte de comandos customizados, exemplos validos sao:

```text
AT+BAUD?   # consulta baud configurado
AT+FPS?    # consulta FPS configurado
AT+FPS=10  # altera FPS para 10
AT+DISP=1  # para stream USB
AT+DISP=3  # inicia stream USB
```

Use letras maiusculas nos comandos AT, seguindo o formato do manual/wiki.

Para simular sem enviar comandos:

```bash
python3 tools/maixsense_at_config.py --device /dev/ttyUSB0 --dry-run
```

Evite alterar `BAUD` e `UNIT` sem necessidade. `BAUD` pode quebrar a comunicacao
se o host e o sensor ficarem em velocidades diferentes; `UNIT` altera a escala de
profundidade e pode exigir recalibrar a conversao de distancia no driver.

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
