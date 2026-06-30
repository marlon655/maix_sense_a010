# Falha MaixSense A010 - estado `ISP is busy`

## Resumo

Durante os testes com o MaixSense A010, o sensor entrou em um estado onde a serial ainda aparece no Linux, mas o firmware nao responde normalmente aos comandos AT. A resposta observada foi:

```text
ISP is busy
Dragonfly ISP stop failed
```

Nesse estado, reiniciar apenas o node ROS ou recarregar o driver USB serial nao garante recuperacao. O sensor pode precisar de power-cycle fisico, ou seja, cortar e religar a alimentacao USB.

## Sintomas observados

O dispositivo continuou aparecendo:

```text
/dev/tof -> ttyUSB0
/dev/ttyUSB0
/dev/ttyUSB1
```

Mas o teste serial retornou:

```text
SEND b'AT'
READ b'ISP is busy\n'

SEND b'AT+ISP=1'
READ b'ISP is busy\nDragonfly ISP stop failed\r\nOK\r\n'
```

Ao tentar iniciar o driver:

```text
use device: /dev/tof
AT+ISP=0: get dummy: 02
finish: AT+ISP=0
AT+DISP=1: get dummy: 0
finish: AT+DISP=1
AT+ISP=1: get dummy: 0
finish: AT+ISP=1
finish: AT ISP is busy
```

O driver espera que o comando `AT` responda `OK`. Como recebeu `ISP is busy`, ele nao consegue continuar para `AT+COEFF?` e nao publica `/cloud`.

## O que foi descartado

O teste isolado da STVL com nuvem fake funcionou no PC local:

```text
max: 100
nonzero: 158
lethal: 8
```

Isso indica que a STVL consegue transformar uma nuvem `PointCloud2` em custo 2D quando recebe dados validos. Portanto, esta falha especifica nao parece ser do `local_costmap` nem da STVL.

Tambem foi verificado que recarregar modulos USB recria as portas:

```bash
sudo modprobe -r ftdi_sio
sudo modprobe ftdi_sio
```

Mas isso reseta a interface USB serial, nao necessariamente corta a alimentacao do A010. Se o microcontrolador interno estiver preso no estado ISP, a porta pode voltar a existir sem o sensor voltar ao modo normal.


## Pesquisa online

Foi feita busca por ocorrencias publicas dos termos:

```text
MaixSense A010 "ISP is busy"
"Dragonfly ISP stop failed"
"AT+ISP" "MaixSense"
"AT+COEFF?" "MaixSense"
```

Nao foi encontrado um relato publico especifico da falha `Dragonfly ISP stop failed` no MaixSense A010. Tambem nao apareceu uma solucao oficial documentada para esse erro especifico.

As fontes oficiais mais proximas encontradas foram a organizacao Sipeed no GitHub e a wiki da Sipeed, mas sem uma pagina publica indexada descrevendo esse erro especifico. Portanto, a analise abaixo e uma inferencia tecnica baseada nos logs do sensor, no comportamento da serial e no codigo do driver local.

Links consultados:

- https://github.com/sipeed
- https://wiki.sipeed.com/

## Causa provavel

A causa provavel nao e o ROS nem a STVL. O sensor fica preso em uma rotina interna de ISP/firmware. Neste contexto de camera/ToF, `ISP` provavelmente se refere ao pipeline/processador interno de imagem/profundidade, nao necessariamente ao bootloader.

Evidencias:

- A porta USB serial continua existindo (`/dev/tof`, `/dev/ttyUSB0`).
- A serial abre normalmente.
- O sensor responde `ISP is busy` ate para comandos AT simples.
- O comando `AT+ISP=1` retorna `Dragonfly ISP stop failed`, indicando falha interna ao tentar parar/sair do ISP.
- O driver espera `AT -> OK`, mas recebe `AT -> ISP is busy`.

Isso sugere um estado interno travado do firmware/controlador do A010. Nessa situacao, reiniciar apenas o processo ROS nao e suficiente.

## Impacto no app real

Sim, isso e um risco para a aplicacao real.

Se o A010 congelar nesse estado:

- `/cloud` para de publicar.
- O `ground_segmentation`, se usado, para de receber dados novos.
- A STVL deixa de marcar novos obstaculos.
- Os voxels existentes somem depois do `voxel_decay`.
- A navegacao perde a camada extra de percepcao frontal/3D.

Reiniciar somente o processo ROS pode nao resolver, porque o sensor continua alimentado e preso em `ISP is busy`.

## Diagnostico recomendado

Verificar se o sensor esta publicando:

```bash
ros2 topic hz /cloud
```

Verificar se a serial responde:

```bash
python3 - <<'PY'
import serial, time

for dev in ["/dev/tof", "/dev/ttyUSB0", "/dev/ttyUSB1"]:
    print("\nTEST", dev)
    try:
        ser = serial.Serial(dev, 115200, timeout=1, write_timeout=1)
        print("opened")
        ser.write(b"AT\r")
        time.sleep(0.5)
        print("read:", ser.read(200))
        ser.close()
    except Exception as e:
        print("ERR:", repr(e))
PY
```

Se a resposta for `ISP is busy`, o sensor nao esta em estado operacional normal.

Verificar se algum processo esta segurando a porta:

```bash
fuser -v /dev/tof /dev/ttyUSB0 /dev/ttyUSB1
```

## Recuperacao manual

Primeiro, parar processos usando o sensor:

```bash
pkill -f sipeed_tof_node
```

Tentar recarregar o driver USB serial, se for FTDI:

```bash
sudo modprobe -r ftdi_sio
sudo modprobe ftdi_sio
```

Se continuar em `ISP is busy`, fazer power-cycle fisico:

1. Desconectar o A010 do USB.
2. Aguardar alguns segundos.
3. Reconectar.
4. Confirmar que `/dev/tof` foi recriado.
5. Iniciar novamente o driver.

## Mitigacao para producao

A solucao robusta deve assumir que reiniciar o node pode nao recuperar o sensor.

Recomendacoes:

1. Criar watchdog para `/cloud`.
   - Se `/cloud` ficar sem mensagens por mais de 1 ou 2 segundos, declarar falha do ToF.

2. Reiniciar o node via `systemd`.
   - Ajuda quando o problema e somente processo.
   - Nao resolve se o sensor estiver preso em `ISP is busy`.

3. Implementar power-cycle controlado.
   - Hub USB com controle de energia.
   - Circuito com MOSFET/rele para cortar 5V do A010.
   - Saida GPIO do controlador do robo acionando esse corte.

4. Depois do power-cycle, reiniciar o driver.

5. Manter o LiDAR 2D como camada principal de seguranca.
   - O A010/STVL deve ser camada adicional, nao unico sensor de obstaculo.

## Exemplo de politica de watchdog

Fluxo sugerido:

```text
monitorar /cloud
  se hz normal:
    manter navegacao
  se /cloud parado por > 1s:
    registrar falha ToF
    parar/reiniciar sipeed_tof_node
  se /cloud nao voltar:
    cortar alimentacao do A010 por 2s
    religar alimentacao
    reiniciar sipeed_tof_node
  se ainda falhar:
    manter navegacao sem camada ToF ou entrar em modo seguro
```

## Conclusao

A falha `ISP is busy` indica um estado interno ruim do MaixSense A010/firmware. O sistema operacional ainda pode ver `/dev/tof`, mas isso nao significa que o sensor esteja operacional.

Para testes, desconectar e reconectar o USB deve recuperar. Para robo em producao, a solucao correta e watchdog mais power-cycle controlado do sensor.
