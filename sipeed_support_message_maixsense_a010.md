# Message to Sipeed Support - MaixSense A010 `ISP is busy`

Subject: MaixSense A010 stuck with "ISP is busy" / "Dragonfly ISP stop failed"

Hello Sipeed support,

I am testing a MaixSense A010 100x100 3D ToF sensor on Linux through USB serial.

The sensor works initially, but after some time it stops publishing frames. The USB serial device is still detected by Linux, but the sensor no longer responds normally to AT commands.

Device paths:

```text
/dev/tof -> /dev/ttyUSB0
/dev/ttyUSB0
/dev/ttyUSB1
```

When I send AT commands manually at 115200 baud, I get:

```text
AT -> ISP is busy
AT+ISP=1 -> ISP is busy
Dragonfly ISP stop failed
OK
```

Example manual serial test log:

```text
SEND b'AT+ISP=0'
READ b'ISP is busy\n'

SEND b'AT+DISP=0'
READ b''

SEND b'AT+DISP=1'
READ b'ISP is busy\n'

SEND b'AT'
READ b'ISP is busy\n'

SEND b'AT+ISP=1'
READ b'ISP is busy\nDragonfly ISP stop failed\r\nOK\r\n'

SEND b'AT+COEFF?'
READ b'ISP is busy\n'
```

When I start the driver after this state, it also fails:

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

The driver expects the command `AT` to return `OK`, but the sensor returns `ISP is busy` instead, so the driver cannot continue to `AT+COEFF?` and does not publish point cloud frames.

When this happens:

- Restarting the ROS driver does not recover the sensor.
- Reloading the Linux USB serial driver recreates `/dev/ttyUSB*`, but does not recover the sensor.
- Physically unplugging and plugging the USB cable seems to recover it.

The driver initialization sequence currently used is:

```text
AT+ISP=0
AT+DISP=1
AT+ISP=1
AT
AT+COEFF?
AT+DISP=3
```

Questions:

1. What does `ISP is busy` mean on the MaixSense A010?
2. What does `Dragonfly ISP stop failed` indicate?
3. Is there an AT command to reset or recover the sensor without physically power-cycling USB?
4. Is there a recommended initialization sequence for `AT+ISP` / `AT+DISP` to avoid this state?
5. Is the initialization sequence above correct for the current MaixSense A010 firmware?
6. Is there a firmware update for the MaixSense A010 that fixes this issue?
7. Is it safe to power-cycle the sensor automatically from an external controller if this state is detected?

Environment:

```text
OS: Ubuntu Linux
ROS 2: Jazzy
Serial baudrate: 115200
Driver package: sipeed_tof_ms_a010
Sensor: MaixSense A010 100x100 2.5m 3D ToF
```

Thank you.
