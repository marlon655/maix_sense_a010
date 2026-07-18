# sim_ws ToF v0.1

Snapshot limpo do workspace de simulacao ToF.

Conteudo principal:

- `sim_bot`: simulador Gazebo/RViz, mundos e modelo do robo;
- `nav_hub`: Nav2, mapas, route graph e perfis de simulacao;
- `maix_sense_a010`: driver A010 e pre-processador ToF;
- `setup_sim.bash`: source dos overlays do workspace.

## Build em outro PC

Instale dependencias ROS 2 Jazzy, clone esta branch e compile cada pacote como
workspace separado:

```bash
cd ~/sim_ws/sim_bot
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install

cd ~/sim_ws/nav_hub
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install

cd ~/sim_ws/maix_sense_a010
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

Depois:

```bash
cd ~/sim_ws
source setup_sim.bash
```

## Guias

Veja:

```text
maix_sense_a010/START_COMMANDS.md
```

Esse arquivo contem as sequencias para:

- simulacao normal;
- circuito de testes ToF;
- ToF real.
