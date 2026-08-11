# reBotArm ROS 2 Workspace

Workspace ROS 2 para controle, visualizacao, simulacao e visao computacional do
braco robotico reBotArm.

O projeto reune o controlador ROS 2 do hardware, mensagens customizadas, bringup,
configuracao MoveIt 2, uma GUI desktop em PySide6/Qt, pipeline de visao com
OpenCV e ferramentas auxiliares para treino YOLO.

## Pacotes

| Pacote | Funcao |
| --- | --- |
| `rebotarm_msgs` | Mensagens, servicos e actions customizadas do projeto. |
| `rebotarmcontroller` | No controlador que conversa com o SDK/hardware e expoe topicos, servicos e actions. |
| `rebotarm_bringup` | Launch files, parametros, URDFs, meshes e configuracao do hardware. |
| `rebotarm_moveit_config` | Configuracao MoveIt 2, planning scene, controladores e RViz. |
| `rebotarm_moveit_demos` | Demos de movimento e pick/place usando MoveIt 2. |
| `rebotarm_gui` | Interface desktop PySide6/Qt e mock do braco para testes sem hardware. |
| `rebotarm_vision` | Captura de imagem, segmentacao, calibracao, pose de objetos e fila de coleta. |
| `rebotarm_yolo_trainer` | Utilitarios para treino, inspecao e exportacao de modelos YOLO. |

## Requisitos

- Ubuntu com ROS 2 instalado.
- `colcon`, `rosdep` e ferramentas de build ROS 2.
- Python 3 e dependencias dos pacotes.
- PySide6 para a interface grafica.
- Para hardware real: acesso ao barramento serial/CAN configurado no sistema.

Este workspace foi usado localmente com ROS 2 em `/opt/ros/lyrical`. Para
Raspberry Pi, o guia de instalacao cobre Ubuntu 22.04/Humble e 24.04/Jazzy.

## Instalacao Rapida

Clone o repositorio como workspace ROS 2:

```bash
cd ~
git clone https://github.com/wesleycatao/rebotarm-ros2-ws.git ros2_ws
cd ros2_ws
```

Instale dependencias do sistema e do ROS 2 conforme seu ambiente. Para um
Raspberry Pi novo, siga o guia dedicado:

```text
README_RASPBERRY_PI.md
```

## Build

Carregue o ROS 2 e compile:

```bash
cd ~/ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Para compilar apenas os pacotes principais do workspace:

```bash
colcon build --packages-select \
  rebotarm_msgs \
  rebotarmcontroller \
  rebotarm_bringup \
  rebotarm_moveit_config \
  rebotarm_moveit_demos \
  rebotarm_gui \
  rebotarm_vision \
  rebotarm_yolo_trainer \
  --symlink-install
```

## Executar Sem Hardware

O pacote `rebotarm_gui` inclui um mock ROS 2 do braco. Ele permite testar a GUI,
servicos, actions, estados de juntas e fluxos de programa sem o robo fisico.

Terminal 1:

```bash
cd ~/ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
ros2 run rebotarm_gui mock_arm
```

Terminal 2:

```bash
cd ~/ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
ros2 run rebotarm_gui rebotarm_gui
```

Para visualizar o mock no RViz:

```bash
ros2 launch rebotarm_gui mock_visualize.launch.py
```

## Executar Com Hardware

Revise primeiro os parametros de hardware:

```text
src/rebotarm_bringup/config/rebotarm_hardware.yaml
src/rebotarm_bringup/config/driver_params.yaml
```

Depois inicie o bringup apropriado ao modelo e interface configurados:

```bash
cd ~/ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
ros2 launch rebotarm_bringup bringup.launch.py
```

Para habilitar a execução cartesiana com verificação de auto-colisão e dos
obstáculos presentes na planning scene, inicie também o MoveIt (sem um
`/move_action` disponível, a GUI bloqueia por padrão a execução real):

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
ros2 launch rebotarm_moveit_config hardware.launch.py use_rviz:=false
```

Em outra janela, execute a GUI:

```bash
ros2 run rebotarm_gui rebotarm_gui
```

## Visao Computacional

O pacote `rebotarm_vision` processa imagem de camera ou arquivo, segmenta
objetos, estima pose e publica dados para a GUI e para fluxos de coleta.

Launch principal:

```bash
ros2 launch rebotarm_vision vision.launch.py
```

Launch com configuracao YOLO:

```bash
ros2 launch rebotarm_vision vision_yolo.launch.py
```

Veja detalhes em `src/rebotarm_vision/README.md`.

## Testes

Depois do build:

```bash
cd ~/ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
colcon test
colcon test-result --verbose
```

Para testes de GUI em ambiente sem display:

```bash
QT_QPA_PLATFORM=offscreen python3 -m pytest src/rebotarm_gui/test -v
```

Alguns testes de integracao ROS sobem nos, DDS/RTPS e sockets locais; eles
precisam de um ambiente que permita rede/sockets.

## Documentacao

- Guia completo para Raspberry Pi: `README_RASPBERRY_PI.md`
- GUI e mock: `src/rebotarm_gui/README.md`
- Pipeline de visao: `src/rebotarm_vision/README.md`
- Trainer YOLO: `src/rebotarm_yolo_trainer/README.md`
- Redesenho/UX da GUI: `src/rebotarm_gui/GUI_UX_REDESIGN.md`
- Acessibilidade/usabilidade: `src/rebotarm_gui/ACCESSIBILITY_USABILITY.md`
- Auditoria da GUI: `src/rebotarm_gui/AUDIT_LOG.md`

## Estrutura

```text
ros2_ws/
├── README.md
├── README_RASPBERRY_PI.md
├── setup_all.sh
└── src/
    ├── rebotarm_bringup/
    ├── rebotarm_gui/
    ├── rebotarm_moveit_config/
    ├── rebotarm_moveit_demos/
    ├── rebotarm_msgs/
    ├── rebotarm_vision/
    ├── rebotarm_yolo_trainer/
    └── rebotarmcontroller/
```

Artefatos gerados como `build/`, `install/`, `log/`, caches Python e arquivos
locais de maquina ficam fora do Git pelo `.gitignore`.
