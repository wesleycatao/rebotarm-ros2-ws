# reBotArm ROS 2 Workspace

Este documento descreve o projeto e serve como guia para carregar, instalar,
compilar e executar o workspace em um Raspberry Pi em uma instalacao futura.

O foco do Raspberry Pi deve ser rodar o controlador e o bringup do robo. A GUI
pode rodar no proprio Pi se houver ambiente grafico, mas normalmente e mais
confortavel executar a GUI em um notebook na mesma rede ROS 2.

Repositorio do projeto:

```text
https://github.com/wesleycatao/rebotarm-ros2-ws
```

O repositorio esta privado. Para clonar em outro computador ou Raspberry Pi,
use uma chave SSH cadastrada no GitHub ou um Personal Access Token com acesso a
este repositorio.

## Visao geral

Workspace ROS 2 para controlar o braco robotico reBotArm.

Pacotes principais:

| Pacote | Funcao |
| --- | --- |
| `rebotarm_msgs` | Mensagens, servicos e actions customizadas do projeto. |
| `rebotarmcontroller` | No ROS 2 que conversa com o SDK do hardware e expoe topicos, servicos e actions. |
| `rebotarm_bringup` | Launch files, parametros, URDFs, meshes e configuracao de hardware. |
| `rebotarm_moveit_config` | Configuracao MoveIt 2 para planejamento e RViz. |
| `rebotarm_moveit_demos` | Demos de movimento usando MoveIt 2. |
| `rebotarm_gui` | GUI PySide6/Qt e o simulador `mock_arm` para testes sem hardware. |

Interfaces principais no namespace padrao `/rebotarm`:

| Tipo | Interface |
| --- | --- |
| Estado | `/rebotarm/joint_states`, `/rebotarm/arm_status`, `/rebotarm/gripper/state` |
| Estado detalhado | `/rebotarm/joints/<joint>/state` |
| Comando bruto | `/rebotarm/joints/<joint>/cmd/mit`, `/rebotarm/joints/<joint>/cmd/pos_vel` |
| Servicos | `/rebotarm/enable`, `/rebotarm/disable`, `/rebotarm/safe_home`, `/rebotarm/set_zero` |
| Gripper | `/rebotarm/gripper/open`, `/rebotarm/gripper/close`, `/rebotarm/gripper/set`, `/rebotarm/gripper/command` |
| Actions | `/rebotarm/move_to_pose`, `/rebotarm/follow_joint_trajectory` |

Modelos configurados em `src/rebotarm_bringup/config/rebotarm_hardware.yaml`:

| Modelo | Canal padrao | Observacao |
| --- | --- | --- |
| `dm` | `/dev/ttyACM0` | reBotArm B601-DM, Damiao USB2CAN serial bridge, baud 921600 no SDK. |
| `rs` | `can0` | reBotArm RS, RobStride via SocketCAN. |

## Requisitos recomendados no Raspberry Pi

- Raspberry Pi 4/5, preferencialmente com 4 GB ou mais de RAM.
- Cartao microSD rapido ou SSD USB.
- Ubuntu Server/Desktop 24.04 arm64 para ROS 2 Jazzy, ou Ubuntu 22.04 arm64 para ROS 2 Humble.
- Fonte de alimentacao adequada para o Pi e alimentacao separada/adequada para o robo.
- Acesso SSH habilitado.
- Rede local estavel se a GUI rodar em outra maquina.

Use a mesma distribuicao ROS 2 no Pi e na maquina que rodar a GUI. Neste
workspace local foi usado `/opt/ros/lyrical`, mas o script `setup_all.sh`
detecta Ubuntu 22.04 como Humble e Ubuntu 24.04 como Jazzy. Se o Pi estiver em
outra versao de Ubuntu/ROS, revise o valor de `ROS_DISTRO` antes de instalar.

## Preparar o Raspberry Pi

Atualize o sistema:

```bash
sudo apt update
sudo apt upgrade -y
sudo apt install -y git curl wget lsb-release ca-certificates \
  build-essential cmake python3-pip python3-venv
```

Opcional, mas util para acesso remoto:

```bash
sudo apt install -y openssh-server tmux htop
sudo systemctl enable --now ssh
```

Configure grupos de usuario para acesso a dispositivos seriais:

```bash
sudo usermod -aG dialout $USER
sudo usermod -aG plugdev $USER
```

Depois desse comando, faca logout/login ou reinicie o Pi.

## Carregar o projeto no Pi a partir do GitHub

O repositorio ja contem a estrutura completa do workspace ROS 2. No Raspberry
Pi, clone o projeto diretamente como `~/ros2_ws`.

### Opcao A: HTTPS com token

```bash
cd ~
git clone https://github.com/wesleycatao/rebotarm-ros2-ws.git ros2_ws
```

Quando o Git pedir credenciais:

- Username: `wesleycatao`
- Password: cole um Personal Access Token do GitHub, nao a senha da conta

O token precisa ter permissao de leitura para este repositorio privado. Para
atualizar o repositorio a partir do Pi no futuro, ele tambem precisa de
`Contents: Read and write`.

### Opcao B: SSH

Configure uma chave SSH no Pi e adicione a chave publica em:

```text
https://github.com/settings/keys
```

Exemplo:

```bash
ssh-keygen -t ed25519 -C "raspberry-pi-rebotarm"
cat ~/.ssh/id_ed25519.pub
```

Depois de cadastrar a chave no GitHub:

```bash
cd ~
git clone git@github.com:wesleycatao/rebotarm-ros2-ws.git ros2_ws
```

### Opcao C: copiar desta maquina por SSH

Use esta opcao apenas se nao quiser clonar pelo GitHub:

```bash
rsync -av --exclude build --exclude install --exclude log \
  /home/wesley/ros2_ws/ usuario@IP_DO_PI:~/ros2_ws/
```

No Pi:

```bash
cd ~/ros2_ws
chmod +x setup_all.sh
```

Nao copie `build/`, `install/` e `log/` entre maquinas. Essas pastas sao
artefatos locais de compilacao e devem ser recriadas no Raspberry Pi.

## Instalacao rapida

O projeto possui um instalador na raiz:

```bash
cd ~/ros2_ws
./setup_all.sh
```

O script faz, em linhas gerais:

1. Detecta a versao do Ubuntu.
2. Instala ROS 2.
3. Instala ferramentas de build como `colcon` e `rosdep`.
4. Instala dependencias ROS do controlador, MoveIt, RViz e mensagens.
5. Instala PySide6 e dependencias Qt.
6. Clona o SDK `reBotArm_control_py`.
7. Configura `.bashrc`.
8. Executa `rosdep`.
9. Compila o workspace com `colcon build --symlink-install`.

Ao terminar, abra um novo terminal ou rode:

```bash
source ~/.bashrc
source ~/ros2_ws/install/setup.bash
```

## Instalacao manual

Use este caminho se quiser controlar cada etapa ou se o script falhar.

Defina a distro ROS 2:

```bash
# Ubuntu 24.04
export ROS_DISTRO=jazzy

# Ubuntu 22.04
# export ROS_DISTRO=humble
```

Adicione o repositorio ROS 2:

```bash
sudo apt install -y software-properties-common curl
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
```

Instale ROS 2 e ferramentas:

```bash
sudo apt install -y ros-${ROS_DISTRO}-desktop \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-argcomplete
```

Instale dependencias ROS do projeto:

```bash
sudo apt install -y \
  ros-${ROS_DISTRO}-controller-manager \
  ros-${ROS_DISTRO}-joint-state-broadcaster \
  ros-${ROS_DISTRO}-joint-trajectory-controller \
  ros-${ROS_DISTRO}-ros2-control \
  ros-${ROS_DISTRO}-ros2-controllers \
  ros-${ROS_DISTRO}-moveit-configs-utils \
  ros-${ROS_DISTRO}-moveit-kinematics \
  ros-${ROS_DISTRO}-moveit-planners-ompl \
  ros-${ROS_DISTRO}-moveit-ros-move-group \
  ros-${ROS_DISTRO}-moveit-ros-visualization \
  ros-${ROS_DISTRO}-moveit-simple-controller-manager \
  ros-${ROS_DISTRO}-tf-transformations \
  ros-${ROS_DISTRO}-tf2-ros \
  ros-${ROS_DISTRO}-robot-state-publisher \
  ros-${ROS_DISTRO}-xacro \
  ros-${ROS_DISTRO}-rviz2 \
  ros-${ROS_DISTRO}-launch-ros \
  ros-${ROS_DISTRO}-launch \
  ros-${ROS_DISTRO}-ament-index-python \
  ros-${ROS_DISTRO}-moveit-msgs \
  ros-${ROS_DISTRO}-shape-msgs \
  ros-${ROS_DISTRO}-control-msgs \
  ros-${ROS_DISTRO}-trajectory-msgs \
  ros-${ROS_DISTRO}-sensor-msgs \
  ros-${ROS_DISTRO}-geometry-msgs \
  ros-${ROS_DISTRO}-std-msgs \
  ros-${ROS_DISTRO}-std-srvs \
  ros-${ROS_DISTRO}-builtin-interfaces
```

Instale dependencias Python/Qt:

```bash
sudo apt install -y \
  libgl1 libglib2.0-0 libfontconfig1 libdbus-1-3 \
  libxkbcommon-x11-0 libxcb-cursor0 libxcb-xinerama0 \
  libxcb-icccm4 libxcb-image0 libxcb-keysyms1 \
  libxcb-randr0 libxcb-render-util0 libxcb-shape0 \
  libxcb-xfixes0 libxcb-sync1

pip3 install --user PySide6 numpy PyYAML
```

Instale o SDK do hardware:

```bash
cd ~/ros2_ws
mkdir -p src/rebotarmcontroller/third_party
git clone https://github.com/vectorBH6/reBotArm_control_py.git \
  src/rebotarmcontroller/third_party/reBotArm_control_py

mkdir -p third_party
ln -sfn ~/ros2_ws/src/rebotarmcontroller/third_party/reBotArm_control_py \
  ~/ros2_ws/third_party/reBotArm_control_py
```

Configure o ambiente no `.bashrc`:

```bash
echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash 2>/dev/null || true" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=30" >> ~/.bashrc
```

Abra um novo terminal, ou rode:

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
```

Inicialize `rosdep`:

```bash
sudo rosdep init 2>/dev/null || true
rosdep update
```

## Build

Na raiz do workspace:

```bash
cd ~/ros2_ws
source /opt/ros/${ROS_DISTRO}/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Para rebuildar apenas a GUI:

```bash
colcon build --packages-select rebotarm_gui --symlink-install
source install/setup.bash
```

Para rebuildar apenas mensagens e controlador:

```bash
colcon build --packages-select rebotarm_msgs rebotarmcontroller rebotarm_bringup --symlink-install
source install/setup.bash
```

Validacoes basicas:

```bash
ros2 pkg list | grep rebotarm
ros2 pkg executables rebotarm_gui
ros2 pkg executables rebotarmcontroller
```

Saida esperada para a GUI:

```text
rebotarm_gui mock_arm
rebotarm_gui rebotarm_gui
```

## Teste sem hardware

Terminal 1:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run rebotarm_gui mock_arm
```

Terminal 2:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run rebotarm_gui rebotarm_gui
```

Validar topicos:

```bash
ros2 node list
ros2 topic echo /rebotarm/joint_states --once
ros2 topic echo /rebotarm/arm_status --once
```

## Executar com hardware real

Antes de energizar e comandar o robo, deixe a area livre e garanta que o braco
possa se mover sem colisao.

### Modelo DM via USB serial

Verifique se o USB2CAN apareceu:

```bash
ls /dev/ttyACM*
```

Permissao temporaria, se necessario:

```bash
sudo chmod 666 /dev/ttyACM0
```

Rodar apenas o driver:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch rebotarm_bringup driver.launch.py model:=dm channel:=/dev/ttyACM0
```

Rodar bringup com `robot_state_publisher` e sem RViz:

```bash
ros2 launch rebotarm_bringup bringup.launch.py \
  model:=dm \
  channel:=/dev/ttyACM0 \
  use_rviz:=false
```

### Modelo RS via SocketCAN

Instale utilitarios CAN:

```bash
sudo apt install -y can-utils
```

Suba a interface CAN:

```bash
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 up type can bitrate 500000
ip -details link show can0
```

Rodar:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch rebotarm_bringup bringup.launch.py \
  model:=rs \
  channel:=can0 \
  use_rviz:=false
```

## GUI no Pi ou em outra maquina

### GUI local no Pi

Requer ambiente grafico local, VNC, X11 forwarding ou Wayland funcional:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run rebotarm_gui rebotarm_gui --namespace rebotarm
```

### GUI em notebook e controlador no Pi

Use a mesma distro ROS 2, o mesmo `ROS_DOMAIN_ID` e a mesma rede.

No Pi:

```bash
export ROS_DOMAIN_ID=30
ros2 launch rebotarm_bringup bringup.launch.py model:=dm channel:=/dev/ttyACM0 use_rviz:=false
```

No notebook:

```bash
export ROS_DOMAIN_ID=30
source ~/ros2_ws/install/setup.bash
ros2 node list
ros2 run rebotarm_gui rebotarm_gui --namespace rebotarm
```

Se a descoberta ROS 2 falhar entre maquinas, verifique firewall, VPNs, sub-redes
diferentes e se ambas usam a mesma implementacao DDS/RMW.

## MoveIt e demos

Demo visual:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch rebotarm_moveit_config demo.launch.py
```

Hardware com MoveIt:

```bash
ros2 launch rebotarm_moveit_config hardware.launch.py
```

Demos:

```bash
ros2 launch rebotarm_moveit_demos draw_square.launch.py
ros2 launch rebotarm_moveit_demos pick_place.launch.py
```

Em Raspberry Pi, RViz e MoveIt podem ser pesados. Para uso headless, prefira
rodar planejamento/visualizacao em uma maquina mais forte e manter o Pi como
controlador do hardware.

## Comandos uteis

Listar pacotes:

```bash
ros2 pkg list | grep rebotarm
```

Verificar topicos:

```bash
ros2 topic list
ros2 topic hz /rebotarm/joint_states
```

Verificar servicos:

```bash
ros2 service list | grep rebotarm
```

Testar enable/disable:

```bash
ros2 service call /rebotarm/enable std_srvs/srv/Trigger {}
ros2 service call /rebotarm/disable std_srvs/srv/Trigger {}
```

Limpar e rebuildar, somente se precisar:

```bash
rm -rf build install log
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Troubleshooting

### `Package 'rebotarm_gui' not found`

Rode:

```bash
cd ~/ros2_ws
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build --packages-select rebotarm_gui --symlink-install
source install/setup.bash
ros2 pkg prefix rebotarm_gui
ros2 pkg executables rebotarm_gui
```

Confira se `src/rebotarm_gui/package.xml` contem:

```xml
<buildtool_depend>ament_python</buildtool_depend>
<export>
  <build_type>ament_python</build_type>
</export>
```

### SDK nao encontrado

Erro esperado:

```text
Cannot find reBotArm_control_py
```

Corrija:

```bash
cd ~/ros2_ws
mkdir -p src/rebotarmcontroller/third_party third_party
git clone https://github.com/vectorBH6/reBotArm_control_py.git \
  src/rebotarmcontroller/third_party/reBotArm_control_py
ln -sfn ~/ros2_ws/src/rebotarmcontroller/third_party/reBotArm_control_py \
  ~/ros2_ws/third_party/reBotArm_control_py
```

### Sem acesso a `/dev/ttyACM0`

```bash
ls -l /dev/ttyACM0
sudo usermod -aG dialout $USER
sudo chmod 666 /dev/ttyACM0
```

Faca logout/login depois de adicionar o usuario ao grupo `dialout`.

### `can0` nao aparece

```bash
ip link
sudo apt install -y can-utils
sudo ip link set can0 up type can bitrate 500000
ip -details link show can0
```

Confira tambem se o adaptador USB-CAN foi reconhecido pelo kernel:

```bash
dmesg | grep -i can
```

### GUI nao abre no Pi

Confirme display grafico:

```bash
echo $DISPLAY
echo $WAYLAND_DISPLAY
```

Se estiver em SSH puro, rode a GUI em outra maquina ou use VNC/X11 forwarding.

### Nos ROS nao se enxergam pela rede

Em todas as maquinas:

```bash
export ROS_DOMAIN_ID=30
ros2 node list
```

Verifique se estao na mesma rede, sem VPN bloqueando multicast DDS. Se precisar,
teste primeiro tudo na mesma maquina com `mock_arm`.

## Checklist para uma instalacao futura no Pi

1. Instalar Ubuntu arm64 no Raspberry Pi.
2. Atualizar sistema e habilitar SSH.
3. Clonar `wesleycatao/rebotarm-ros2-ws` em `~/ros2_ws`.
4. Rodar `./setup_all.sh` ou seguir a instalacao manual.
5. Garantir o SDK em `~/ros2_ws/third_party/reBotArm_control_py`.
6. Compilar com `colcon build --symlink-install`.
7. Testar `mock_arm` e GUI.
8. Configurar permissao de `/dev/ttyACM0` ou `can0`.
9. Rodar `rebotarm_bringup` com o modelo correto.
10. Rodar a GUI localmente ou em outro computador na mesma rede.
