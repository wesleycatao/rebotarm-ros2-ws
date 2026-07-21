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
| `rebotarm_vision` | Webcam/imagem fixa, segmentacao OpenCV, limite operacional, pose de objetos e fila de coleta. |

Interfaces principais no namespace padrao `/rebotarm`:

| Tipo | Interface |
| --- | --- |
| Estado | `/rebotarm/joint_states`, `/rebotarm/arm_status`, `/rebotarm/gripper/state` |
| Estado detalhado | `/rebotarm/joints/<joint>/state` |
| Comando bruto | `/rebotarm/joints/<joint>/cmd/mit`, `/rebotarm/joints/<joint>/cmd/pos_vel` |
| Servicos | `/rebotarm/enable`, `/rebotarm/disable`, `/rebotarm/safe_home`, `/rebotarm/set_zero` |
| Gripper | `/rebotarm/gripper/open`, `/rebotarm/gripper/close`, `/rebotarm/gripper/set`, `/rebotarm/gripper/command` |
| Seguranca | `/rebotarm/safety/set_torque_profile`, `/rebotarm/safety/set_step`, `/rebotarm/safety/check_torque_profile`, `/rebotarm/safety/reset_fault` |
| Actions | `/rebotarm/move_to_pose`, `/rebotarm/follow_joint_trajectory` |
| E/S digital (GPIO) | `/rebotarm/dio/state` (publicado pelo no `DioBridge` no Pi) |
| Visao | `/camera/image_raw`, `/rebotarm/vision/debug_image`, `/rebotarm/vision/objects_2d`, `/rebotarm/vision/objects`, `/rebotarm/vision/pick_queue`, `/rebotarm_vision_calibration/progress` |
| Visao (servicos) | `/rebotarm_segmentation/process_frame`, `/rebotarm_vision_calibration/calibrate` |

> **Documentacao relacionada:**
> - Detalhes do pipeline de visao: [`src/rebotarm_vision/README.md`](src/rebotarm_vision/README.md)
> - Uso pela interface grafica: [`src/rebotarm_gui/README.md`](src/rebotarm_gui/README.md)

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

## Comecar com um Raspberry Pi zerado

Use este caminho quando o Raspberry Pi estiver virgem, sem ROS, sem Git e sem
dependencias do projeto.

### 1. Gravar o sistema operacional

Em outro computador, use o Raspberry Pi Imager ou ferramenta equivalente.

Imagem recomendada:

```text
Ubuntu Server 24.04 LTS arm64
```

Essa imagem sera usada com ROS 2 Jazzy. Se voce precisar usar ROS 2 Humble,
grave `Ubuntu Server 22.04 LTS arm64`.

Durante a gravacao, configure:

- Usuario e senha do Pi.
- Hostname, por exemplo `rebotarm-pi`.
- Wi-Fi ou rede cabeada.
- SSH habilitado.
- Locale/teclado conforme sua preferencia.

Se voce gravar Raspberry Pi OS/Debian em vez de Ubuntu, os comandos de
instalacao do ROS 2 via `apt` deste documento podem nao funcionar. Para este
projeto, prefira Ubuntu arm64.

### 2. Primeiro boot e acesso

Ligue o Pi, aguarde o primeiro boot e acesse por SSH:

```bash
ssh usuario@IP_DO_PI
```

Exemplo:

```bash
ssh wesley@192.168.1.50
```

Confirme sistema e arquitetura:

```bash
lsb_release -a
uname -m
```

Saida esperada:

- Ubuntu `24.04` ou `22.04`.
- Arquitetura `aarch64`.

### 3. Atualizar o sistema base

Em um Pi novo, faca a atualizacao antes de clonar ou instalar o projeto:

```bash
sudo apt update
sudo apt upgrade -y
sudo reboot
```

Depois do reboot, entre novamente por SSH:

```bash
ssh usuario@IP_DO_PI
```

### 4. Instalar ferramentas minimas

Esses pacotes sao necessarios antes do clone do repositorio e antes do
`setup_all.sh`:

```bash
sudo apt update
sudo apt install -y git curl wget lsb-release ca-certificates \
  build-essential cmake python3-pip python3-venv \
  gnupg software-properties-common openssh-client
```

Se voce quiser administrar o Pi por SSH e manter sessoes longas durante build:

```bash
sudo apt install -y openssh-server tmux htop
sudo systemctl enable --now ssh
```

## Preparar permissoes do Raspberry Pi

Configure grupos de usuario para acesso a dispositivos seriais:

```bash
sudo usermod -aG dialout $USER
sudo usermod -aG plugdev $USER
```

Depois desse comando, faca logout/login ou reinicie o Pi. Em um Pi recem
instalado, reiniciar neste ponto evita problemas de permissao mais tarde:

```bash
sudo reboot
```

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

Para criar um token:

```text
https://github.com/settings/tokens
```

Use token apenas no terminal. Nao salve o token dentro do repositorio, README,
scripts ou historico de comandos compartilhado.

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

Teste a conexao SSH com o GitHub, se precisar:

```bash
ssh -T git@github.com
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
chmod +x setup_all.sh
./setup_all.sh
```

Esse comando precisa de internet e vai pedir senha de `sudo` para instalar ROS
2, dependencias do sistema, pacotes Python e ferramentas de build. Em um
Raspberry Pi virgem, a primeira execucao pode demorar bastante.

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

Se o script adicionou seu usuario a grupos ou se algum dispositivo USB/CAN nao
aparecer com permissao correta, reinicie:

```bash
sudo reboot
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

Para rebuildar apenas a GUI e as dependencias diretas de seguranca:

```bash
colcon build --packages-select rebotarm_msgs rebotarmcontroller rebotarm_gui --symlink-install
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

Para usar `WAIT_FOR_SIGNAL` (entradas digitais do GPIO), rode tambem no Pi o no
`DioBridge` (em outro terminal):

```bash
ros2 run rebotarmcontroller DioBridge --ros-args -p input_pins:=[17,18,27]
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

### Aba Segurança e perfis de torque

A GUI possui a aba **Seguranca** para monitorar torque por etapa de movimento.
O fluxo recomendado e:

1. Gravar execucoes supervisionadas por etapa/payload.
2. Gerar um perfil YAML com tolerancias `WARN` e `FAIL`.
3. Aplicar o perfil primeiro em modo `warn_only`.
4. Depois de validar falsos positivos, usar `stop_on_fail`.

Os percentuais padrao sao:

| Nivel | Default | Acao |
| --- | --- | --- |
| `WARN` | `10%` | Sinaliza em amarelo, nao para. |
| `FAIL` | `20%` | Em `stop_on_fail`, entra em `PROTECTIVE_STOP` apos amostras consecutivas. |

Os servicos de runtime sao:

```bash
ros2 service list | grep /rebotarm/safety
```

Para gravar e gerar um perfil pelo terminal:

```bash
ros2 run rebotarmcontroller TorqueProfile record \
  --duration 20 \
  --program palletizacao_cv \
  --step-id icar \
  --payload-name carga_visao \
  --payload-kg 0.7 \
  --output torque_runs/icar_run1.csv

ros2 run rebotarmcontroller TorqueProfile build \
  torque_runs/icar_run1.csv torque_runs/icar_run2.csv \
  --warn-tolerance-percent 10 \
  --fail-tolerance-percent 20 \
  --output torque_profiles/palletizacao_cv.yaml
```

Para ativar no launch do driver:

```bash
ros2 launch rebotarm_bringup driver.launch.py \
  model:=dm \
  channel:=/dev/ttyACM0 \
  torque_profile_path:=/home/wesley/ros2_ws/torque_profiles/palletizacao_cv.yaml \
  torque_profile_mode:=warn_only \
  torque_profile_step_id:=default
```

Use `torque_profile_mode:=stop_on_fail` apenas depois de validar o perfil em
`warn_only`. Essa protecao por software nao substitui botao fisico de emergencia
nem limites eletricos/mecanicos do robo.

## Visao computacional com camera no Raspberry Pi 5

O fluxo recomendado para este projeto e manter a **camera fisica conectada ao Raspberry Pi 5** e rodar o pipeline de visao diretamente nele. A GUI pode continuar em um notebook na mesma rede ROS 2, consumindo os topicos publicados pelo Pi. Isso evita transferir imagens brutas pela rede e mantem a latencia baixa.

### Conexao fisica da camera

- Use uma camera USB UVC padrao (webcam) ou a camera oficial Raspberry Pi via cabo flexivel.
- No Pi 5, a camera oficial precisa do cabo flat conectado ao conector CSI/CSI2 e do modulo `camera-stack` habilitado (`sudo apt install raspi-config` se necessario).
- Para cameras USB, basta conectar em uma porta USB 3.0 do Pi 5.

Verifique se o dispositivo de video aparece:

```bash
ls /dev/video*
```

Saida esperada (camera USB):

```text
/dev/video0  /dev/video1
```

### Permissoes de camera no Pi

Adicione o usuario aos grupos `video` e `plugdev`:

```bash
sudo usermod -aG video $USER
sudo usermod -aG plugdev $USER
```

Faca logout/login ou reinicie para aplicar. Em caso de duvida, reinicie:

```bash
sudo reboot
```

### Dependencias de visao no Pi

O `setup_all.sh` ja instala `python3-opencv` e `python3-numpy`. Para confirmar:

```bash
sudo apt install -y python3-opencv python3-numpy v4l-utils
```

Teste rapido da camera (sem ROS):

```bash
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video0 --all | grep -i "pixel format\|framesize"
```

### Rodar o pipeline de visao no Pi

A principio **nao sera usado YOLO**. A segmentacao e feita com OpenCV classico (brilho, bordas, HSV e contornos), configuravel pelo arquivo `src/rebotarm_vision/config/vision.yaml`.

No Pi 5, inicie apenas os nos de camera, segmentacao, pose e scheduler. Deixe o executor desligado (`use_executor:=false`) para que o programa de coleta seja construido e disparado pela GUI/notebook:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch rebotarm_vision vision.launch.py \
  use_camera:=true \
  use_static_image:=false \
  use_calibration:=true \
  use_executor:=false
```

Parametros uteis para ajuste no launch:

| Parametro | Padrao | Descricao |
| --- | --- | --- |
| `use_camera` | `true` | Habilita o no `camera_node`. |
| `use_static_image` | `false` | Usa uma imagem fixa em vez da webcam. |
| `static_image_path` | `""` | Caminho da imagem fixa (se `use_static_image:=true`). |
| `use_calibration` | `false` | Habilita o no `vision_calibration_node` para auto-calibracao por IA. |
| `use_executor` | `true` | Habilita o no que executa pega automatica. No Pi, prefira `false` e controle pela GUI. |

Para uma camera USB com resolucao diferente, ajuste em `config/vision.yaml`:

```yaml
rebotarm_webcam:
  ros__parameters:
    camera_index: 0
    width: 640
    height: 480
    fps: 15.0
```

### Performance no Raspberry Pi 5

- Mantenha a resolucao em 640x480 ou menor. Resolucoes maiores aumentam muito o tempo de segmentacao.
- Use `process_mode: manual` em `config/vision.yaml`. A segmentacao so roda quando a GUI ou o programa de movimento solicita (`Processar 1 frame`).
- Limite `max_processing_fps` a `1.0` ou menos durante o ajuste fino.
- Evite processamento continuo enquanto o braco se move.
- O Pi 5 roda OpenCV classico sem problemas, mas nao e recomendado rodar modelos de deep-learning (YOLO) nele em producao.

### Visao em modo headless (sem monitor no Pi)

Quando a GUI roda em outra maquina, a imagem de debug ainda e publicada em `/rebotarm/vision/debug_image`. No notebook voce pode visualizar com:

```bash
ros2 run rqt_image_view rqt_image_view /rebotarm/vision/debug_image
```

Ou inspecionar os objetos detectados:

```bash
ros2 topic echo /rebotarm/vision/objects --once
ros2 topic echo /rebotarm/vision/pick_queue --once
```

### Auto-calibracao por IA

A calibracao do perfil de deteccao e feita pelo nó
`vision_calibration_node`. Lance-o com `use_calibration:=true` e defina a chave
via variavel de ambiente `OPENAI_API_KEY`:

```bash
export OPENAI_API_KEY=sk-...
ros2 launch rebotarm_vision vision.launch.py \
  use_camera:=true \
  use_calibration:=true \
  use_executor:=false
```

A GUI chama o servico `/rebotarm_vision_calibration/calibrate`, acompanha o
progresso em `/rebotarm_vision_calibration/progress` e, ao final, exibe a
imagem anotada com BBoxes, orientacao e zona de pega. Perfis aprovados sao
persistidos em `~/.config/rebotarm_vision/profiles/`. A chave nunca e gravada
no perfil.

### Calibracao pixel -> XY no Pi

A calibracao geometrica (camera -> `base_link`) e feita uma vez e persistida em
`config/vision.yaml` (ou em um YAML separado carregado por `--params-file`). O
procedimento:

1. Posicione a camera fixa apontando para a mesa/esteira.
2. Meça 3 ou 4 pontos conhecidos na area util (ex: cantos da mesa).
3. Mova o braco ate cada ponto e anote o XY atual do TCP.
4. Clique os mesmos pontos na imagem (na GUI) ou anote as coordenadas em pixels.
5. Preencha `calibration_pixel_points` e `calibration_xy_points` em `vision.yaml`.
6. Ative `use_workspace_homography: true`.

Exemplo com 4 pontos (homografia):

```yaml
rebotarm_object_pose:
  ros__parameters:
    use_workspace_homography: true
    calibration_pixel_points: [u1, v1, u2, v2, u3, v3, u4, v4]
    calibration_xy_points: [x1, y1, x2, y2, x3, y3, x4, y4]
    table_z: 0.0
```

Se a mesa for circular ou irregular, distribua os pontos de forma equilibrada sobre a area util.

### Fluxo tipico Pi + notebook

1. **No Pi 5:**
   ```bash
   ros2 launch rebotarm_vision vision.launch.py use_camera:=true use_calibration:=true use_executor:=false
   ```
2. **No notebook:**
   ```bash
   export ROS_DOMAIN_ID=30
   source ~/ros2_ws/install/setup.bash
   ros2 run rebotarm_gui rebotarm_gui --namespace rebotarm
   ```
3. Na aba **Visao** da GUI, defina o limite operacional, preencha o prompt do
   cenario e use **Auto-calibrar com IA**. Aprove, repita ou complemente o
   texto conforme o resultado.
4. Processe frames manualmente para validar a deteccao.
5. Na aba **Pegas**, recalcule estrategias e gere o programa de movimento.
6. Execute o programa de movimento; ele consome os topicos de visao publicados pelo Pi.

### Troubleshooting de camera no Pi

| Problema | Solucao |
| --- | --- |
| `/dev/video0` nao existe | Verifique cabo, porta USB e `v4l2-ctl --list-devices`. |
| Aplicacao nao consegue abrir a camera | Confira permissoes (`groups`) e reinicie apos `usermod -aG video`. |
| Imagem muito escura/clara | Ajuste iluminacao ou parametros `brightness_threshold`/`brightness_invert`. |
| Taxa de frames baixa | Reduza resolucao em `vision.yaml` e use `process_mode: manual`. |
| Segmentacao lenta | Limite `max_processing_fps` e evite `continuous`. |

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

1. Gravar Ubuntu Server/Desktop arm64 no Raspberry Pi.
2. Habilitar rede e SSH no primeiro boot.
3. Atualizar o sistema com `sudo apt update`, `sudo apt upgrade -y` e reboot.
4. Instalar ferramentas minimas: `git`, `curl`, `wget`, `ca-certificates`, `build-essential`, `cmake`, `python3-pip`.
5. Configurar acesso ao repositorio privado via token HTTPS ou chave SSH.
6. Clonar `wesleycatao/rebotarm-ros2-ws` em `~/ros2_ws`.
7. Configurar grupos `dialout` e `plugdev`, depois reiniciar.
8. Rodar `./setup_all.sh` ou seguir a instalacao manual.
9. Garantir o SDK em `~/ros2_ws/third_party/reBotArm_control_py`.
10. Compilar com `colcon build --symlink-install`.
11. Testar `mock_arm` e GUI.
12. Configurar permissao de `/dev/ttyACM0` ou `can0`.
13. Configurar permissao de camera (`/dev/video*`) e adicionar usuario ao grupo `video`.
14. Ajustar `src/rebotarm_vision/config/vision.yaml` para a camera e cenario (resolucao, `opencv_mode`, `min_area_px`, calibracao pixel -> XY).
15. Rodar `rebotarm_bringup` com o modelo correto.
16. Rodar o pipeline de visao no Pi (`vision.launch.py` com `use_executor:=false`).
17. Rodar a GUI localmente ou em outro computador na mesma rede.
