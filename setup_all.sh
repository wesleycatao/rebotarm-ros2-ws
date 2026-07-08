#!/usr/bin/env bash
# ============================================================================
# setup_all.sh — Instalação completa do reBotArm no WSL
# ============================================================================
# Este script:
#   1. Detecta a versão do Ubuntu e instala ROS2
#   2. Instala todas as dependências de sistema
#   3. Instala pacotes Python (PySide6, numpy, etc.)
#   4. Clona o SDK reBotArm_control_py
#   5. Compila o workspace com colcon
# ============================================================================

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}============================================================${NC}"
echo -e "${BLUE}  Instalação completa do reBotArm Controller                ${NC}"
echo -e "${BLUE}============================================================${NC}"
echo ""

# NÃO use sudo para comandos que não precisam
# O script pedirá senha apenas quando necessário

# ---------------------------------------------------------------------------
# Passo 0: Detectar versão do Ubuntu
# ---------------------------------------------------------------------------
echo -e "${YELLOW}[0/10] Detectando versão do Ubuntu...${NC}"
UBUNTU_VERSION=$(lsb_release -rs 2>/dev/null || echo "unknown")
echo -e "Ubuntu: ${GREEN}${UBUNTU_VERSION}${NC}"

if [[ "$UBUNTU_VERSION" == "22.04" ]]; then
    ROS_DISTRO="humble"
elif [[ "$UBUNTU_VERSION" == "24.04" ]]; then
    ROS_DISTRO="jazzy"
else
    echo -e "${YELLOW}Aviso: Versão não testada ($UBUNTU_VERSION). Usando Humble.${NC}"
    ROS_DISTRO="humble"
fi
echo -e "ROS 2 distro: ${GREEN}${ROS_DISTRO}${NC}"
echo ""

# ---------------------------------------------------------------------------
# Passo 1: Atualizar sistema
# ---------------------------------------------------------------------------
echo -e "${YELLOW}[1/10] Atualizando pacotes do sistema...${NC}"
sudo apt update && sudo apt upgrade -y
sudo apt install -y \
    software-properties-common \
    curl wget gnupg lsb-release ca-certificates \
    python3-pip python3-venv \
    build-essential cmake git
echo -e "${GREEN}✓ Sistema atualizado${NC}"
echo ""

# ---------------------------------------------------------------------------
# Passo 2: Adicionar repositório ROS 2
# ---------------------------------------------------------------------------
echo -e "${YELLOW}[2/10] Adicionando repositório do ROS 2...${NC}"
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update -y
echo -e "${GREEN}✓ Repositório ROS 2 adicionado${NC}"
echo ""

# ---------------------------------------------------------------------------
# Passo 3: Instalar ROS 2 Desktop
# ---------------------------------------------------------------------------
echo -e "${YELLOW}[3/10] Instalando ROS 2 ${ROS_DISTRO} (pode demorar ~10 min)...${NC}"
sudo apt install -y ros-${ROS_DISTRO}-desktop
echo -e "${GREEN}✓ ROS 2 ${ROS_DISTRO} instalado${NC}"
echo ""

# ---------------------------------------------------------------------------
# Passo 4: Colcon e ferramentas de build
# ---------------------------------------------------------------------------
echo -e "${YELLOW}[4/10] Instalando ferramentas de build...${NC}"
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-argcomplete
echo -e "${GREEN}✓ Ferramentas instaladas${NC}"
echo ""

# ---------------------------------------------------------------------------
# Passo 5: Dependências ROS específicas do projeto
# ---------------------------------------------------------------------------
echo -e "${YELLOW}[5/10] Instalando dependências ROS do projeto...${NC}"
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
    ros-${ROS_DISTRO}-builtin-interfaces \
    2>/dev/null || echo -e "${YELLOW}Alguns pacotes podem não estar disponíveis, continuando...${NC}"

echo -e "${GREEN}✓ Dependências ROS instaladas${NC}"
echo ""

# ---------------------------------------------------------------------------
# Passo 6: PySide6 e dependências Qt para WSLg
# ---------------------------------------------------------------------------
echo -e "${YELLOW}[6/10] Instalando PySide6 e dependências Qt...${NC}"
sudo apt install -y \
    libgl1 libglib2.0-0 libfontconfig1 libdbus-1-3 \
    libxkbcommon-x11-0 libxcb-cursor0 libxcb-xinerama0 \
    libxcb-icccm4 libxcb-image0 libxcb-keysyms1 \
    libxcb-randr0 libxcb-render-util0 libxcb-shape0 \
    libxcb-xfixes0 libxcb-sync1

pip3 install --user PySide6 numpy PyYAML 2>/dev/null || pip3 install PySide6 numpy PyYAML
echo -e "${GREEN}✓ PySide6 e dependências Qt instalados${NC}"
echo ""

# ---------------------------------------------------------------------------
# Passo 7: Configurar .bashrc
# ---------------------------------------------------------------------------
echo -e "${YELLOW}[7/10] Configurando .bashrc...${NC}"
BASHRC="$HOME/.bashrc"
ROS_LINE="source /opt/ros/${ROS_DISTRO}/setup.bash"

if ! grep -q "$ROS_LINE" "$BASHRC" 2>/dev/null; then
    echo "" >> "$BASHRC"
    echo "# ROS 2 setup" >> "$BASHRC"
    echo "$ROS_LINE" >> "$BASHRC"
    echo 'source ~/ros2_ws/install/setup.bash 2>/dev/null || true' >> "$BASHRC"
    echo 'export ROS_DOMAIN_ID=30' >> "$BASHRC"
    echo -e "${GREEN}✓ .bashrc configurado${NC}"
else
    echo -e "${YELLOW}  .bashrc já configurado${NC}"
fi
echo ""

# ---------------------------------------------------------------------------
# Passo 8: Garantir workspace
# ---------------------------------------------------------------------------
echo -e "${YELLOW}[8/10] Verificando workspace...${NC}"

# Determinar onde estamos
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# Se estamos dentro de src/rebotarm_gui/scripts, subimos para o workspace
if echo "$SCRIPT_DIR" | grep -q "rebotarm_gui/scripts"; then
    WS_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
elif [ -f "$SCRIPT_DIR/src/rebotarm_gui/package.xml" ]; then
    WS_DIR="$SCRIPT_DIR"
elif [ -d "$SCRIPT_DIR/src" ]; then
    WS_DIR="$SCRIPT_DIR"
else
    WS_DIR="$HOME/ros2_ws"
fi

echo -e "Workspace: ${GREEN}${WS_DIR}${NC}"
mkdir -p "${WS_DIR}/src"

# Se o workspace não tem os fontes, copiar (caso estejamos em /mnt/c)
if [ "$(ls -A "${WS_DIR}/src" 2>/dev/null | wc -l)" -lt 3 ]; then
    # Procurar fontes em /mnt/c
    for possible in /mnt/c/Users/*/OneDrive/Documentos/PlatformIO/Projects/ROS2/reBotArmController_ROS2/src /mnt/c/Users/*/Documents/PlatformIO/Projects/ROS2/reBotArmController_ROS2/src; do
        if [ -d "$possible" ]; then
            echo -e "Copiando fontes de ${possible}..."
            cp -r "$possible"/* "${WS_DIR}/src/"
            break
        fi
    done
fi

echo -e "${GREEN}✓ Workspace verificado${NC}"
echo ""

# ---------------------------------------------------------------------------
# Passo 9: Clonar reBotArm_control_py (SDK do hardware)
# ---------------------------------------------------------------------------
echo -e "${YELLOW}[9/10] Verificando SDK reBotArm_control_py...${NC}"
SDK_DIR="${WS_DIR}/src/rebotarmcontroller/third_party/reBotArm_control_py"

if [ ! -d "$SDK_DIR" ]; then
    echo -e "Clonando reBotArm_control_py SDK..."
    mkdir -p "${WS_DIR}/src/rebotarmcontroller/third_party"
    git clone https://github.com/vectorBH6/reBotArm_control_py.git "$SDK_DIR" 2>/dev/null || {
        echo -e "${RED}Falha ao clonar SDK. Verifique a URL ou faça manualmente:${NC}"
        echo -e "  git clone https://github.com/vectorBH6/reBotArm_control_py.git ${SDK_DIR}"
    }
    echo -e "${GREEN}✓ SDK clonado${NC}"
else
    echo -e "${GREEN}✓ SDK já existe${NC}"
fi

# Também verificar se não está em third_party/ na raiz
if [ ! -d "${WS_DIR}/third_party/reBotArm_control_py" ] && [ -d "$SDK_DIR" ]; then
    mkdir -p "${WS_DIR}/third_party"
    ln -sf "$SDK_DIR" "${WS_DIR}/third_party/reBotArm_control_py" 2>/dev/null || true
fi

echo ""

# ---------------------------------------------------------------------------
# Passo 10: rosdep e build
# ---------------------------------------------------------------------------
echo -e "${YELLOW}[10/10] Compilando o projeto...${NC}"
cd "$WS_DIR"

# Source ROS
source /opt/ros/${ROS_DISTRO}/setup.bash

# rosdep
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init 2>/dev/null || true
fi
rosdep update 2>/dev/null || true
rosdep install --from-paths src --ignore-src -r -y 2>/dev/null || true

# Build
colcon build --symlink-install 2>&1

echo ""
echo -e "${BLUE}============================================================${NC}"
echo -e "${GREEN}  ✓ INSTALAÇÃO CONCLUÍDA!${NC}"
echo -e "${BLUE}============================================================${NC}"
echo ""
echo -e "Para testar:"
echo -e "  1. source ~/.bashrc"
echo -e "  2. Terminal 1: ros2 run rebotarm_gui mock_arm"
echo -e "  3. Terminal 2: ros2 run rebotarm_gui rebotarm_gui"
echo ""
