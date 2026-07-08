#!/usr/bin/env bash
# ============================================================================
# setup_wsl.sh — Instalação automatizada do ambiente ROS 2 + GUI no WSL
# ============================================================================
# Este script automatiza toda a instalação para iniciantes em Linux.
# Rode dentro do WSL (Ubuntu).
#
# Uso:
#   bash setup_wsl.sh
# ============================================================================

set -e  # Para o script se algum comando falhar

# Cores para output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}============================================================${NC}"
echo -e "${BLUE}  Instalação do ambiente ROS 2 + reBotArm GUI no WSL       ${NC}"
echo -e "${BLUE}============================================================${NC}"
echo ""

# ----------------------------------------------------------------------------
# Passo 0: Detectar versão do Ubuntu
# ----------------------------------------------------------------------------
echo -e "${YELLOW}[Passo 0] Detectando versão do Ubuntu...${NC}"
UBUNTU_VERSION=$(lsb_release -rs 2>/dev/null || echo "")
echo -e "Ubuntu versão: ${GREEN}${UBUNTU_VERSION}${NC}"

if [[ "$UBUNTU_VERSION" == "22.04" ]]; then
    ROS_DISTRO="humble"
elif [[ "$UBUNTU_VERSION" == "24.04" ]]; then
    ROS_DISTRO="jazzy"
else
    echo -e "${YELLOW}Aviso: Versão do Ubuntu não testada ($UBUNTU_VERSION).${NC}"
    echo -e "Recomendado: Ubuntu 22.04 (ROS 2 Humble) ou 24.04 (ROS 2 Jazzy)."
    read -p "Continuar mesmo assim? (s/N): " CONTINUE
    [[ "$CONTINUE" != "s" && "$CONTINUE" != "S" ]] && exit 1
    ROS_DISTRO="humble"
fi

echo -e "ROS 2 distro: ${GREEN}${ROS_DISTRO}${NC}"
echo ""

# ----------------------------------------------------------------------------
# Passo 1: Atualizar pacotes do sistema
# ----------------------------------------------------------------------------
echo -e "${YELLOW}[Passo 1] Atualizando pacotes do sistema...${NC}"
sudo apt update && sudo apt upgrade -y
sudo apt install -y \
    software-properties-common \
    curl \
    wget \
    gnupg \
    lsb-release \
    ca-certificates \
    python3-pip \
    python3-venv \
    build-essential \
    cmake \
    git
echo -e "${GREEN}✓ Sistema atualizado${NC}"
echo ""

# ----------------------------------------------------------------------------
# Passo 2: Adicionar repositório do ROS 2
# ----------------------------------------------------------------------------
echo -e "${YELLOW}[Passo 2] Adicionando repositório do ROS 2...${NC}"
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
echo -e "${GREEN}✓ Repositório ROS 2 adicionado${NC}"
echo ""

# ----------------------------------------------------------------------------
# Passo 3: Instalar ROS 2 Desktop
# ----------------------------------------------------------------------------
echo -e "${YELLOW}[Passo 3] Instalando ROS 2 ${ROS_DISTRO} (isso pode demorar ~10 min)...${NC}"
sudo apt install -y ros-${ROS_DISTRO}-desktop
echo -e "${GREEN}✓ ROS 2 ${ROS_DISTRO} instalado${NC}"
echo ""

# ----------------------------------------------------------------------------
# Passo 4: Instalar colcon e ferramentas ROS
# ----------------------------------------------------------------------------
echo -e "${YELLOW}[Passo 4] Instalando colcon e ferramentas de build...${NC}"
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-argcomplete
echo -e "${GREEN}✓ colcon instalado${NC}"
echo ""

# ----------------------------------------------------------------------------
# Passo 5: Instalar PySide6 e dependências Qt para WSLg
# ----------------------------------------------------------------------------
echo -e "${YELLOW}[Passo 5] Instalando PySide6 e dependências do Qt...${NC}"
# Dependências de sistema para o Qt funcionar no WSLg
sudo apt install -y \
    libgl1 \
    libglib2.0-0 \
    libfontconfig1 \
    libdbus-1-3 \
    libxkbcommon-x11-0 \
    libxcb-cursor0 \
    libxcb-xinerama0 \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-randr0 \
    libxcb-render-util0 \
    libxcb-shape0 \
    libxcb-xfixes0 \
    libxcb-sync1

# PySide6 via pip
pip3 install --user PySide6 || pip3 install PySide6
echo -e "${GREEN}✓ PySide6 e dependências Qt instalados${NC}"
echo ""

# ----------------------------------------------------------------------------
# Passo 6: Configurar .bashrc (source automático do ROS)
# ----------------------------------------------------------------------------
echo -e "${YELLOW}[Passo 6] Configurando .bashrc...${NC}"
BASHRC="$HOME/.bashrc"
ROS_LINE="source /opt/ros/${ROS_DISTRO}/setup.bash"

if ! grep -q "$ROS_LINE" "$BASHRC"; then
    echo "" >> "$BASHRC"
    echo "# ROS 2 setup" >> "$BASHRC"
    echo "$ROS_LINE" >> "$BASHRC"
    echo 'source ~/ros2_ws/install/setup.bash 2>/dev/null || true' >> "$BASHRC"
    echo 'export ROS_DOMAIN_ID=30' >> "$BASHRC"
    echo -e "${GREEN}✓ .bashrc configurado (ROS será carregado automaticamente)${NC}"
else
    echo -e "${YELLOW}  .bashrc já tem o source do ROS${NC}"
fi
echo ""

# ----------------------------------------------------------------------------
# Passo 7: Criar workspace
# ----------------------------------------------------------------------------
echo -e "${YELLOW}[Passo 7] Criando workspace ~/ros2_ws...${NC}"
mkdir -p ~/ros2_ws/src
echo -e "${GREEN}✓ Workspace criado em ~/ros2_ws${NC}"
echo ""

# ----------------------------------------------------------------------------
# Passo 8: Copiar o projeto para o WSL (se estiver em /mnt/c)
# ----------------------------------------------------------------------------
echo -e "${YELLOW}[Passo 8] Verificando cópia do projeto...${NC}"
PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
WS_PROJECT="$HOME/ros2_ws/src/rebotarm_gui"

# Se o script está rodando de /mnt/c/, copiar para o filesystem do WSL
if [[ "$(pwd)" == /mnt/* ]]; then
    # Descobrir a raiz do workspace (2 níveis acima de scripts/)
    WS_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
    WS_ROOT_NAME=$(basename "$WS_ROOT")

    echo -e "  Detectado: você está rodando de ${YELLOW}/mnt/c${NC}"
    echo -e "  Copiando projeto do Windows para o WSL (mais rápido para builds)..."

    # Copiar todos os pacotes do workspace
    SRC_DIR="$WS_ROOT/src"
    if [ -d "$SRC_DIR" ]; then
        cp -r "$SRC_DIR"/* ~/ros2_ws/src/ 2>/dev/null || true
        echo -e "${GREEN}  ✓ Pacotes copiados para ~/ros2_ws/src/${NC}"
    else
        echo -e "${YELLOW}  ! Não encontrei a pasta src. Você precisará copiar manualmente.${NC}"
    fi
else
    echo -e "${GREEN}  ✓ Já está no filesystem do WSL${NC}"
fi
echo ""

# ----------------------------------------------------------------------------
# Passo 9: rosdep init (se ainda não feito)
# ----------------------------------------------------------------------------
echo -e "${YELLOW}[Passo 9] Inicializando rosdep...${NC}"
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init 2>/dev/null || true
fi
rosdep update 2>/dev/null || true
echo -e "${GREEN}✓ rosdep inicializado${NC}"
echo ""

# ----------------------------------------------------------------------------
# Passo 10: Build do projeto
# ----------------------------------------------------------------------------
echo -e "${YELLOW}[Passo 10] Compilando o projeto (primeiro build pode demorar)...${NC}"
cd ~/ros2_ws
source /opt/ros/${ROS_DISTRO}/setup.bash

# Instalar dependências dos pacotes
rosdep install --from-paths src --ignore-src -r -y 2>/dev/null || true

colcon build --symlink-install
echo -e "${GREEN}✓ Build concluído${NC}"
echo ""

# ----------------------------------------------------------------------------
# Resumo final
# ----------------------------------------------------------------------------
echo -e "${BLUE}============================================================${NC}"
echo -e "${GREEN}  ✓ INSTALAÇÃO CONCLUÍDA COM SUCESSO!${NC}"
echo -e "${BLUE}============================================================${NC}"
echo ""
echo -e "Para começar a usar:"
echo ""
echo -e "  ${YELLOW}1.${NC} Feche e reabra o terminal (ou rode: ${GREEN}source ~/.bashrc${NC})"
echo ""
echo -e "  ${YELLOW}2.${NC} Para testar a GUI com o simulador:"
echo ""
echo -e "     ${GREEN}# Terminal 1:${NC}"
echo -e "     ros2 run rebotarm_gui mock_arm"
echo ""
echo -e "     ${GREEN}# Terminal 2:${NC}"
echo -e "     ros2 run rebotarm_gui rebotarm_gui"
echo ""
echo -e "  ${YELLOW}3.${NC} Para abrir no VS Code:"
echo -e "     ${GREEN}cd ~/ros2_ws && code .${NC}"
echo ""
echo -e "${BLUE}============================================================${NC}"