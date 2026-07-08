# reBotArm GUI

Interface gráfica (PySide6/Qt) para o controlador ROS 2 do braço robótico **reBotArm**.

A GUI consome as interfaces expostas no namespace `/rebotarm` (tópicos, serviços e actions) e oferece um painel único para comandar o braço, o gripper e monitorar telemetria.

## Requisitos

- ROS 2 (Iron/Jazzy/Rolling)
- Pacotes do workspace: `rebotarm_msgs`, `rebotarm_bringup`, `rebotarmcontroller`
- **PySide6** (biblioteca Qt para Python)

Instale o PySide6 no ambiente ROS 2:

```bash
pip install PySide6
```

## Compilação

No workspace ROS 2 que contém a pasta `src/rebotarm_gui`:

```bash
colcon build --packages-select rebotarm_gui
source install/setup.bash
```

## Execução

### Direto pelo entry point

```bash
ros2 run rebotarm_gui rebotarm_gui
```

### Via launch (com namespace customizado)

```bash
ros2 launch rebotarm_gui gui.launch.py namespace:=rebotarm
```

### Parâmetros

| Parâmetro    | Default     | Descrição                                  |
|--------------|-------------|--------------------------------------------|
| `namespace`  | `rebotarm`  | Namespace das interfaces do braço          |

## Abas

| Aba          | Função                                                                 |
|--------------|------------------------------------------------------------------------|
| **Sistema**  | Status (enable/mode/model/rate/channel), Enable/Disable/Safe Home, Gravity ON/OFF, Set Zero |
| **Juntas**   | Sliders por junta + FollowJointTrajectory (com duração)                |
| **Pose**     | XYZ + RPY/Quaternion + duração → MoveToPose (action) / MoveToPoseIK    |
| **Gripper**  | Estado do gripper + Open/Close/Set(service)/Command(action)            |
| **Telemetria**| Tabelas live de `joint_states` e detalhe por motor (`/joints/<j>/state`) |
| **MIT / Raw**| Publicação bruta de MIT (`kp/kd/tau`) ou pos-vel (`pos/vlim`) por junta |

## Arquitetura

```
┌──────────────────────────── Qt GUI (main thread) ─────────────────────────────┐
│  MainWindow                                                                  │
│   ├─ SystemTab / JointsTab / PoseTab / GripperTab / TelemetryTab / MitTab    │
│   └─ widgets (StatusLed, JointSlider, PoseInput)                             │
│                                                                              │
│            ▲ Qt signals (thread-safe)                                        │
│            │                                                                 │
│   ┌────────┴─────────── RosBridge (QObject) ─────────┐                       │
│   │  QThread + rclpy.spin (MultiThreadedExecutor)   │                       │
│   │   subs / clients / action clients / publishers  │                       │
│   └──────────────────────────────────────────────────┘                       │
└──────────────────────────────────────────────────────────────────────────────┘
```

- O `RosBridge` roda um `rclpy` node em uma `QThread` separada e emite **Qt signals** para atualizar a UI com segurança entre threads.
- Nenhuma chamada ROS bloqueia a thread da UI; as respostas de serviços/ações chegam via sinais `service_result` / `action_result`.

## Estrutura de arquivos

```
rebotarm_gui/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/rebotarm_gui
├── launch/gui.launch.py
└── rebotarm_gui/
    ├── __init__.py
    ├── main.py            # entry point (--namespace)
    ├── app.py             # MainWindow + estilo dark
    ├── ros_bridge.py      # rclpy node + QThread + Qt signals
    ├── styles/dark.qss    # tema Catppuccin Mocha-ish
    ├── widgets/
    │   ├── status_led.py
    │   ├── joint_slider.py
    │   └── pose_input.py
    └── tabs/
        ├── system_tab.py
        ├── joints_tab.py
        ├── pose_tab.py
        ├── gripper_tab.py
        ├── telemetry_tab.py
        └── mit_tab.py
```

## Testes sem hardware (Mock Arm)

O pacote inclui um **mock node** (`mock_arm.py`) que simula o braço completo (6 juntas + gripper), publicando todos os tópicos e respondendo a todos os serviços/ações. Isso permite testar a GUI **sem o braço físico**.

### Rodar o mock

```bash
# Terminal 1 — simulador
ros2 run rebotarm_gui mock_arm
```

```bash
# Terminal 2 — a GUI
ros2 run rebotarm_gui rebotarm_gui
```

A GUI deve conectar imediatamente: LED verde, sliders aparecem, telemetria flui, e todos os botões funcionam (com resposta simulada).

---

## Testando no WSL 2 (Windows)

Sim! Você pode rodar a GUI no **WSL 2 com Ubuntu**, usando o **WSLg** (Windows Subsystem for Linux GUI), que já vem embutido no Windows 11.

> 🆘 **Nunca usou Linux?** Leia o **[GUIA_WSL.md](./GUIA_WSL.md)** — tem explicação detalhada de cada comando, conceitos básicos (terminal, sudo, apt, source) e um script de instalação automatizada (`scripts/setup_wsl.sh`) que faz tudo sozinho.

### Pré-requisitos no Windows

1. **WSL 2 + Ubuntu** instalado:
   ```powershell
   wsl --install -d Ubuntu
   ```
2. **WSLg** já está incluso no Windows 11 (sem necessidade de servidor X separado).

### Instalar ROS 2 no Ubuntu (WSL)

```bash
# Dentro do WSL (Ubuntu 22.04 → ROS 2 Humble; 24.04 → Jazzy)
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-desktop  # ou ros-jazzy-desktop
```

### Instalar PySide6 e dependências da GUI

```bash
# Dentro do WSL
pip install PySide6
# Dependências de sistema para Qt no WSLg
sudo apt install -y libgl1 libglib2.0-0 libfontconfig1 libdbus-1-3 libxkbcommon-x11-0 libxcb-cursor0
```

### Compilar e rodar dentro do WSL

```bash
# Copie o projeto para o WSL (ou acesse via /mnt/c/...)
cd ~/ros2_ws/src
# (coloque os pacotes aqui)

cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select rebotarm_msgs rebotarm_gui
source install/setup.bash

# Terminal 1 — mock
ros2 run rebotarm_gui mock_arm

# Terminal 2 — GUI (abre como janela nativa no Windows via WSLg)
ros2 run rebotarm_gui rebotarm_gui
```

A janela da GUI aparecerá como uma **janela nativa no seu desktop Windows**, graças ao WSLg.

### Desenvolvimento com VS Code + WSL (recomendado)

O tutorial oficial da Microsoft ([VS Code + WSL](https://learn.microsoft.com/en-us/windows/wsl/tutorials/wsl-vscode)) descreve o fluxo ideal — que é perfeito para este projeto:

1. **Instale o VS Code no Windows** (não dentro do WSL) e marque a opção *Add to PATH* durante a instalação.
2. **Instale a extensão WSL** no VS Code (ou o *Remote Development extension pack*, que inclui WSL + SSH + Dev Containers).
3. **Abra o projeto dentro do WSL**:
   ```bash
   # No terminal do WSL, dentro da pasta do projeto:
   code .
   ```
   Ou pelo VS Code: `Ctrl+Shift+P` → digite `WSL` → **"Reopen Folder in WSL"**.

   O VS Code roda em arquitetura **cliente-servidor**: a UI fica no Windows, mas o servidor (código, terminal, tasks, debug) roda dentro do WSL — exatamente onde o ROS 2 está instalado.

4. **Extensões importantes para instalar no WSL** (o VS Code mostrará um botão verde "Install in WSL"):
   - **Python** (Microsoft) — para autocomplete, linting e debug
   - **C/C++** (Microsoft) — para os pacotes C++ do workspace
   - **CMake Tools** — se for depurar builds CMake

5. **Múltiplos terminais no VS Code**: use o terminal integrado (`Ctrl+Shift+\``) com múltiplas abas/splits para rodar simultaneamente:
   - Aba 1: `ros2 run rebotarm_gui mock_arm`
   - Aba 2: `ros2 run rebotarm_gui rebotarm_gui`
   - Aba 3: `ros2 topic echo /rebotarm/joint_states` (inspeção)

6. **Tasks do VS Code** (opcional): crie `.vscode/tasks.json` para automatizar build + source + run. Exemplo:
   ```json
   {
     "version": "2.0.0",
     "tasks": [
       {
         "label": "colcon build (gui + msgs)",
         "type": "shell",
         "command": "colcon build --packages-select rebotarm_msgs rebotarm_gui && source install/setup.bash",
         "group": "build"
       },
       {
         "label": "mock arm",
         "type": "shell",
         "command": "source install/setup.bash && ros2 run rebotarm_gui mock_arm",
         "isBackground": true
       },
       {
         "label": "launch GUI",
         "type": "shell",
         "command": "source install/setup.bash && ros2 run rebotarm_gui rebotarm_gui",
         "isBackground": true
       }
     ]
   }
   ```

> **Dica de performance:** mantenha o código dentro do filesystem do WSL (`~/ros2_ws/`), não em `/mnt/c/...` — o acesso via `/mnt/c` é significativamente mais lento para builds (`colcon`), conforme recomendado pela Microsoft.

### Dicas de troubleshooting no WSL

| Problema | Solução |
|---|---|
| `qt.qpa.plugin: Could not load the Qt platform plugin "xcb"` | `sudo apt install -y libxcb-xinerama0 libxcb-cursor0 libxkbcommon-x11-0` |
| `Could not initialize GLX` | Verifique se está usando WSL2 (não WSL1): `wsl -l -v` |
| Tela preta / renderização lenta | `export QT_QUICK_BACKEND=software` antes de rodar a GUI |
| `DISPLAY` não definido | WSLg define automaticamente; se não: `export DISPLAY=:0` |

---

## Limitações / Avisos

- O braço precisa estar publicado via `rebotarm_bringup` antes de usar a GUI.
- A aba **MIT / Raw** publica comandos de baixo nível diretamente — use com cuidado e com o braço habilitado em ambiente seguro.
- A pose atual do TCP na aba **Pose** não é computada pela GUI (requer FK); o campo é informativo.