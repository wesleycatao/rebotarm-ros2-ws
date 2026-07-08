# Guia Passo a Passo para Iniciantes em Linux (WSL)

Este guia foi feito para quem **nunca usou Linux**. Vou explicar cada conceito e cada comando com detalhes.

> 💡 **Dica importante:** Leia uma seção inteira antes de executar os comandos. Entenda o *porquê* de cada passo.

---

## Conceitos basicos (leia antes de comecar)

### O que e WSL?
**WSL** (Windows Subsystem for Linux) e como ter um Linux rodando *dentro* do Windows. Voce nao precisa instalar uma maquina virtual pesada nem fazer dual-boot. E como um "aplicativo" que roda Linux nativamente.

### O que e WSLg?
**WSLg** (WSL GUI) e a capacidade do WSL de mostrar **janelas de programas Linux** diretamente no seu desktop Windows. Ou seja, a GUI que roda no Linux aparece como uma janela normal no Windows. Isso ja vem pronto no Windows 11.

### O que e terminal?
O **terminal** e aquela tela preta onde voce digita comandos de texto. No WSL, e onde voce vai digitar `ros2 run ...`, `colcon build`, etc.

### O que e `~` (til)?
Em Linux, `~` significa "sua pasta de usuario" (equivalente a `C:\Users\SeuNome`). Entao `~/ros2_ws` e o mesmo que `/home/seunome/ros2_ws`.

### O que e `source`?
O comando `source` "carrega" configuracoes. Por exemplo, `source /opt/ros/humble/setup.bash` faz o Linux reconhecer os comandos `ros2`. Sem isso, o comando `ros2` nao existe para o terminal.

### O que e `sudo`?
`sudo` significa "Super User DO" - executa um comando como administrador. Sempre que um comando precisar instalar algo no sistema (`apt install`), precisa de `sudo`.

### O que e `apt`?
`apt` e o "gerenciador de pacotes" do Ubuntu. E como uma loja de aplicativos de texto. `sudo apt install nome-do-pacote` baixa e instala um programa.

### O que e `pip`?
`pip` e o instalador de bibliotecas Python. Enquanto `apt` instala programas do Linux, `pip` instala bibliotecas Python (como o PySide6).

---

## Passo 1: Verificar se o WSL esta instalado

Para confirmar, abra o **PowerShell** no Windows e digite:

```powershell
wsl --list --verbose
```

Voce deve ver:
```
NAME      STATE           VERSION
Ubuntu    Running         2
```

> O `VERSION` deve ser **2** (nao 1). Se for 1, converta com:
> ```powershell
> wsl --set-version Ubuntu 2
> ```

---

## Passo 2: Abrir o terminal do WSL (Ubuntu)

1. No Windows, clique no menu **Iniciar**
2. Digite **Ubuntu**
3. Abra o aplicativo "Ubuntu"

Uma tela preta vai aparecer. Esse e o seu terminal Linux.

> **Alternativa:** Use o **Windows Terminal** (ja vem no Windows 11) e clique na setinha para escolher "Ubuntu".

---

## Passo 3: Instalar tudo com o script automatico (recomendado)

Eu criei um script que faz **tudo automaticamente** para voce. Siga estes passos:

### 3.1. Criar a pasta do workspace

No terminal do Ubuntu, digite **um comando por vez** e pressione Enter apos cada um:

```bash
mkdir -p ~/ros2_ws/src
```

> **O que faz:** `mkdir` = "make directory" (criar pasta). `-p` cria pastas pai se nao existirem. `~` = sua home.

### 3.2. Copiar o projeto para dentro do Linux (importante!)

O acesso a `/mnt/c/...` e **lento**. Para compilar rapido, precisamos copiar os arquivos para dentro do Linux:

```bash
cp -r /mnt/c/Users/Wesle/OneDrive/Documentos/PlatformIO/Projects/ROS2/reBotArmController_ROS2/src/* ~/ros2_ws/src/
```

> **O que faz:** `cp` = copy. `-r` = recursivo (copia pastas dentro de pastas). `*` = todos os arquivos.

### 3.3. Rodar o script de instalacao

```bash
cd ~/ros2_ws/src/rebotarm_gui/scripts
bash setup_wsl.sh
```

> **O que faz:** `bash` executa o script. Ele vai instalar ROS 2, PySide6, configurar tudo e compilar o projeto.

**O script vai pedir sua senha** (a senha que voce definiu ao instalar o Ubuntu). Quando voce digita a senha no Linux, **nao aparece nada na tela** (nem asteriscos) - isso e normal. Digite e pressione Enter.

**Paciencia:** Este processo pode demorar de **10 a 30 minutos** dependendo da sua internet. Ele vai baixar ~2GB de pacotes (ROS 2 e grande).

---

## Passo 4: Abrir o VS Code conectado ao WSL

Esta e a parte magica. Voce vai usar o VS Code que ja tem no Windows, mas ele vai "se conectar" ao Linux.

### 4.1. Instalar a extensao WSL no VS Code

1. Abra o VS Code no Windows
2. Va em **Extensions** (Ctrl+Shift+X)
3. Procure por **"WSL"** (da Microsoft)
4. Clique em **Install**

### 4.2. Abrir o projeto no WSL

No terminal do Ubuntu:

```bash
cd ~/ros2_ws
code .
```

> **O que faz:** `code .` abre o VS Code na pasta atual (`.`). Como voce esta no WSL, o VS Code vai abrir conectado ao Linux.

O VS Code vai abrir. No canto inferior esquerdo, voce vera um selo verde **"WSL: Ubuntu"** - isso significa que esta conectado ao Linux.

---

## Passo 5: Testar a GUI com o simulador

Agora vamos testar! Precisamos de **dois terminais** rodando ao mesmo tempo.

### 5.1. Terminal 1: O simulador (mock)

No VS Code, abra o terminal integrado: **Terminal -> New Terminal** (ou Ctrl+Shift+backtick).

```bash
source ~/ros2_ws/install/setup.bash
ros2 run rebotarm_gui mock_arm
```

Voce deve ver:
```
[INFO] Mock arm iniciado - namespace /rebotarm
```

**Deixe este terminal aberto.** O simulador precisa ficar rodando.

### 5.2. Terminal 2: A GUI

No VS Code, clique no icone **+** ao lado do terminal atual (ou Terminal -> Split Terminal) para abrir um segundo terminal.

```bash
source ~/ros2_ws/install/setup.bash
ros2 run rebotarm_gui rebotarm_gui
```

**A janela da GUI deve aparecer no seu desktop Windows!**

Se tudo deu certo, voce vera:
- LED verde (conectado)
- Sliders das juntas se movendo
- Telemetria atualizando em tempo real

---

## Comandos uteis para o dia a dia

| Comando | O que faz | Analogia Windows |
|---------|-----------|------------------|
| `ls` | Lista arquivos da pasta | `dir` |
| `cd pasta` | Entra em uma pasta | Duplo clique na pasta |
| `cd ..` | Volta uma pasta | Voltar no explorer |
| `cd ~` | Vai para sua home | Ir para `C:\Users\SeuNome` |
| `pwd` | Mostra onde voce esta | Barra de endereco |
| `mkdir nome` | Cria uma pasta | Botao direito -> Nova pasta |
| `cp arquivo dest` | Copia arquivo | Ctrl+C / Ctrl+V |
| `mv arquivo dest` | Move ou renomeia | Recortar / Renomear |
| `rm arquivo` | Deleta arquivo | Delete |
| `rm -r pasta` | Deleta pasta inteira | Shift+Delete na pasta |
| `cat arquivo` | Mostra conteudo | Abrir no bloco de notas |
| `clear` | Limpa a tela | `cls` |
| `history` | Mostra comandos digitados | - |
| Ctrl+C | Para o programa atual | Ctrl+C / Ctrl+Break |
| Tab | Autocompleta nomes | - |
| Cima/Baixo | Navega no historico | - |

---

## Como recompilar apos fazer mudancas

Se voce alterar o codigo da GUI, precisa recompilar:

```bash
cd ~/ros2_ws
colcon build --packages-select rebotarm_gui --symlink-install
source install/setup.bash
```

> **Dica:** `--symlink-install` cria links simbolicos em vez de copiar arquivos. Assim, mudancas em arquivos Python sao aplicadas imediatamente (so restart a GUI).

---

## Problemas comuns e solucoes

### "command not found: ros2"
Voce esqueceu de carregar o ambiente ROS:
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

### "ModuleNotFoundError: No module named 'PySide6'"
```bash
pip3 install PySide6
```

### A janela nao abre / erro de Qt
```bash
sudo apt install -y libgl1 libglib2.0-0 libfontconfig1 libdbus-1-3 \
    libxkbcommon-x11-0 libxcb-cursor0 libxcb-xinerama0

ros2 run rebotarm_gui rebotarm_gui
```

### "colcon: command not found"
```bash
sudo apt install python3-colcon-common-extensions
```

### Erro de permissao ao copiar arquivos
```bash
chmod +x ~/ros2_ws/src/rebotarm_gui/scripts/setup_wsl.sh
```

### O simulador nao publica dados
Verifique se o ROS esta funcionando:
```bash
ros2 topic list
ros2 topic echo /rebotarm/joint_states
```

---

## Resumo do fluxo de trabalho

```
1. Abrir terminal WSL (Ubuntu)
2. cd ~/ros2_ws
3. code .    <- abre VS Code conectado ao WSL

No VS Code:
  Terminal 1: ros2 run rebotarm_gui mock_arm
  Terminal 2: ros2 run rebotarm_gui rebotarm_gui

A janela aparece no Windows via WSLg!
```

---

## Se nada funcionar

Copie a **mensagem de erro completa** e procure ajuda. Em Linux, a mensagem de erro e a melhor pista do que esta errado.