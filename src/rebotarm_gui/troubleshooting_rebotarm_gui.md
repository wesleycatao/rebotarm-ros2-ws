# Troubleshooting — `rebotarm_gui`

Registro de problemas e resoluções do pacote `rebotarm_gui` (GUI PySide6/Qt + ROS 2).

---

## Caso 1: pacote não encontrado pelo `ros2 run`

### Status: resolvido

O problema foi corrigido em `/home/wesley/ros2_ws/src/rebotarm_gui/package.xml`.

Causa raiz:

- O `package.xml` do pacote fonte estava salvo com XML escapado como texto (`&lt;package&gt;` em vez de `<package>`).
- O manifesto também não declarava `<buildtool_depend>ament_python</buildtool_depend>`.
- Por isso o build chegava a instalar scripts e alguns arquivos, mas não gerava o hook `ament_prefix_path` em `install/rebotarm_gui/share/rebotarm_gui/package.dsv`.
- Sem esse hook, `source install/setup.bash` não adicionava `/home/wesley/ros2_ws/install/rebotarm_gui` ao `AMENT_PREFIX_PATH`, então `ros2 pkg` e `ros2 run` não encontravam o pacote.

Correção aplicada:

```xml
<buildtool_depend>ament_python</buildtool_depend>
```

O `package.xml` também foi convertido para XML válido.

Rebuild executado:

```bash
rm -rf build/rebotarm_gui install/rebotarm_gui
source /opt/ros/lyrical/setup.bash
source install/setup.bash
colcon build --packages-select rebotarm_gui --event-handlers console_direct+
source install/setup.bash
```

Validações:

```bash
ros2 pkg list | rg '^rebotarm_gui$'
ros2 pkg prefix rebotarm_gui
ros2 pkg executables rebotarm_gui
ros2 run --prefix true rebotarm_gui rebotarm_gui
python3 -c 'from ament_index_python.packages import get_package_prefix; print(get_package_prefix("rebotarm_gui"))'
```

Resultados confirmados:

- `ros2 pkg list` retorna `rebotarm_gui`
- `ros2 pkg prefix rebotarm_gui` retorna `/home/wesley/ros2_ws/install/rebotarm_gui`
- `ros2 pkg executables rebotarm_gui` mostra `mock_arm` e `rebotarm_gui`
- `AMENT_PREFIX_PATH` inclui `/home/wesley/ros2_ws/install/rebotarm_gui`
- `package.dsv` agora contém o hook `ament_prefix_path`

### Problema

Após `source install/setup.bash`, o comando `ros2 run rebotarm_gui rebotarm_gui` retorna:

```
Package 'rebotarm_gui' not found
```

O pacote `rebotarm_gui` **não aparece** no `AMENT_PREFIX_PATH`:

```
/home/wesley/ros2_ws/install/rebotarm_moveit_demos
/home/wesley/ros2_ws/install/rebotarm_moveit_config
/home/wesley/ros2_ws/install/rebotarm_bringup
/home/wesley/ros2_ws/install/rebotarmcontroller
/home/wesley/ros2_ws/install/rebotarm_msgs
/opt/ros/lyrical
```

Também não aparece em `ros2 pkg list | grep rebotarm_gui` nem no `ament index list`.

### Sintomas identificados

1. **Pacote existe em `install/`** com arquivos gerados:
   - `install/rebotarm_gui/share/rebotarm_gui/package.xml` ✅
   - `install/rebotarm_gui/share/rebotarm_gui/hook/pythonpath.dsv` ✅
   - `install/rebotarm_gui/lib/rebotarm_gui/rebotarm_gui` (entry point) ✅
   - `install/rebotarm_gui/share/rebotarm_gui/package.sh` ✅

2. **`local_setup.bash` está faltando** dentro de `install/rebotarm_gui/share/rebotarm_gui/` ❌
   - Outros pacotes (ex: `rebotarm_msgs`) possuem este arquivo
   - Sem ele, o ambiente não carrega as configurações do pacote

3. **Build foi bem-sucedido:**
   - `colcon build --packages-select rebotarm_gui --symlink-install` → `Finished <<< rebotarm_gui [1.70s]`
   - `build/rebotarm_gui/colcon_build.rc` → `0` (success)
   - Pasta `build/rebotarm_gui/` existe com todos os artefatos

4. **`package.sh` referenceia hook de desenvolvimento inexistente:**
   ```bash
   _colcon_package_sh_source_script "$COLCON_CURRENT_PREFIX/../../build/rebotarm_gui/share/rebotarm_gui/hook/pythonpath_develop.sh"
   ```
   Este arquivo `pythonpath_develop.sh` não existe.

### Setup.py (configuração do pacote)

```python
from setuptools import find_packages, setup

package_name = "rebotarm_gui"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/styles", ["rebotarm_gui/styles/dark.qss"]),
        (f"share/{package_name}/launch", ["launch/gui.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="reBotArm Maintainers",
    maintainer_email="support@example.com",
    description="Graphical user interface for the reBotArm B601 controller (ROS 2).",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "rebotarm_gui = rebotarm_gui.main:main",
            "mock_arm = rebotarm_gui.mock_arm:main",
        ],
    },
)
```

### Package.xml (dependências)

```xml
<package format="3">
  <name>rebotarm_gui</name>
  <version>0.1.0</version>
  <description>Graphical user interface for the reBotArm B601 controller (ROS 2).</description>
  <maintainer email="support@example.com">reBotArm Maintainers</maintainer>
  <license>Apache-2.0</license>
  <depend>rclpy</depend>
  <depend>rebotarm_msgs</depend>
  <depend>std_srvs</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>control_msgs</depend>
  <depend>trajectory_msgs</depend>
  <depend>builtin_interfaces</depend>
  <exec_depend>python3-pyside6</exec_depend>
  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>
  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### O que foi tentado

#### 1. Verificação do ambiente
- `source ~/ros2_ws/install/setup.bash` seguido de `ros2 run rebotarm_gui rebotarm_gui` → **falhou**
- `echo $AMENT_PREFIX_PATH | tr ':' '\n'` → `rebotarm_gui` não aparece na lista
- `ros2 pkg list | grep rebotarm_gui` → **sem resultados**
- `ament index list 2>&1 | grep rebotarm_gui` → **não encontrado no índice**

#### 2. Verificação da instalação
- `ls ~/ros2_ws/install/rebotarm_gui/share/rebotarm_gui/` → diretório existe com `hook/`, `launch/`, `styles/`, `package.bash`, `package.sh`, `package.xml`, `package.dsv`, `package.ps1`
- `ls ~/ros2_ws/install/rebotarm_gui/lib/rebotarm_gui/` → contém `mock_arm` e `rebotarm_gui` (entry points compilados)
- `cat ~/ros2_ws/install/rebotarm_gui/share/rebotarm_gui/local_setup.bash` → **arquivo não existe** ❌

#### 3. Rebuild do pacote
- `colcon build --packages-select rebotarm_gui --symlink-install` → **buildou com sucesso** (código 0)
- Mas o problema persistiu após novo `source install/setup.bash`

#### 4. Verificação do build
- `ls ~/ros2_ws/build/rebotarm_gui/` → diretório existe com artefatos de build
- `cat ~/ros2_ws/build/rebotarm_gui/colcon_build.rc` → `0` (sucesso)

#### 5. Arquivos de configuração
- `setup.py` → configurado corretamente com `console_scripts`
- `package.xml` → configurado corretamente com `build_type = ament_python`
- `setup.cfg` → **não existe** (mas o `colcon build` funciona sem ele para pacotes `ament_python`)

### Hipóteses

1. **Problema no `local_setup.bash` não gerado** — Pode ser um bug na versão do `colcon` ou `ament_package` do ROS 2 Lyrical, ou o arquivo foi gerado mas não copiado para `install/`.

2. **`COLCON_IGNORE` em algum lugar** — Verificado, não existe (`cat ~/ros2_ws/build/rebotarm_gui/COLCON_IGNORE` → exit code 1, arquivo não existe).

3. **Pacote não registrado no índice de recursos** — O arquivo `resource/rebotarm_gui` (marcador do ament index) pode não ter sido gerado corretamente em `install/`.

4. **Ordem de carregamento dos pacotes** — Pode ser que o `setup.bash` esteja ignorando `rebotarm_gui` por algum problema na geração do arquivo de ordem.

### Próximos passos sugeridos

- Verificar se o arquivo `install/rebotarm_gui/share/ament_index/resource_index/packages/rebotarm_gui` existe
- Verificar o conteúdo de `install/_order_isolated_packages.bash` (se existir)
- Tentar `colcon build --packages-select rebotarm_gui` (sem `--symlink-install`)
- Verificar a versão do colcon: `colcon --version`
- Tentar rodar diretamente: `~/ros2_ws/install/rebotarm_gui/lib/rebotarm_gui/rebotarm_gui`
- Verificar os logs do colcon build com `colcon build --packages-select rebotarm_gui --event-handlers console_direct+`

---

## Caso 2: janela da GUI não aparece no WSLg (após reboot do Windows)

### Status: resolvido

**Ambiente:** WSL 2 (Ubuntu) com WSLg no Windows 11, VSCode. Versões no momento do problema/solução: WSL 2.9.3.0, WSLg 1.0.79, kernel 6.18.x.

#### Sintoma

- O ícone da GUI aparece na **barra de tarefas do Windows**, mas a janela **não é exibida** ao clicar.
- O título da janela vem com um prefixo estranho: `[WARN: COPY MODE] reBot...`.
- O processo da GUI roda sem erro e registra no compositor, mas nada aparece no desktop.
- Ocorreu **após reiniciar o computador** (provável atualização do Windows no reboot).
- Afeta **qualquer** app gráfico Linux, não só a reBotArm (confirmado com uma janela Qt mínima).

#### Causa raiz

O WSLg entrega os frames das janelas ao Windows por um mount especial de memória compartilhada em `/mnt/shared_memory`. Após o reboot, o host do WSL **parou de injetar esse mount** no distro. Resultado: o Weston (compositor) registra as janelas e os ícones chegam via metadados RDP, mas o **conteúdo renderizado nunca chega ao desktop**.

#### Diagnóstico

Verificar o mount de shared memory:

```bash
ls /mnt/shared_memory && echo "WSLg OK"   # saudável: diretório existe
```

Confirmar no log do Weston:

```bash
grep -niE "shared_memory|allocate_shared|Input/output" /mnt/wslg/weston.log
```

Assinatura do problema:

```text
rdp_allocate_shared_memory: Failed to open "/mnt/shared_memory/{guid}" with error: Input/output error
```

> O `[COPY MODE]` no título é só um aviso de performance (caminho Xwayland em software), **não** a causa da janela não aparecer.

#### Solução aplicada (corrige o shared memory)

No **PowerShell do Windows** (não dentro do WSL), como Administrador:

```powershell
wsl --update            # se disser "já atualizado", use: wsl --update --pre-release
wsl --shutdown
```

Depois **fechar e reabrir o VSCode** (a VM reinicia zerada e recria `/mnt/shared_memory`). Confirmar:

```bash
ls /mnt/shared_memory && echo "WSLg OK"
```

> `wsl --shutdown` encerra a VM inteira — todos os processos (mock, GUI) caem. É esperado.

#### Se `wsl --update` + `wsl --shutdown` não bastar

- **Integridade de Memória (HVCI)**: `Segurança do Windows` → `Segurança do dispositivo` → `Detalhes do isolamento do núcleo` → **Integridade da memória = DESLIGADO** → reiniciar o PC. Ativada por update, quebra a virtualização do WSL.
- **Reset rápido do WSL** (PowerShell admin): `Restart-Service LxssManager`, depois reabrir o VSCode.
- **Bypass total do WSLg**: instalar o **VcXsrv** no Windows e usar `export DISPLAY=$(ip route show default | awk '{print $3}'):0.0` + `export LIBGL_ALWAYS_SOFTWARE=1`.

### Recomendado (permanente): rodar a GUI em Wayland nativo

Mesmo com o WSLg saudável, forçar o Qt a usar **Wayland nativo** evita o caminho Xwayland/copy-mode lento (e o `[COPY MODE]` no título). **Já deixado fixo** em `~/.bashrc`:

```bash
# Forca Qt a usar Wayland nativo no WSLg (evita o caminho Xwayland/copy-mode lento)
export QT_QPA_PLATFORM=wayland
```

Assim, todo novo terminal já vem com `QT_QPA_PLATFORM=wayland` ativo — não é preciso digitar ao rodar a GUI:

```bash
ros2 run rebotarm_gui mock_arm        # terminal 1 (simulador)
ros2 run rebotarm_gui rebotarm_gui    # terminal 2 (GUI)
```

Pré-requisito do plugin Wayland do Qt6 (já satisfeito neste ambiente):

```bash
ls /usr/lib/x86_64-linux-gnu/qt6/plugins/platforms/libqwayland.so   # pacote: qt6-wayland
```

### Cheat sheet de diagnóstico

| Verificação | Comando | Saudável |
|---|---|---|
| Mount de shared memory | `ls /mnt/shared_memory` | diretório existe |
| Erro no log do Weston | `grep shared_memory /mnt/wslg/weston.log` | sem `Input/output error` |
| Uptime da VM | `uptime -s` | mudou após `wsl --shutdown` |
| Plataforma Qt em uso | `python3 -c "from PySide6.QtWidgets import QApplication as A; print(A([]).platformName())"` | `wayland` |
| Versões do WSL (Windows) | `wsl.exe --version` (via interop) | WSL 2.9.x, WSLg 1.0.7x |

---

## Caso 3: aba Segurança / perfil de torque

### Sintoma: serviços `/rebotarm/safety/*` não aparecem

Verifique:

```bash
source install/setup.bash
ros2 service list | grep /rebotarm/safety
```

Se não aparecerem, recompile as mensagens, o controlador e a GUI:

```bash
colcon build --packages-select rebotarm_msgs rebotarmcontroller rebotarm_gui rebotarm_bringup --event-handlers console_direct+
source install/setup.bash
```

As interfaces novas são:

- `/rebotarm/safety/set_torque_profile`
- `/rebotarm/safety/set_step`
- `/rebotarm/safety/check_torque_profile`
- `/rebotarm/safety/reset_fault`

### Sintoma: importar YAML falha

O perfil atual deve usar `version: 2` ou um perfil `version: 1` antigo
compatível. O formato v2 tem `steps`, e cada etapa contém limites `warn_min`,
`warn_max`, `fail_min` e `fail_max` por junta.

Gere novamente pela aba **Segurança** ou por CLI:

```bash
ros2 run rebotarmcontroller TorqueProfile build \
  torque_runs/run1.csv torque_runs/run2.csv \
  --warn-tolerance-percent 10 \
  --fail-tolerance-percent 20 \
  --output torque_profiles/profile.yaml
```

### Sintoma: WARN/FAIL inesperado

Cheque:

- O `Step ID` ativo na aba **Segurança** é o mesmo gravado no CSV/YAML.
- A etapa no **Planejamento** tem rótulo estável; o rótulo vira `step_id`.
- A carga/payload gravada é compatível com a carga real.
- A margem mínima (`min_margin`) não está baixa demais.
- Comece em `warn_only`; use `stop_on_fail` só depois de validar.

### Sintoma: braço entrou em `PROTECTIVE_STOP`

1. Afaste pessoas/obstáculos e verifique a causa.
2. Corrija a condição de risco.
3. Na aba **Segurança**, clique **Resetar falha**.
4. Se necessário, volte o modo para `warn_only` antes de repetir a tarefa.

---

## Caso 4: comandos bloqueados, heartbeat perdido ou action em timeout

### Comportamento esperado

A GUI só habilita comandos de movimento quando recebe `arm_status` recente,
o braço está habilitado, não está em estado de falha e o action server usado
foi descoberto. Se o heartbeat ficar mais de 1 s sem atualização:

- o cabeçalho muda para **BRAÇO SEM SINAL**;
- Home, Desabilitar, jog, pose, gripper, freedrive e MIT/Raw são bloqueados;
- uma execução real ativa recebe solicitação de cancelamento;
- o último estado não continua sendo apresentado como atual.

Diagnóstico sem terminal:

1. Selecione o modo **Supervisor** ou **Engenharia**.
2. Abra **Ferramentas e ajustes → Diagnóstico ROS**.
3. Confira o card **HEARTBEAT**, a linha `/rebotarm/arm_status` e os três
   action servers na aba **Serviços e actions**.
4. Revise **Último erro desta sessão**. Falhas de serviços/actions e erros da
   bridge são registrados automaticamente.

A frequência é calculada em uma janela móvel de 5 s. **SEM ATUALIZAÇÃO** em um
tópico orientado a eventos não é, sozinho, evidência de falha. Latência `N/D`
significa que a mensagem não possui timestamp utilizável ou usa outro relógio.

Se for necessário confirmar fora da GUI:

```bash
ros2 topic hz /rebotarm/arm_status
ros2 topic echo /rebotarm/arm_status --once
ros2 action info /rebotarm/move_to_pose
ros2 action info /rebotarm/follow_joint_trajectory
ros2 action info /rebotarm/gripper/command
ros2 service type /rebotarm/disable
```

Goals possuem timeout de aceitação de 3 s. Após aceitos, movimentos possuem a
duração solicitada mais 5 s de margem; o gripper possui timeout de 10 s. Um
timeout aparece no banner persistente e em **Eventos ROS**, e a GUI solicita o
cancelamento do goal.

### Reproduzir falhas sem hardware

O mock possui injeção determinística para validar este comportamento. Execute
apenas uma falha por vez e abra a GUI normalmente em outro terminal:

```bash
# heartbeat some após 3 s, mas joint_states continua sendo publicado
ros2 run rebotarm_gui mock_arm --heartbeat-stop-after 3

# servidor aceita o movimento, mas nunca devolve resultado
ros2 run rebotarm_gui mock_arm --hang-action move_to_pose

# servidor rejeita o goal
ros2 run rebotarm_gui mock_arm --reject-action move_to_pose

# JointState passa a ter vetor truncado e NaN após 3 s
ros2 run rebotarm_gui mock_arm --invalid-telemetry-after 3

# aceitação só chega 2 s depois; use Parar durante esse intervalo
ros2 run rebotarm_gui mock_arm \
  --late-accept-action move_to_pose --accept-delay 2
```

Para executar todos os seis casos, inclusive matar o nó durante um movimento:

```bash
QT_QPA_PLATFORM=offscreen python3 -m pytest \
  src/rebotarm_gui/test/test_fault_integration.py -v
```

Resultado esperado: `6 passed`. Cada teste usa um namespace novo, espera o
heartbeat saudável antes de injetar a falha e encerra o processo ao final.
Uma telemetria inválida deve aparecer em **Último erro desta sessão**, sem
substituir os valores válidos que já estavam no cache da GUI.

### Parada de software

O botão **PARADA SOFTWARE** solicita cancelamento de todas as actions rastreadas
e chama `/rebotarm/disable`. A operação é assíncrona: confira em **Eventos ROS**
se o servidor aceitou o cancelamento e se `disable` concluiu. Não existe atalho
global por `Esc`; a tecla permanece reservada para cancelar diálogos e edições.
A função não substitui o E-stop físico da célula.

### Execução bloqueada por falha na auditoria

A página **Programa** bloqueia o movimento se não conseguir registrar o início
da execução ou da próxima etapa. A mensagem informa o destino esperado, por
padrão:

```text
~/.local/state/rebotarm_gui/audit/execution.jsonl
```

Verifique espaço livre, propriedade e permissões da pasta. Com
`XDG_STATE_HOME`, o arquivo fica sob essa raiz. Não remova ou edite o JSONL com
a GUI aberta. Depois de corrigir o armazenamento, tente novamente; não é
necessário reiniciar o controlador ROS. O formato completo e a exportação CSV
estão descritos em [AUDIT_LOG.md](AUDIT_LOG.md).

## Histórico

- **Caso 1** (`package not found`): corrigido ajustando `package.xml` (XML válido + `<buildtool_depend>ament_python</buildtool_depend>`) e rebuildando.
- **Caso 2** (GUI não aparece no WSLg, 2026-07-08): mount `/mnt/shared_memory` do WSLg não injetado pelo host após reboot (`Input/output error` no `weston.log`). Resolvido com `wsl --update` (→ 2.9.3.0) + `wsl --shutdown` + reinício. `QT_QPA_PLATFORM=wayland` fixado em `~/.bashrc` para renderização ideal no WSLg.
- **Caso 3** (Segurança/torque): nova aba usa services geradas em `rebotarm_msgs`; se não aparecem, rebuildar `rebotarm_msgs`, `rebotarmcontroller`, `rebotarm_gui` e `rebotarm_bringup`.
