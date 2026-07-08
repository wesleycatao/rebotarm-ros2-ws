# Troubleshooting: `rebotarm_gui` package not found by `ros2 run`

## Status: resolvido

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

## Problema

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

## Sintomas identificados

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

## Setup.py (configuração do pacote)

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

## Package.xml (dependências)

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

## O que foi tentado

### 1. Verificação do ambiente
- `source ~/ros2_ws/install/setup.bash` seguido de `ros2 run rebotarm_gui rebotarm_gui` → **falhou**
- `echo $AMENT_PREFIX_PATH | tr ':' '\n'` → `rebotarm_gui` não aparece na lista
- `ros2 pkg list | grep rebotarm_gui` → **sem resultados**
- `ament index list 2>&1 | grep rebotarm_gui` → **não encontrado no índice**

### 2. Verificação da instalação
- `ls ~/ros2_ws/install/rebotarm_gui/share/rebotarm_gui/` → diretório existe com `hook/`, `launch/`, `styles/`, `package.bash`, `package.sh`, `package.xml`, `package.dsv`, `package.ps1`
- `ls ~/ros2_ws/install/rebotarm_gui/lib/rebotarm_gui/` → contém `mock_arm` e `rebotarm_gui` (entry points compilados)
- `cat ~/ros2_ws/install/rebotarm_gui/share/rebotarm_gui/local_setup.bash` → **arquivo não existe** ❌

### 3. Rebuild do pacote
- `colcon build --packages-select rebotarm_gui --symlink-install` → **buildou com sucesso** (código 0)
- Mas o problema persistiu após novo `source install/setup.bash`

### 4. Verificação do build
- `ls ~/ros2_ws/build/rebotarm_gui/` → diretório existe com artefatos de build
- `cat ~/ros2_ws/build/rebotarm_gui/colcon_build.rc` → `0` (sucesso)

### 5. Arquivos de configuração
- `setup.py` → configurado corretamente com `console_scripts`
- `package.xml` → configurado corretamente com `build_type = ament_python`
- `setup.cfg` → **não existe** (mas o `colcon build` funciona sem ele para pacotes `ament_python`)

## Hipóteses

1. **Problema no `local_setup.bash` não gerado** — Pode ser um bug na versão do `colcon` ou `ament_package` do ROS 2 Lyrical, ou o arquivo foi gerado mas não copiado para `install/`.

2. **`COLCON_IGNORE` em algum lugar** — Verificado, não existe (`cat ~/ros2_ws/build/rebotarm_gui/COLCON_IGNORE` → exit code 1, arquivo não existe).

3. **Pacote não registrado no índice de recursos** — O arquivo `resource/rebotarm_gui` (marcador do ament index) pode não ter sido gerado corretamente em `install/`.

4. **Ordem de carregamento dos pacotes** — Pode ser que o `setup.bash` esteja ignorando `rebotarm_gui` por algum problema na geração do arquivo de ordem.

## Próximos passos sugeridos

- Verificar se o arquivo `install/rebotarm_gui/share/ament_index/resource_index/packages/rebotarm_gui` existe
- Verificar o conteúdo de `install/_order_isolated_packages.bash` (se existir)
- Tentar `colcon build --packages-select rebotarm_gui` (sem `--symlink-install`)
- Verificar a versão do colcon: `colcon --version`
- Tentar rodar diretamente: `~/ros2_ws/install/rebotarm_gui/lib/rebotarm_gui/rebotarm_gui`
- Verificar os logs do colcon build com `colcon build --packages-select rebotarm_gui --event-handlers console_direct+`
