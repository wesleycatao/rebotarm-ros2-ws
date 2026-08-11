# rebotarm_bringup

Lançamento, configuração e descrição (URDF + malhas) do braço robótico
**reBotArm**.

Centraliza os arquivos necessários para subir o robô em simulação ou hardware
real: parâmetros de driver, descrição URDF com as malhas (`.STL`), presets de
RViz e os launch files de bringup.

> **Documentação relacionada:**
>
> - Controlador: [`src/rebotarmcontroller/README.md`](../rebotarmcontroller/README.md)
> - Configuração MoveIt 2: [`src/rebotarm_moveit_config/README.md`](../rebotarm_moveit_config/README.md)
> - Instalação no Raspberry Pi 5: [`README_RASPBERRY_PI.md`](../../README_RASPBERRY_PI.md)

## Lançamentos

| Comando | Descrição |
| --- | --- |
| `ros2 launch rebotarm_bringup bringup.launch.py` | Bringup completo (driver + `robot_state_publisher` + RViz). |
| `ros2 launch rebotarm_bringup driver.launch.py` | Apenas o driver/controlador. |

## Conteúdo

- `launch/` — launch files.
- `config/driver_params.yaml` — parâmetros do nó driver.
- `config/rebotarm_hardware.yaml` — configuração de hardware (juntas, limites,
  caminho do URDF relativo ao SDK).
- `description/urdf/` — modelos URDF do braço.
- `description/meshes*/` — malhas (`.STL`) referenciadas pelo URDF.
- `rviz/rebotarm.rviz` — preset de visualização RViz.

> Os caminhos de URDF em `rebotarm_hardware.yaml` são relativos ao SDK do braço
> (consumidos por `reBotArm_control_py` em `rebotarmcontroller/third_party/`),
> não caminhos absolutos do workspace.

## Dependências de execução

`rebotarmcontroller`, `robot_state_publisher`, `rviz2`, `launch`, `launch_ros`.
