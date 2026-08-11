# rebotarm_moveit_config

Configuração do **MoveIt 2** para o braço robótico **reBotArm**.

Contém os arquivos de configuração do MoveIt (SRDF, cinemática, planejamento
OMPL, controladores ros2_control) e um serviço de verificação de clearance de
trajetória (`clearance_service_node`, em C++).

> **Documentação relacionada:**
>
> - Bringup/descrição URDF: [`src/rebotarm_bringup/README.md`](../rebotarm_bringup/README.md)
> - Demos de aplicação: [`src/rebotarm_moveit_demos/README.md`](../rebotarm_moveit_demos/README.md)

## Lançamentos

| Comando | Descrição |
| --- | --- |
| `ros2 launch rebotarm_moveit_config demo.launch.py` | MoveIt em modo demonstração (sem hardware real). |
| `ros2 launch rebotarm_moveit_config hardware.launch.py` | MoveIt acoplado ao hardware do reBotArm. |

## Conteúdo

- `config/` — SRDF (`rebotarm.srdf`, `rebotarm_rs.srdf`), cinemática, planejamento
  OMPL, controladores ros2_control e xacros de URDF/`ros2_control`.
- `src/clearance_service_node.cpp` — nó que expõe `CheckTrajectoryClearance`
  usando as malhas/cena de células. Compilado condicionalmente quando o MoveIt 2
  e suas dependências estão disponíveis (veja o `CMakeLists.txt`).
- `launch/` — launch files e RViz presets (`demo_view.rviz`, `moveit.rviz`).

## Build

Pacote `ament_cmake`. O `clearance_service_node` é compilado apenas quando
`moveit_core`, `moveit_ros_planning`, `rclcpp`, `rebotarm_msgs` e `sensor_msgs`
estão todos disponíveis; caso contrário, o pacote instala apenas os arquivos de
configuração.

```bash
colcon build --packages-select rebotarm_moveit_config
```
