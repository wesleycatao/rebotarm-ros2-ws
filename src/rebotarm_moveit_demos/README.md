# rebotarm_moveit_demos

Demos de aplicação usando o **MoveIt 2** com o braço robótico **reBotArm**.

Exemplos práticos que coordenam planejamento de movimento, controle da garra e
(tipicamente) a visão para tarefas de manipulação.

> **Documentação relacionada:**
>
> - Configuração MoveIt 2: [`src/rebotarm_moveit_config/README.md`](../rebotarm_moveit_config/README.md)
> - Visão: [`src/rebotarm_vision/README.md`](../rebotarm_vision/README.md)

## Executáveis

| Comando | Descrição |
| --- | --- |
| `ros2 run rebotarm_moveit_demos pick_place` | Demo de pick-and-place. |
| `ros2 run rebotarm_moveit_demos draw_square` | Demo de trajetória (desenho de quadrado). |

## Build

Pacote `ament_python`.

```bash
colcon build --packages-select rebotarm_moveit_demos
```

Os executáveis exigem que o `rebotarm_moveit_config` esteja construído e o
MoveIt 2 disponível no ambiente.
