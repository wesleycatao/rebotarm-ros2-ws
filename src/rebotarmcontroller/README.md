# rebotarmcontroller

Nó controlador ROS 2 para o braço robótico **reBotArm**.

Implementa a ponte entre o SDK proprietário do braço (vendored em
[`third_party/`](rebotarmcontroller/third_party/)) e o ecossistema ROS 2:
publica estados das juntas e do braço, expõe serviços de segurança e
configuração, e oferece uma ação de movimento cartesiano.

> **Documentação relacionada:**
>
> - Interface gráfica: [`src/rebotarm_gui/README.md`](../rebotarm_gui/README.md)
> - Lançamento/bringup: [`src/rebotarm_bringup/README.md`](../rebotarm_bringup/README.md)
> - Interfaces de mensagens: [`src/rebotarm_msgs/README.md`](../rebotarm_msgs/README.md)

## Executáveis

| Comando | Descrição |
| --- | --- |
| `ros2 run rebotarmcontroller reBotArmController` | Nó controlador principal. |
| `ros2 run rebotarmcontroller DioBridge` | Ponte de E/S digitais. |
| `ros2 run rebotarmcontroller TorqueProfile` | Utilitário de perfil de torque. |
| `ros2 run rebotarmcontroller MoveTo` | Exemplo: mover juntas. |
| `ros2 run rebotarmcontroller MoveToPose` | Exemplo: mover para pose cartesiana. |
| `ros2 run rebotarmcontroller GripperControl` | Exemplo: controle da garra. |
| `ros2 run rebotarmcontroller GravityCompensation` | Exemplo: compensação de gravidade. |

## Principais módulos

- `rebotarm_controller.py` — nó principal e fiação ROS 2.
- `hardware_manager.py` — gerenciamento do hardware/SDK.
- `hardware_config.py` — carregamento de `rebotarm_hardware.yaml`.
- `ros_actions.py` — ação `MoveToPose`.
- `ros_services.py` / `ros_publishers.py` — serviços e tópicos.
- `torque_profile.py` / `speed_override.py` — perfis de torque e override de velocidade.
- `safety_event_logger.py` — registro de eventos de segurança.
- `authorization.py` — contexto de autorização/capacidades.

## Testes

```bash
colcon test --packages-select rebotarmcontroller
```

Cobre: watchdog de prazos, segurança de torque, override dinâmico de velocidade
e retemporização de trajetória.
