# rebotarm_msgs

Interfaces ROS 2 (mensagens, serviços e ações) customizadas para o braço robótico
**reBotArm**.

Este pacote define o contrato de comunicação entre o controlador
([`rebotarmcontroller`](../rebotarmcontroller/)), a visão
([`rebotarm_vision`](../rebotarm_vision/)) e a interface gráfica
([`rebotarm_gui`](../rebotarm_gui/)).

## Conteúdo

### Mensagens (`msg/`)

| Mensagem | Descrição |
| --- | --- |
| `ArmStatus` | Estado geral do braço (modo, falhas, prontidão). |
| `DigitalIO` / `DigitalIOState` | Comando e estado das E/S digitais. |
| `JointMitCmd` | Comando MIT bruto por junta (torque/velocidade). |
| `JointMotorState` | Detalhes do motor por junta. |
| `JointPosVelCmd` | Comando de posição + velocidade por junta. |
| `SegmentedObject` / `SegmentedObjectArray` | Objetos segmentados pela visão. |
| `TimingStatus` | Métricas de tempo/cumprimento de prazos da trajetória. |

### Serviços (`srv/`)

`CalibrateVision`, `CheckTorqueProfile`, `CheckTrajectoryClearance`,
`GripperCommand`, `MoveToPoseIK`, `SetDigitalIOConfig`, `SetDigitalOutput`,
`SetGripper`, `SetSafetyStep`, `SetTorqueProfile`, `SetZero`.

### Ações (`action/`)

`MoveToPose` — movimento cartesiano para uma pose, com feedback de progresso.

## Build

Pacote `ament_cmake` com `rosidl_default_generators`. É construído normalmente
via `colcon`:

```bash
colcon build --packages-select rebotarm_msgs
```

Após o build, as interfaces ficam disponíveis em Python/C++ como
`rebotarm_msgs/msg/...`, `rebotarm_msgs/srv/...` e `rebotarm_msgs/action/...`.
