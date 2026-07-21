# reBotArm GUI

Interface gráfica (PySide6/Qt) para o controlador ROS 2 do braço robótico **reBotArm**.

A GUI consome as interfaces expostas no namespace `/rebotarm` (tópicos, serviços e actions) e oferece um painel único para comandar o braço, o gripper e monitorar telemetria.

> **Documentação relacionada:**
> - Instalação e configuração no Raspberry Pi 5: [`README_RASPBERRY_PI.md`](../../README_RASPBERRY_PI.md)
> - Detalhes do pipeline de visão: [`src/rebotarm_vision/README.md`](../rebotarm_vision/README.md)
> - Revisão de UX, comparação de mercado e atalhos: [`GUI_UX_REDESIGN.md`](GUI_UX_REDESIGN.md)

## Requisitos

- ROS 2 (Iron/Jazzy/Rolling)
- Pacotes do workspace: `rebotarm_msgs`, `rebotarm_bringup`, `rebotarmcontroller`, `rebotarm_vision`
- **PySide6** (biblioteca Qt para Python)

Instale o PySide6 no ambiente ROS 2:

```bash
pip install PySide6
```

## Compilação

No workspace ROS 2 que contém a pasta `src/rebotarm_gui`:

```bash
colcon build --packages-select rebotarm_msgs rebotarmcontroller rebotarm_vision rebotarm_gui
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

## Mock do braço (testes sem hardware)

O pacote inclui um simulador ROS 2 que publica todos os tópicos de estado e
responde aos serviços/ações que a GUI consome (`/rebotarm/move_to_pose`,
`/rebotarm/gripper/command`, `/rebotarm/safety/set_step`, I/O digital, etc.).

```bash
# Terminal 1
ros2 run rebotarm_gui mock_arm

# Terminal 2
ros2 run rebotarm_gui rebotarm_gui
```

Com o mock ativo é possível editar e executar programas na página **Programa**
(inclusive com dry-run, passo a passo, undo/redo) sem o braço real.

### Teste de integração GUI + mock

Há um teste automatizado que sobe o mock em subprocesso, instancia a aba
**Programa** e executa uma sequência de comandos em modo simulação e em
modo real contra o mock:

```bash
cd ~/ros2_ws
source install/setup.bash
QT_QPA_PLATFORM=offscreen python3 -m pytest src/rebotarm_gui/test/test_mock_integration.py -v
```

O teste verifica:
- carregamento do programa;
- execução até o fim sem erros;
- barra de progresso chegando a 100 %;
- status final `Programa concluído.`.

Testes unitários da aba **Visão** (calibração por serviço):

```bash
cd ~/ros2_ws
source install/setup.bash
QT_QPA_PLATFORM=offscreen python3 -m pytest src/rebotarm_gui/test/test_vision_tab_calibration.py -v
```

Cobertura:
- conversão `QImage → sensor_msgs/Image`;
- habilitação/desabilitação dos botões **Aprovar/Repetir/Complementar**;
- aplicação de perfil convergido e não convergido.

## Visualizando o mock

### 1. RViz (visualização 3D do URDF)

Com o `mock_arm` rodando, inicie o `robot_state_publisher` + RViz:

```bash
ros2 launch rebotarm_gui mock_visualize.launch.py
```

Isso publica o URDF em `/rebotarm/robot_description` e abre o RViz com o modelo
`reBot_B601_DM_with_gripper` acompanhando os `joint_states` do mock.

> O gripper no mock é publicado como uma única junta `gripper`, enquanto o
> URDF possui `gripper_joint1` e `gripper_joint2`. Por isso os dedos do gripper
> não se movem no RViz, mas os 6 elos principais do braço se movem normalmente.

### 2. Plot em tempo real das juntas

Há um utilitário que plota as posições das juntas a partir de `/rebotarm/joint_states`:

```bash
# Terminal 1
ros2 run rebotarm_gui mock_arm

# Terminal 2
ros2 run rebotarm_gui plot_mock_joints
```

Use `--topic` para outro tópico de `JointState` e `--window` para ajustar a
janela de tempo do gráfico.

### 3. Linha de comando / rqt

Acompanhe os valores publicados:

```bash
ros2 topic echo /rebotarm/joint_states
ros2 topic hz /rebotarm/joint_states
```

Ou use o `rqt_plot`:

```bash
ros2 run rqt_plot rqt_plot /rebotarm/joint_states/position[0] /rebotarm/joint_states/position[1]
```

## Navegação

A janela requer no mínimo **1360 × 720 px**. Abaixo de 1700 px o cabeçalho
reorganiza estado e comandos em duas linhas; a página Início usa rolagem para
preservar todo o conteúdo sem sobreposição.

O fluxo diário fica na barra lateral. **Sistema**, **I/O Digital**, **Telemetria**,
**Diagnóstico ROS**, **MIT / Raw** e **Configurações** ficam no menu
**Ferramentas e ajustes**, pois são páginas de engenharia, diagnóstico ou configuração.

| Página       | Função                                                                 |
|--------------|------------------------------------------------------------------------|
| **Início**   | Fluxo principal, atalhos de tarefa e dashboard da célula (braço, modo, programa, objetos detectados) |
| **Sistema**  | Status (enable/mode/model/rate/channel), Enable/Disable/Safe Home, Gravity ON/OFF, Set Zero |
| **Juntas**   | Sliders por junta com limites reais do URDF, jog press-and-hold (passo 1°/5°/10°) + FollowJointTrajectory |
| **Pose**     | XYZ + RPY/Quaternion → MoveToPose (action) / MoveToPoseIK, pose do TCP ao vivo (FK), progresso do movimento e jog cartesiano X/Y/Z (5/10/50 mm) |
| **Robô 3D**  | Visualização em tempo real da postura do braço (esqueleto por cinemática direta do URDF, órbita e zoom) |
| **Programa** | Editor de programas de movimento (ir a ponto, delay, linha reta, arco circular, paletização 3D) com execução sequencial, salvar/carregar JSON, teach (Freedrive + ensinar ponto) e visão em fluxograma |
| **Gripper**  | Estado do gripper + Abrir/Fechar/Definir posição (service)/Comandar (action), posição em mm |
| **Visão**    | Processamento clássico de imagem, teste com imagem, limite operacional, calibração pixel -> XY e auto-tune IA |
| **Pegas**    | Estratégias de coleta a partir dos objetos detectados, clearances, limite operacional e geração de programa |
| **Segurança** | Perfis supervisionados de torque por etapa, tolerâncias WARN/FAIL, alarme de colisão, reset de falha e geração/importação de YAML |
| **I/O Digital** | GPIO do Raspberry Pi: 5 entradas opto (IN1–IN5) + 5 saídas relé (OUT1–OUT5), com ID/TAG/habilita/contador/status e comandos ON/OFF/TOGGLE |
| **Configurações** | Configurações persistentes da GUI, incluindo dados da API OpenAI para auto-tune; acessível em **Ferramentas e ajustes** |
| **Telemetria**| Tabelas live de `joint_states` e detalhe por motor (`/joints/<j>/state`) |
| **Diagnóstico ROS** | Heartbeat, frequência/idade/latência dos tópicos consumidos pela GUI, descoberta de serviços/actions e último erro da sessão |
| **MIT / Raw**| Publicação bruta de MIT (`kp/kd/tau`) ou pos-vel (`pos/vlim`) por junta |

Na página **Diagnóstico ROS**, a frequência usa uma janela móvel de 5 s e a
idade representa o tempo desde a última mensagem recebida. A latência é
calculada a partir de `header.stamp`; aparece como `N/D` para mensagens sem
header, timestamp zero ou relógio incompatível. Tópicos orientados a eventos
podem aparecer como **SEM ATUALIZAÇÃO** sem que isso represente necessariamente
uma falha. A disponibilidade dos serviços e actions usa a descoberta local do
ROS 2 e nunca realiza uma chamada de comando.

### Atalhos

| Atalho | Ação |
|---|---|
| `Esc` | Cancela/fecha o diálogo ou a edição atual; não aciona a parada de software |
| `Ctrl+1`…`Ctrl+4` | Início, Visão, Pegas, Programa |
| `Ctrl+5`…`Ctrl+7` | Pose cartesiana, Juntas, Gripper |
| `Ctrl+8` / `Ctrl+9` | Robô 3D / Segurança |
| `Ctrl+,` | Configurações |
| `Ctrl+L` | Mostrar/ocultar Eventos ROS |
| `Ctrl+Shift+A` | Abrir o centro de Alarmes |
| `F6` / `Shift+F6` | Próxima / anterior região da interface |
| `F1` | Ajuda completa de teclado |
| `Ctrl+Z` / `Ctrl+Y` | Desfazer / refazer no Programa |

### Acessibilidade

A interface pode ser percorrida integralmente por teclado. Os atalhos de página
levam o foco ao conteúdo, `F6` alterna entre as regiões principais e todos os
controles operáveis têm nome textual. Campos de formulário estão associados aos
rótulos; alarmes e perda de heartbeat enviam anúncios pela API de acessibilidade
do Qt. Estados usam texto e, nos indicadores compactos, forma além de cor.

O tema oferece foco de alto contraste em todos os tipos de controle. Os pares de
texto ativos auditados atendem 4,5:1 e o foco atende pelo menos 3:1 contra o
fundo. Consulte [ACCESSIBILITY_USABILITY.md](ACCESSIBILITY_USABILITY.md) para a
matriz de leitor de tela, critérios e protocolo humano.

Para abrir a GUI com o painel de tarefas cronometradas:

```bash
ros2 run rebotarm_gui rebotarm_gui --usability-test
```

O painel exporta um JSON anonimizado. A execução humana continua dependente de
participantes reais; testes automatizados não são contabilizados como resultado
de operador.

## Segurança operacional

- **Parada de software**: o botão vermelho fixo solicita o cancelamento de todas
  as actions rastreadas pela GUI (`MoveToPose`, `FollowJointTrajectory` e
  gripper) e chama `disable`. A aceitação do cancelamento e o resultado de
  `disable` são assíncronos e aparecem nos eventos. O botão não possui atalho
  global e **não substitui** E-stop físico, PLC de segurança ou avaliação de risco.
- **Confirmações centralizadas**: Home seguro e Desabilitar pedem confirmação
  em todos os pontos de entrada. Zerar uma ou todas as juntas também exige
  confirmação explícita.
- **LED de conexão por heartbeat**: o indicador da barra de status reflete o
  heartbeat de `/{ns}/arm_status` (timeout de 1 s). Verde = controlador
  publicando; âmbar = sem sinal do controlador (mesmo com o ROS 2 ativo). Ao
  perder o heartbeat, estados antigos são invalidados, comandos de movimento
  são bloqueados e uma execução real ativa recebe solicitação de cancelamento.

### Modos da interface

A barra lateral permite alternar o perfil ativo (persistido em
`~/.config/rebotarm_gui/settings.json`):

| Modo | Conteúdo exibido |
|--------|--------|
| **Operador** | Fluxo diário (Início, Visão, Pegas, Programa, movimento manual, Robô 3D, Segurança). Menu Ferramentas oculto e páginas de engenharia bloqueadas. |
| **Supervisor** (padrão) | Tudo do Operador + Sistema, I/O, Telemetria, Diagnóstico ROS e Configurações. MIT/Raw bloqueado. |
| **Engenharia** | Acesso total, incluindo MIT / Raw. |

A navegação para páginas ocultas é recusada, não apenas removida da lateral.
Entretanto, o seletor é uma adaptação de interface local: **não autentica o
usuário e não é uma fronteira de segurança**. Restrições reais de autorização
devem ser validadas pelo controlador ROS ou por um sistema externo de identidade.

## Arquitetura

```
┌──────────────────────────── Qt GUI (main thread) ─────────────────────────────┐
│  MainWindow                                                                  │
│   ├─ SystemTab / JointsTab / PoseTab / MotionTab / GripperTab / SafetyTab    │
│   ├─ IoTab / VisionTab / GraspTab / SettingsTab / TelemetryTab / MitTab      │
│   ├─ DiagnosticsTab (tópicos, endpoints, heartbeat e último erro)            │
│   ├─ Robot3dTab (visualização 3D por FK)                                     │
│   └─ widgets (StatusLed, JointSlider, PoseInput, ProgramFlowWidget)          │
│                                                                              │
│            ▲ Qt signals (thread-safe)                                        │
│            │                                                                 │
│   ┌────────┴─────────── RosBridge (QObject) ─────────┐                       │
│   │  QThread + rclpy.spin (MultiThreadedExecutor)   │                       │
│   │   subs / clients / action clients / publishers  │                       │
│   │   heartbeat · métricas ROS · feedback MoveToPose│                       │
│   └──────────────────────────────────────────────────┘                       │
│   kinematics.py: parsing de URDF, cinemática direta e limites das juntas     │
└──────────────────────────────────────────────────────────────────────────────┘
```

- O `RosBridge` roda um `rclpy` node em uma `QThread` separada e emite **Qt signals** para atualizar a UI com segurança entre threads.
- Nenhuma chamada ROS bloqueia a thread da UI; as respostas de serviços/ações chegam via sinais `service_result` / `action_result`.
- Goals são rastreados desde a solicitação até o resultado. A aceitação possui
  timeout de 3 s; movimentos usam duração solicitada + 5 s de margem e o
  gripper usa 10 s. Timeout solicita cancelamento e produz falha visível.
- O sinal `arm_alive` (heartbeat de `/{ns}/arm_status`) indica se o controlador está de fato publicando; `move_feedback` entrega o progresso e a pose corrente da action `MoveToPose`.
- O sinal `diagnostics_changed` entrega, a cada segundo, uma fotografia somente
  leitura das métricas dos tópicos e da descoberta de serviços/actions. A
  coleta ocorre na bridge e não bloqueia a thread da interface.
- O `RosBridge` assina `/robot_description` (latched pelo `robot_state_publisher`) e o `kinematics.py` extrai limites das juntas e computa a cinemática direta `base_link → gripper_link` (TCP ao vivo, jog cartesiano, teach e visualização 3D).

## Abas Visão e Pegas

A aba **Visão** implementa processamento clássico de imagem com OpenCV: brilho,
bordas, HSV, morfologia, filtros de área, razão do BBox, extensão, solidez e
forma. **Não é necessário modelo de rede neural** — o pipeline foi pensado para
rodar em um Raspberry Pi 5 com câmera local.

O modo recomendado para operação é `manual`: a imagem da webcam ou do teste fica
disponível em `/camera/image_raw`, mas a segmentação só é executada quando a GUI
chama **Processar 1 frame** (serviço `/rebotarm_segmentation/process_frame`).

### Fluxo recomendado na aba Visão

1. Abra a aba **Visão**.
2. Publique a imagem padrão já carregada ou escolha outra imagem local. A imagem
   padrão fica em `src/rebotarm_vision/test_images/default_vision_test.png`;
   quando ela não existe, a GUI usa o fallback `garrafas.png`.
3. Em **Limites de operação**, selecione `limite desenhado`.
4. Escolha `polygon` ou `circle`, clique em **Desenhar na imagem** e marque a
   área útil de trabalho.
5. Clique em **Aplicar limite**. O limite vira máscara no segmentador e também
   é enviado para a aba **Pegas**.
6. Preencha o **Prompt do cenário** e pressione **Auto-calibrar com IA**.
   O `vision_calibration_node` executará o loop iterativo e retornará o perfil
   calibrado.
7. Ao convergir, revise a imagem anotada e clique em **Aprovar calibração**.
   Se necessário, use **Repetir** ou **Complementar texto**.
8. Clique em **Processar 1 frame** para validar a detecção.
9. Abra a aba **Pegas** e clique em **Recalcular estratégias**.
10. Selecione uma estratégia válida e envie/salve o programa de movimento.

Caso precise ajustar manualmente, os controles clássicos continuam disponíveis
como fallback.

> Durante a execução da coleta **não processe novos frames**. Uma nova leitura
> de câmera deve acontecer somente quando o fluxo de movimento solicitar a etapa
> de visão.

### Parâmetros principais da aba Visão

| Campo | Significado | Quando ajustar |
|-------|-------------|----------------|
| `opencv_mode` | Modo de segmentação (`bright`, `edge`, `bright_or_edge`, `hsv`, `hsv_or_edge`, `hsv_or_bright`). | Troque conforme objeto/fundo. |
| `min_area_px` | Área mínima do contorno em px². | Aumente para eliminar tampas/reflexos pequenos. |
| `max_area_px` | Área máxima absoluta em px² (`0` = desligado). | Útil para rejeitar dois objetos colados. |
| `brightness_threshold` | Limiar de brilho. | Ajuste se a iluminação mudar. |
| `hsv_lower` / `hsv_upper` | Janela de cor. | Use no modo `hsv*` para objetos coloridos. |
| `canny_low` / `canny_high` | Limiares do detector de bordas. | Aumente se houver muito ruído; reduza se faltar borda. |
| `blur_kernel` | Suavização antes da segmentação. | Aumente para reduzir ruído (perde detalhes). |
| `morph_kernel` | Kernel de abertura/fechamento. | Aumente para unir/tirar pequenos buracos. |
| `morph_open_iterations` | Iterações de abertura. | Aumente para separar objetos levemente grudados. |
| `morph_close_iterations` | Iterações de fechamento. | Aumente para preencher buracos internos. |
| `min_solidity` | Solidez mínima do contorno. | Rejeita objetos com buracos grandes. |
| `min_extent` / `max_extent` | Extensão do contorno no bbox. | Filtra formas muito espiúricas. |
| `min_aspect_ratio` / `max_aspect_ratio` | Razão de aspecto do bbox rotacionado. | Selecione objetos mais alongados ou mais compactos. |
| `center_method` | Método de cálculo do centro. | `moments` geralmente é o mais robusto. |
| `direction_method` | Método de cálculo da orientação. | `min_area_rect` ou `pca`. |
| `direction_sense_method` | Resolve ambiguidade de 180° do eixo. | Use `bright_end`/`dark_end` se uma ponta for mais clara/escura. |
| `shape_filter` | Restringe formas. | `any`, `circle`, `square`, `rectangle`, `triangle`, `object`. |

### Configuração de visão e pega

A aba **Visão** usa um JSON unificado para carregar o modelo YOLO, a área de
trabalho e o planejamento de pega:

- `src/rebotarm_vision/config/vision_pick_yolo_config.json` — contrato atual
  para modelo, classes, disposição, clearances e rota.
- `src/rebotarm_vision/config/perfil_garrafas_deitado.json` e
  `src/rebotarm_vision/config/perfil_garrafas_em_pe.json` — perfis calibrados
  para o fallback OpenCV por orientação.
- `src/rebotarm_vision/config/vision_bottles_mixed_orientation_config.json` —
  exemplo de perfil misto para garrafas em pé e deitadas.

### Calibração pixel → XY pela GUI

A calibração liga os pixels da imagem às coordenadas do mundo (`base_link`).
Sem ela, a aba **Pegas** mostra `pose XYZ indisponível`.

Procedimento:

1. Na aba **Visão**, processe um frame e confirme que há objetos detectados.
2. Mova o braço (aba **Pose** ou **Juntas**) até um ponto conhecido da mesa.
3. Anote o `x` e `y` atuais do TCP.
4. Clique no mesmo ponto na imagem da aba **Visão** e anote `u` e `v`.
5. Repita para pelo menos 3 pontos (4 é melhor, com homografia completa).
6. Preencha os pontos em `config/vision.yaml` no grupo `rebotarm_object_pose`:
   ```yaml
   rebotarm_object_pose:
     ros__parameters:
       use_workspace_homography: true
       calibration_pixel_points: [u1, v1, u2, v2, u3, v3, u4, v4]
       calibration_xy_points:    [x1, y1, x2, y2, x3, y3, x4, y4]
       table_z: 0.0
   ```
7. Reinicie o nó de pose (ou o launch inteiro).

### Aba Pegas

A aba **Pegas** consome `/rebotarm/vision/objects` e o limite operacional vindo
da aba **Visão**. A tabela de estratégias **não é recalculada automaticamente**
quando chega um novo frame: clique em **Recalcular estratégias**.

Estratégias atuais:

- **superior vertical**: usada para objetos deitados ou indefinidos. Para
  garrafas deitadas, é a estratégia preferida; exige espaço para os dedos nas
  laterais longas.
- **lateral horizontal**: reservada para objetos classificados como `em pe`.
  A busca varre 360° ao redor do objeto, escolhe os maiores corredores livres
  sem colisão com outros BBoxes e respeita o limite operacional da mesa.
- **pela ponta**: estratégia experimental, desabilitada por padrão.

A abertura do gripper é calculada em pixels e convertida para milímetros por
`mm_per_px`. Para gerar movimentos reais, a calibração pixel → XY deve estar
ativa.

### Auto-calibração por IA

A aba **Visão** chama o serviço `~/calibrate` do nó
`vision_calibration_node` (pacote `rebotarm_vision`). O nó executa o loop
iterativo com a OpenAI e a validação local; a GUI é apenas um cliente fino.
A resposta é aplicada como parâmetros OpenCV internos; a IA não substitui o
segmentador em tempo de execução.

A chave da API é lida pelo `vision_calibration_node` (parâmetro `api_key` ou
variável de ambiente `OPENAI_API_KEY`). O launch `vision.launch.py` injeta
`OPENAI_API_KEY` automaticamente quando `use_calibration:=true`. A chave nunca
é enviada no perfil calibrado, nem gravada em logs ou mensagens ROS.

Na caixa **Auto-calibração por IA > Prompt do cenário**, informe a contagem
esperada e o tipo de objeto (por exemplo, "25 garrafas PET transparentes"). A
IA recebe a imagem, a configuração base em JSON e o schema completo dos
parâmetros calibráveis; ela deve reconhecer quais unidades estão em pé,
deitadas, horizontais, verticais ou ambíguas, preencher o polígono da área de
interesse em `scene.workspace_polygon` e devolver apenas o JSON de configuração
já sintonizado. O operador não precisa informar thresholds, kernels, filtros
HSV/Canny, vértices da ROI ou regras manuais de orientação.

Durante a calibração a GUI exibe o progresso de cada iteração. Ao final, a
imagem de debug com BBoxes, orientação, zona de pega e tolerância é exibida no
preview. O operador pode então:

- **Aprovar calibração**: confirma o perfil convergido (botão só habilitado
  quando `status.converged` for `true`).
- **Repetir**: executa novamente a auto-calibração com a mesma imagem/prompt
  ou com outro frame.
- **Complementar texto**: foca o prompt para que o operador adicione detalhes
  e pressione **Auto-calibrar com IA** novamente.

Os botões **Salvar config.** e **Carregar config.** persistem o prompt e o
perfil calibrado, incluindo dois conjuntos de parâmetros: detecção vertical e
detecção horizontal. Parâmetros como método, limitadores, filtros RGB/HSV,
bordas/brilho, forma e limite de operação não ficam expostos na GUI operacional.

### Troubleshooting das abas Visão e Pegas

| Problema | Solução |
|----------|---------|
| Imagem da câmera não aparece | Verifique se o nó de câmera está rodando, o tópico `/camera/image_raw` existe e as permissões de vídeo. |
| Botão **Processar 1 frame** não faz nada | Confira se o serviço `/rebotarm_segmentation/process_frame` está disponível (`ros2 service list`). |
| **Auto-calibrar com IA** não inicia | Verifique se o `vision_calibration_node` está rodando e o serviço `/rebotarm_vision_calibration/calibrate` está visível. |
| Calibração falha ou não converge | Confira URL, API key e modelo em **Ferramentas e ajustes → Configurações**; revise o prompt e forneça uma imagem mais nítida. |
| Nenhum objeto na tabela da Visão | Ajuste `min_area_px`, modo OpenCV e `brightness_threshold`. |
| Objetos aparecem mas são rejeitados | Leia o campo `reject_reason`; verifique limites de área, aspecto e limite operacional. |
| Aba Pegas vazia | Processe um frame e clique **Recalcular estratégias**. Verifique se há objetos `reachable`. |
| `pose XYZ indisponível` | Calibração pixel → XY não está ativa. Preencha `calibration_pixel_points`/`calibration_xy_points`. |
| Estratégias em vermelho | Limite operacional muito restrito ou clearances insuficientes. Aumente a área útil ou afaste outros objetos. |
| Frame processado durante movimento | Use `process_mode: manual` e só processe quando o braço estiver parado numa posição segura. |

### Integração com programas de movimento

A integração de visão computacional usa os tópicos do pacote
`rebotarm_vision`: `/camera/image_raw`, `/rebotarm/vision/debug_image`,
`/rebotarm/vision/objects_2d`, `/rebotarm/vision/objects` e
`/rebotarm/vision/pick_queue`. A GUI chama o serviço
`/rebotarm_segmentation/process_frame` quando o operador clica
**Processar 1 frame** na aba **Visão**.

No programa de movimento (página **Programa**) você pode usar condições do
tipo `Visão: peça detectada` e `Visão: coordenada` para tomar decisões. O fluxo
recomendado é manual por etapa: a câmera mantém o frame mais recente, mas a
segmentação só roda quando o programa solicita a leitura visual.

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
    ├── app.py             # MainWindow + shell responsivo + parada + modos da interface
    ├── ros_bridge.py      # rclpy node + QThread + sinais, heartbeat e métricas ROS
    ├── kinematics.py      # parsing de URDF, FK e limites das juntas
    ├── styles/dark.qss    # tema Catppuccin Mocha-ish
    ├── widgets/
    │   ├── status_led.py
    │   ├── joint_slider.py        # slider com botões de jog press-and-hold
    │   ├── pose_input.py
    │   ├── pose_editor.py         # editor XYZ+RPY reutilizável (página Programa)
    │   ├── program_flow_widget.py # fluxograma do programa (somente leitura)
    │   ├── step_list_editor.py    # editor reutilizável, drag/drop e atalhos
    │   └── trajectory_preview_dialog.py # prévia 3D dos waypoints compilados
    ├── motion/                 # modelo de programa e geometria de trajetórias
    │   ├── trajectory.py       # waypoints de linha / arco / paletização + escala de velocidade
    │   ├── conditions.py       # condições estruturadas para IF-ELSE (sinais, estado, visão)
    │   └── program.py          # dataclasses de etapas, opcodes da VM e serialização JSON
    └── tabs/
        ├── system_tab.py
        ├── joints_tab.py       # sliders + jog por junta com limites do URDF
        ├── pose_tab.py         # MoveToPose + TCP ao vivo + jog cartesiano
        ├── robot_3d_tab.py     # visualização 3D da postura (FK + QPainter)
        ├── motion_tab.py       # página "Programa" (programas de movimento + VM + teach)
        ├── gripper_tab.py
        ├── vision_tab.py       # visão clássica, limites, calibração e auto-tune
        ├── grasp_tab.py        # planejamento de pegadas por objeto detectado
        ├── settings_tab.py     # configurações gerais/OpenAI
        ├── safety_tab.py       # perfis de torque WARN/FAIL e alarme
        ├── telemetry_tab.py
        ├── diagnostics_tab.py   # heartbeat, frequência, latência e endpoints ROS
        └── mit_tab.py
```

## Página Programa (programas de movimento)

A página **Programa** permite compor um *programa* de etapas de movimento,
parametrizar cada etapa, executar contra a action `MoveToPose` e persistir o
programa num arquivo de configuração **JSON** (salvar, carregar ou
pré-carregar automaticamente). Além dos movimentos, o programa suporta **fluxo
de controle** (`LOOP`, `IF-ELSE`) e **esperas** (`WAIT`, `WAIT_FOR_STATE_MACHINE`,
`WAIT_FOR_SIGNAL`), **acionamento de saídas** (`OUTPUT_ON`/`OUTPUT_OFF`/
`OUTPUT_TOGGLE`), e uma **escala de velocidade global** aplicada a toda a
rotina.

### Teach (ensinar pontos com a mão)

No estilo dos teach pendants de mercado (UR/FANUC):

- **Freedrive**: toggle que liga/desliga a compensação de gravidade
  (`/rebotarm/gravity_compensation/start|stop`), permitindo mover o braço
  manualmente.
- **Ensinar ponto atual**: captura a pose atual do TCP (cinemática direta do
  URDF; fallback para a última pose atingida) e anexa uma etapa `move` ao
  programa.

### Visão em fluxograma

O toggle **Lista / Fluxo** alterna entre o editor de etapas e um fluxograma
somente leitura do programa (`ProgramFlowWidget`), com blocos coloridos por
categoria (movimento, gripper, esperas/IO, controle) e destaque da etapa em
execução — inspirado em interfaces como TMflow.

### Edição rápida e pré-visualização

- Arraste uma etapa na lista para reordená-la. O recurso também funciona nas
  sublistas de `LOOP` e nos ramos de `IF-ELSE`, e pode ser desfeito com
  `Ctrl+Z`.
- Com a lista em foco, `Ctrl+D` duplica a etapa selecionada; `Delete` ou
  `Backspace` remove. A duplicação cria uma cópia independente, inclusive para
  blocos com sub-etapas.
- **Executar da etapa** inicia na etapa top-level selecionada e segue até o
  fim. A confirmação, o preflight, o progresso e o destaque consideram apenas
  esse sufixo; a seleção nunca pula diretamente para dentro de um `LOOP` ou
  ramo condicional.
- **Pré-visualizar** compila o programa sem chamar ROS e mostra em 3D os mesmos
  waypoints cartesianos usados pela VM, incluindo os pontos gerados por linha,
  arco e paletização. O resumo informa distância entre waypoints, duração de
  movimento e ações não espaciais.

Na prévia, `LOOP` e `IF-ELSE` são desenhados em ordem estrutural. Repetições e
o ramo escolhido por sensores em tempo real podem alterar o percurso efetivo;
essa limitação fica indicada no próprio diálogo.

### Escala de velocidade global

O campo **Escala de velocidade** (multiplicador, ex.: `1.0x`, `2.0x`, `0.5x`)
escala a duração de **todos** os movimentos do programa de forma homogênea:
valores `>1` aceleram e valores `<1` desaceleram. Cada segmento é limitado aos
limites individuais do planejador (`0.25 s` a `60 s`), de modo que a rotina
nunca viola os limites de cada waypoint, por maior que seja a escala.

### Tipos de etapa

| Etapa | Campos |
|-------|--------|
| **Ir ao ponto** (`move`) | Pose XYZ+RPY + duração |
| **Linha reta** (`line`) | Pose inicial, pose final, orientação (inicial/final), velocidade |
| **Arco circular** (`arc`) | 3 poses (P1/P2/P3), nº de amostras, velocidade, *orientação apontando para o centro* (soldagem) |
| **Paletização 3D** (`pallet`) | Canto de origem, nx/ny/nz, espaçamento dx/dy/dz, velocidade, altura de aproximação, padrão *snake* |
| **WAIT (tempo)** (`wait`) | Tempo (s) |
| **WAIT_FOR_STATE_MACHINE** (`wait_sm`) | Estado alvo, timeout (0 = infinito) |
| **WAIT_FOR_SIGNAL** (`wait_sig`) | Entrada digital (IN1–IN5), valor esperado, tipo de espera (nível/borda de subida/borda de descida), timeout |
| **Saída digital** (`output`) | Saída (OUT1–OUT5) + ação: `OUTPUT_ON` / `OUTPUT_OFF` / `OUTPUT_TOGGLE` |
| **LOOP (repetir)** (`loop`) | Nº de repetições (ou *para sempre*) + sub-etapas aninhadas |
| **IF-ELSE (condicional)** (`if`) | Condição(ões) + sub-etapas `then` e `else` aninhadas |

- **Linha**: amostragem cartesiana entre início e fim; a duração de cada
  segmento é `comprimento do segmento / velocidade`.
- **Arco**: ajusta a circunferência pelos 3 pontos e percorre o arco de P1 a P3
  passando por P2. Com a opção **"Orientação apontando para o centro"** o eixo
  +Z da ferramenta é mantido apontando para o centro do círculo (ideal para
  tochas de soldagem).
- **Paletização 3D**: gera, para cada célula `(i, j, k)`, um movimento de
  aproximação (acima da célula) seguido de descida até a célula, camada a camada.

### Controle de fluxo (LOOP / IF-ELSE)

As etapas `LOOP` e `IF-ELSE` contêm suas próprias **sub-etapas** (editadas com o
mesmo editor do programa principal, de forma recursiva). A execução é feita por
uma **máquina virtual** (VM) que percorre um programa linear de *opcodes* com
contador de programa, pilha de loops e desvios:

- **LOOP**: repete as sub-etapas `N` vezes ou *para sempre*. Há um limite de
  segurança contra loops infinitos travarem a interface.
- **IF-ELSE**: avalia a **condição**; se verdadeira executa o bloco *então*,
  caso contrário o bloco *senão* (opcional).

#### Condições

As condições são **estruturadas** (montadas por combo boxes, sem texto livre).
Cada condição é `<fonte> <op> <valor>` e várias condições podem ser combinadas
com **E (todas)** ou **OU (qualquer)**:

| Fonte | Exemplo de uso |
|-------|----------------|
| Entrada digital (DI) | `IN1 == 1` — entrada opto do Raspberry Pi |
| Máquina de estados | `estado == IDLE` — aguardar braço pronto |
| Posição de junta | `joint2 > 0.5` |
| Posição do gripper | `gripper > 0.8` |
| Visão: peça detectada | `detectado == 1` |
| Visão: coordenada | `visão x > 0.2` |

> Veja a seção **Integração com programas de movimento** em **Abas Visão e
> Pegas** para o fluxo completo de visão dentro de um programa de movimento.

### Esperas (WAIT / WAIT_FOR_STATE_MACHINE / WAIT_FOR_SIGNAL)

- **WAIT (tempo)**: espera fixa em segundos.
- **WAIT_FOR_STATE_MACHINE**: aguarda a máquina de estados do braço atingir um
  estado válido (`IDLE`, `TRAJ_RUNNING`, `LOWLEVEL_STREAMING`, `GRAVITY_COMP`,
  `SAFE_HOMING`, `PROTECTIVE_STOP`), com timeout opcional.
- **WAIT_FOR_SIGNAL**: aguarda uma **entrada digital** (`IN1`–`IN5`) atingir o
  valor esperado. Suporta *nível* (valor atual), *borda de subida* (0→1) e
  *borda de descida* (1→0), com timeout opcional. Os pontos de I/O são publicados
  em ROS pelo nó `DioBridge` (ver abaixo), então funciona mesmo com a GUI rodando
  em outra máquina da rede.

### Saídas digitais (OUTPUT_ON / OUTPUT_OFF / OUTPUT_TOGGLE)

- **Saída digital** aciona um relé (`OUT1`–`OUT5`) em uma das três ações:
  `OUTPUT_ON` (liga), `OUTPUT_OFF` (desliga) ou `OUTPUT_TOGGLE` (inverte o
  estado atual). É uma etapa *fire-and-forget*: o comando é enviado ao
  `DioBridge` e o programa avança imediatamente. Saídas desabilitadas na aba
  **I/O Digital** são ignoradas.

### Execução

- **Executar** envia cada waypoint à action `MoveToPose` de forma sequencial
  (um novo waypoint só é enviado após o anterior ser concluído), honrando loops
  e condicionais.
- **Parar** interrompe o programa e cancela goals de movimento/gripper iniciados
  pela execução.
- Antes de executar, a GUI valida as pré-condições de segurança: ROS 2 conectado,
  heartbeat recente, braço habilitado, estado sem falha e action server
  `MoveToPose` realmente descoberto.
  Um diálogo resume o programa (etapas, comandos, escala de velocidade e estado
  do braço) e pede confirmação explícita.
- A barra de status mostra o progresso (`passo N/total`) e o resultado.
- Uma **barra de progresso visual** indica o percentual de comandos de ação
  executados.
- A **etapa ativa** fica destacada no editor durante a execução (apenas o nível
  raiz do programa).
- **Modo passo a passo**: quando ativado, a execução pausa antes de cada ação
  assíncrona e aguarda o clique em **Avançar**.
- **Modo simulação (dry-run)**: executa a lógica do programa sem enviar comandos
  ao braço, usando timers curtos para simular movimentos/gripper/esperas.
  Útil para validar a sequência sem movimentar o robô.
- Durante a execução, a GUI informa ao controlador a **etapa ativa** de
  segurança. O `step_id` é derivado do rótulo da etapa e inclui um índice DFS
  único; se não houver rótulo, usa `step_001_move`, `step_002_line`, etc.
  IDs duplicados por rótulo são desambiguados automaticamente. Esses IDs são
  usados pela aba **Segurança** para aplicar limites de torque diferentes por
  etapa.

### Registro de auditoria

Cada execução confirmada é persistida em JSONL com início/fim, operador,
programa e hash, etapa e parâmetros, resultado, duração, cancelamentos e
falhas. O operador pode ser ajustado no rodapé da navegação. A execução é
bloqueada se o evento inicial ou o início da próxima etapa não puder ser
gravado.

O arquivo padrão é
`~/.local/state/rebotarm_gui/audit/execution.jsonl` (ou sob
`XDG_STATE_HOME`). Em **Programa → Mais ações → Exportar auditoria CSV...** é
possível gerar uma cópia tabular, mantendo parâmetros estruturados como JSON.
Consulte [AUDIT_LOG.md](AUDIT_LOG.md) para o esquema, permissões, resultados e
orientações de retenção.

### Gravação e plot das juntas

A página **Programa** pode gravar os ângulos das juntas durante toda a execução
de um programa e exibir um gráfico temporal depois:

1. Marque o checkbox **Gravar juntas** antes de executar.
2. Execute o programa normalmente (real ou dry-run).
3. Ao final, clique em **Plotar juntas** para abrir um gráfico matplotlib com a
   posição de cada junta ao longo do tempo.

Isso é útil para diagnosticar trajetórias, identificar atrasos entre waypoints e
verificar se as juntas atingiram os valores esperados.

### Edição

- O editor marca o programa como modificado com um `*` no título do grupo.
- Ao carregar um novo programa com alterações não salvas, a GUI pede confirmação
  antes de descartá-las.
- **Desfazer / Refazer** (`Ctrl+Z` / `Ctrl+Y`) estão disponíveis para todas as
  alterações do programa (adição, remoção, movimentação, duplicação, edição de
  parâmetros, nome e escala de velocidade).

### Arquivo de configuração

O programa é salvo em JSON (versão 2). Exemplo com controle de fluxo e escala de
velocidade:

```json
{
  "name": "meu_programa",
  "version": 2,
  "speed_scale": 1.0,
  "steps": [
    { "kind": "move",  "label": "", "pose": {"x":0.30,"y":0.0,"z":0.30,"roll":0,"pitch":0,"yaw":0}, "duration": 2.0 },
    { "kind": "wait",  "label": "", "time": 0.5 },
    { "kind": "wait_sm", "label": "", "state": "IDLE", "timeout": 5.0 },
    { "kind": "wait_sig", "label": "", "pin": 17, "value": 1, "edge": "rising", "timeout": 10.0 },
    { "kind": "line",  "label": "", "start": {"x":0.30,"y":-0.10,"z":0.30,"roll":0,"pitch":0,"yaw":0},
                            "end":   {"x":0.30,"y": 0.10,"z":0.30,"roll":0,"pitch":0,"yaw":0}, "speed": 0.10 },
    { "kind": "arc",   "label": "", "p1": {...}, "p2": {...}, "p3": {...},
                            "speed": 0.08, "samples": 24, "point_to_center": true },
    { "kind": "pallet","label": "", "corner": {...}, "nx":3,"ny":3,"nz":1,
                            "dx":0.05,"dy":0.05,"dz":0.04,"speed":0.12,
                            "approach_height":0.05,"snake":true },
    { "kind": "loop",  "label": "pick_loop", "count": 4,
      "steps": [
        { "kind": "move", "label": "", "pose": {...}, "duration": 1.0 },
        { "kind": "wait_sig", "label": "", "pin": 18, "value": 1, "edge": "level", "timeout": 0.0 }
      ] },
    { "kind": "if",    "label": "decide",
      "condition": { "combiner": "all", "items": [
        { "source": "signal", "params": {"pin": 17}, "op": "==", "value": 1, "value_type": "number" }
      ] },
      "then_steps": [ { "kind": "move", "label": "", "pose": {...}, "duration": 1.0 } ],
      "else_steps": [ { "kind": "wait", "label": "", "time": 1.0 } ] }
  ]
}
```

- **Salvar...** / **Carregar...** usam diálogo de arquivo (qualquer caminho).
- **Definir como pré-carga** salva o programa em
  `~/.config/rebotarm_gui/motion_program.json` (ou `$XDG_CONFIG_HOME`), que é
  **carregado automaticamente** ao abrir a aba.
- O formato atual usa `wait` para esperas temporizadas e IDs lógicos de I/O
  como `DI1`/`OUT1`.

> 📂 **Arquivo de exemplo pronto para testar**:
> `src/rebotarm_gui/examples/exemplo_programa.json`
> (pick-and-place simples com movimentos, gripper, saída digital e um loop).
> Basta clicar em **Carregar...** na página **Programa** e selecioná-lo.

### Sinais digitais do Raspberry Pi (DioBridge)

A etapa `WAIT_FOR_SIGNAL` lê as **entradas digitais (GPIO)** do Raspberry Pi. Como
a GUI pode rodar em outra máquina, os pinos são expostos em ROS por um nó
separado, o **`DioBridge`** (pacote `rebotarmcontroller`), que roda no próprio Pi:

```bash
# No Raspberry Pi:
ros2 run rebotarmcontroller DioBridge
# Pinos BCM padrão: [17, 18, 27]. Para customizar:
ros2 run rebotarmcontroller DioBridge --ros-args -p input_pins:=[17,18,27,22]
```

Esse nó publica `/rebotarm/dio/state` (`rebotarm_msgs/DigitalIOState`) com os
níveis dos pinos configurados. Ele usa `gpiozero` quando disponível; caso
contrário (ex.: desenvolvimento em notebook), roda em *dry-run* publicando
níveis baixos, mantendo o tópico vivo. O `mock_arm` também publica esse tópico
(simulando o pino 17 alternando) para testes sem hardware.

## Aba Segurança (perfis de torque WARN/FAIL)


A aba **Segurança** implementa monitoramento de torque baseado em execuções
supervisionadas. A ideia é aprender o torque normal de cada etapa do programa e
comparar a execução real contra dois envelopes:

- **WARN**: desvio moderado, default `10%`; sinaliza em amarelo e registra
  evento, mas não para o braço.
- **FAIL**: desvio severo, default `20%`; em modo `stop_on_fail`, após algumas
  amostras consecutivas fora do envelope, o controlador entra em
  `PROTECTIVE_STOP`.

A regra por junta, fase e etapa é:

```text
WARN = torque_normal ± max(abs(torque_normal) * warn_pct, min_margin)
FAIL = torque_normal ± max(abs(torque_normal) * fail_pct, min_margin)
```

### Modos de operação

| Modo | Comportamento |
|------|---------------|
| `off` | Perfil desativado |
| `warn_only` | Verifica WARN/FAIL e sinaliza, mas não para |
| `stop_on_fail` | Sinaliza WARN e para apenas em FAIL persistente |

O padrão seguro é `off`. O fluxo recomendado é:

1. Executar a tarefa supervisionada e gravar CSVs por etapa/payload.
2. Gerar um perfil YAML com tolerância WARN/FAIL.
3. Importar o YAML na aba **Segurança**.
4. Testar em `warn_only`.
5. Só depois usar `stop_on_fail`.

### Gravação supervisionada

Na área **Gravação supervisionada por etapa**, preencha:

- `Programa`: nome lógico do programa.
- `Step ID`: identificador da etapa, por exemplo `aproximar`, `icar`,
  `transportar`, `depositar`.
- `Payload` / `Massa`: carga esperada ou estimada por visão computacional.
- `Duração`: tempo de gravação.
- `CSV`: destino das amostras.

Ao clicar **Gravar perfil**, a GUI salva uma linha por junta/amostra contendo:

```text
run_id, program, step_id, step_label, payload_name, payload_kg,
sample_index, stamp_sec, elapsed_sec, joint_name, position, velocity, torque
```

### Geração e importação de perfil

Na área **Gerar perfil WARN/FAIL**:

- selecione um ou mais CSVs;
- ajuste `WARN %` (default `10`);
- ajuste `FAIL %` (default `20`);
- ajuste `Margem min.`, `Bins` e `Std factor`;
- clique **Gerar YAML**.

Depois, use **Importar YAML** e **Aplicar**. A aba chama o serviço
`/rebotarm/safety/set_torque_profile`.

### Verificação e reset

A área **Verificação** compara o torque atual contra o perfil carregado:

- selecione a etapa ativa;
- escolha a fase da etapa entre `0.0` e `1.0`;
- clique **Verificar agora**.

A tabela mostra torque atual, limites WARN/FAIL e status por junta. O botão
**Resetar falha** chama `/rebotarm/safety/reset_fault` e limpa
`PROTECTIVE_STOP` depois de uma parada por FAIL.

### Serviços ROS usados

| Serviço | Função |
|---------|--------|
| `/rebotarm/safety/set_torque_profile` | Importa/desativa perfil e define modo |
| `/rebotarm/safety/set_step` | Define a etapa ativa durante a execução |
| `/rebotarm/safety/check_torque_profile` | Verifica torque atual contra o perfil |
| `/rebotarm/safety/reset_fault` | Reseta uma falha de segurança |

### CLI equivalente

Também é possível gravar e gerar perfis pelo terminal:

```bash
ros2 run rebotarmcontroller TorqueProfile record \
  --duration 20 \
  --program palletizacao_cv \
  --step-id icar \
  --step-label "Icar carga" \
  --payload-name carga_visao \
  --payload-kg 0.7 \
  --output torque_runs/icar_run1.csv
```

```bash
ros2 run rebotarmcontroller TorqueProfile build \
  torque_runs/icar_run1.csv torque_runs/icar_run2.csv \
  --warn-tolerance-percent 10 \
  --fail-tolerance-percent 20 \
  --output torque_profiles/palletizacao_cv.yaml
```

Exemplo resumido do YAML gerado:

```yaml
version: 2
program: palletizacao_cv
defaults:
  warn_tolerance_percent: 10.0
  fail_tolerance_percent: 20.0
  min_margin: 0.25
  violation_samples: 3
steps:
  icar:
    label: Icar carga
    warn_tolerance_percent: 10.0
    fail_tolerance_percent: 20.0
    joints:
      joint2:
        phase: [0.0, 0.5, 1.0]
        torque_mean: [3.0, 4.2, 5.0]
        warn_min: [2.7, 3.78, 4.5]
        warn_max: [3.3, 4.62, 5.5]
        fail_min: [2.4, 3.36, 4.0]
        fail_max: [3.6, 5.04, 6.0]
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

### Injeção e testes automatizados de falhas

O mesmo mock aceita falhas determinísticas, sem alterar seu comportamento
padrão. Isso permite reproduzir problemas operacionais sem matar tópicos ou
actions manualmente:

| Opção | Falha simulada |
|---|---|
| `--heartbeat-stop-after SEGUNDOS` | interrompe apenas `arm_status`; os demais tópicos continuam vivos |
| `--hang-action NOME` | aceita a action e não entrega resultado |
| `--reject-action NOME` | rejeita o goal durante a negociação ROS |
| `--invalid-telemetry-after SEGUNDOS` | publica `JointState` truncado e com `NaN` |
| `--late-accept-action NOME --accept-delay SEGUNDOS` | atrasa a resposta de aceitação do goal |

Os nomes de action aceitos são `move_to_pose`, `follow_joint_trajectory` e
`gripper_command`. Exemplo manual:

```bash
ros2 run rebotarm_gui mock_arm --heartbeat-stop-after 3
```

A suíte `test_fault_integration.py` cria um namespace e um processo ROS novos
para cada ensaio. Ela verifica perda de heartbeat, action travada, goal
rejeitado, queda do nó durante movimento, telemetria inválida e parada enquanto
a aceitação ainda está pendente:

```bash
QT_QPA_PLATFORM=offscreen python3 -m pytest \
  src/rebotarm_gui/test/test_fault_integration.py -v
```

Telemetria `JointState` com nomes vazios/duplicados, comprimentos incompatíveis
ou valores `NaN/Inf` é descartada pela bridge. A última amostra válida é
preservada e o motivo aparece em **Diagnóstico ROS → Último erro desta sessão**.

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
colcon build --packages-select rebotarm_msgs rebotarmcontroller rebotarm_vision rebotarm_gui
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
         "command": "colcon build --packages-select rebotarm_msgs rebotarmcontroller rebotarm_vision rebotarm_gui && source install/setup.bash",
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
- A aba **Segurança** é uma camada de software para detecção/mitigação; ela não substitui botão físico de emergência, limites elétricos/mecânicos e validação de segurança do equipamento.
- A pose do TCP (aba **Pose**, jog cartesiano, teach e página **Robô 3D**) é computada por cinemática direta a partir do URDF publicado em `/robot_description`. Sem o `robot_state_publisher` ativo (ex.: apenas o `mock_arm`), esses recursos mostram o estado como indisponível e usam fallbacks — limites de junta ±3,14 rad e última pose atingida.
- A **Parada de software** cancela apenas actions conhecidas pela GUI e solicita
  `disable`; ela não atua diretamente sobre freios ou potência e não oferece a
  garantia temporal de uma função de segurança física.

## Propostas de implementação futura

Abaixo estão melhorias e funcionalidades identificadas para evoluir a GUI e o
fluxo de operação do reBotArm. Elas estão agrupadas por proximidade/impacto.

### Curto prazo (qualidade de vida e robustez)

- [x] Corrigir shadowing de `self._speed_scale` no `MotionTab`.
- [x] Adicionar diálogo de confirmação antes de executar um programa.
- [x] Validar pré-condições de execução (conexão ROS, enabled, estado do braço,
      action server).
- [x] Invalidar estado e bloquear comandos quando o heartbeat expira.
- [x] Rastrear e cancelar todas as actions da GUI, com timeout de aceitação e resultado.
- [x] Painel de diagnóstico ROS com heartbeat, métricas de tópicos, descoberta de endpoints e último erro.
- [x] Automatizar seis falhas ROS reais com injeção determinística no mock.
- [x] Reordenar etapas por arraste, adicionar atalhos de edição, execução da
      seleção e pré-visualização cartesiana 3D.
- [x] Revisar navegação por teclado, foco, contraste, nomes para leitor de tela
      e mensagens que não dependem apenas de cor.
- [x] Incluir painel e protocolo reproduzível para tarefas cronometradas com
      operadores; a coleta humana permanece pendente até haver participantes.
- [x] Tornar o cabeçalho responsivo e a página Início rolável em 1360 × 720.
- [x] Diferenciar visualmente controles desabilitados.
- [x] Indicador visual de programa modificado (`*` no título do editor).
- [x] Garantir `step_id` único mesmo com rótulos duplicados.
- [x] Alertar quando bloco `IF-ELSE` tiver condição vazia (sempre verdadeiro).
- [x] Exibir erros de pré-carga no status da aba.
- [x] Confirmar descarte de alterações não salvas ao carregar novo programa.
- [ ] Persistir histórico de arquivos recentes no menu Carregar.
- [ ] Avaliar um atalho dedicado para Executar. A parada de software permanece sem atalho
  global para não conflitar com `Esc` em diálogos e edições.
- [ ] Validar programa antes de salvar (poses alcançáveis, velocidades
      consistentes, condições bem formadas).

### Médio prazo (produtividade e supervisão)

- [x] Destacar a etapa ativa durante a execução.
- [x] Barra de progresso visual baseada nos comandos de ação.
- [x] Modo passo a passo e modo simulação (dry-run).
- [x] Undo / redo do programa (`Ctrl+Z` / `Ctrl+Y`).
- [x] Parada de software global por botão, sem atalho, e confirmação ao desabilitar.
- [x] Modos de interface Operador/Supervisor/Engenharia com bloqueio local de páginas.
- [x] Visualização 3D da postura do braço (página Robô 3D, FK do URDF).
- [x] Jog incremental por junta e jog cartesiano; limites reais do URDF nos sliders.
- [x] Teach: Freedrive + ensinar ponto atual; fluxograma do programa (leitura).
- [x] Log de auditoria JSONL da execução, etapas, operador, parâmetros,
  resultados, durações, cancelamentos e falhas, com exportação CSV.
- [x] Drag-and-drop para reordenar etapas no editor.
- [x] Atalhos para duplicar (`Ctrl+D`) e remover (`Del`) etapas.
- [x] Pré-visualização gráfica simples da trajetória (2D/3D) no editor.
- [ ] Pausa automática em condições de erro ou colisão detectada.
- [x] Modo de execução parcial (executar a partir de uma etapa selecionada).

### Longo prazo (planejamento avançado e integração)

- [~] Editor visual em diagrama de blocos para programas de movimento (visão em
      fluxograma já disponível para leitura; falta edição por arrastar-e-soltar).
- [ ] Validação cinemática e verificação de colisão no editor.
- [ ] Planejador de caminhos com evasão de obstáculos (obstacle avoidance) para movimentos livres.
- [ ] Integração com OpenAI/LLM para geração de programas a partir de descrição
      textual.
- [ ] Sincronização de programas entre múltiplas estações de operação.
- [ ] Suporte a sub-rotinas reutilizáveis (biblioteca de etapas/programas).
- [ ] Geração automática de perfis de torque a partir de execuções supervisionadas.
- [ ] Replay de execuções com possibilidade de pausa/avanço rápido.
