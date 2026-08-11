# Revisão de UX da GUI reBotArm

Data da revisão: 17 de julho de 2026
Escopo: interface desktop PySide6 para operação, programação, visão e diagnóstico do reBotArm.

## Objetivo

Reorganizar uma interface que havia sido dimensionada para toque, mas acumulou funções de
operação, engenharia e diagnóstico. A nova direção assume mouse e teclado e busca:

- manter o fluxo diário imediatamente visível;
- manter estado do robô e comandos essenciais em contexto permanente;
- retirar configurações e comandos de baixo nível da navegação principal;
- reduzir altura, margens e tipografia dos controles sem prejudicar leitura;
- preservar todas as funcionalidades existentes e seus componentes ROS 2.

Esta GUI não é um dispositivo de segurança funcional e não substitui parada de emergência,
dispositivo de habilitação, enclausuramento, PLC de segurança ou avaliação de risco da célula.

## Diagnóstico anterior

O layout anterior apresentava 12 abas horizontais com o mesmo peso visual. A primeira aba era
`Config. gerais`; tarefas frequentes, como Visão, Pegas e Planejamento, competiam com
Telemetria, MIT/Raw e configurações. Em uma janela de 1600 × 900 px, a barra de abas já
consumia praticamente toda a largura.

O tema ainda estava calibrado para touch:

- fonte global de 13 pt;
- abas e botões com altura mínima de 52 px;
- entradas com 46 px;
- sliders com alça de 34 px;
- checkboxes e radios com indicador de 26 px;
- barras de rolagem de 18 px.

Além do desperdício de espaço, o log ROS ocupava permanentemente parte da tela de Visão,
apesar de ser consultado principalmente durante diagnóstico.

## Referências de mercado

Foram comparadas fontes oficiais de quatro famílias de interfaces. O objetivo não foi copiar a
aparência, mas identificar padrões de hierarquia e operação.

| Fabricante/interface | Padrão relevante | Aplicação no reBotArm |
|---|---|---|
| Universal Robots PolyScope X | Navegação principal separa Operator, Program e visualização/movimento; Settings fica em More. Estado, Play e velocidade permanecem no rodapé. | Fluxo de operação na lateral; ferramentas/configurações em menu secundário; estado persistente. |
| FANUC CRX Tablet TP | Barra de status reúne modo, alarmes, programa e habilitação. Painéis de execução e operação do robô ficam diretamente no editor; setup fica no menu. | Cabeçalho com ROS, estado e modo; Programa e movimento no primeiro nível. |
| Doosan DART Platform | Header contém modo/status; footer contém servo, favoritos, Quick Control, execução e velocidade. Settings é um módulo separado. | Ações essenciais persistentes e acesso principal organizado por tarefa. |
| ABB FlexPendant | Start, Stop e avanço/retrocesso têm controles dedicados; funções programáveis e mensagens não disputam espaço com a área principal. | Parar/execução continuam destacados na tela Programa; diagnóstico é secundário. |

Fontes oficiais consultadas:

- [Universal Robots — ícones e hierarquia do PolyScope X](https://www.universal-robots.com/manuals/EN/HTML/SW10_9/Content/prod-usr-man/software/PolyScopeX/polyx-introduction/polyx-icons.htm)
- [Universal Robots — primeiro programa e controles persistentes](https://www.universal-robots.com/manuals/EN/HTML/SW10_12/Content/prod-usr-man/software/PolyScopeX/polyx-initialize/polyx-First-Program.htm)
- [FANUC — CRX Programming Part 1, interface do Tablet TP](https://crx.fanucamerica.com/training/programming-part-1-initial-settings)
- [Doosan — visão geral do layout do DART Platform](https://manual.doosanrobotics.com/en/user-manual/3.5.0/4-p-series/overview-of-program-s-screen-layout)
- [ABB — Product manual, FlexPendant OmniCore](https://library.e.abb.com/public/0200bc191b1e4b5486a6265600d98ad6/3HAC093167-001_en_B_Product%20Manual%2C%20FlexPendant.pdf)

## Nova arquitetura de informação

### Navegação principal

As páginas mais usadas permanecem visíveis na lateral:

1. **Início** — resumo do fluxo, atalhos de tarefa e dashboard da célula;
2. **Visão** — detecção, configuração da tarefa e área de trabalho;
3. **Pegas** — estratégias, clearances e geração do programa;
4. **Programa** — edição, simulação e execução;
5. **Pose cartesiana** — movimento manual do TCP;
6. **Juntas** — movimento manual por junta;
7. **Gripper** — controle do efetuador;
8. **Robô 3D** — postura do braço em tempo real;
9. **Segurança** — estado de torque, falhas e perfis.

O nome `Planejamento` foi apresentado como `Programa`, pois a página também carrega, salva,
simula e executa sequências; o novo nome descreve melhor o ciclo completo.

### Ferramentas e ajustes

As páginas menos frequentes foram movidas para um único menu no rodapé da lateral:

- Sistema;
- I/O digital;
- Telemetria;
- Diagnóstico ROS;
- MIT / Raw;
- Configurações.

Elas continuam a um clique de distância, mas deixam de competir visualmente com a operação.
MIT / Raw permanece acompanhado do aviso de risco e não recebeu atalho global.

### Estado persistente

O cabeçalho apresenta em qualquer página:

- conexão e namespace ROS;
- estado habilitado/desabilitado do braço;
- modo atual;
- **Ativar**;
- **Home seguro**, com confirmação antes de solicitar movimento;
- **Desabilitar**, também com confirmação;
- **Parada de software**, sempre visível e sem atalho global, para preservar
  `Esc` como cancelamento de diálogos/edições.

A barra inferior mantém o resumo de conexão e permite abrir **Eventos ROS**. O log foi removido
da composição fixa de Visão e passou para um painel inferior oculto.

### Tela inicial

A nova página Início explicita dois grupos:

- fluxo principal: Visão → Pegas → Programa;
- controle manual: Pose → Juntas → Gripper.

Ao gerar um programa em Pegas, a interface carrega o resultado e navega automaticamente para
Programa, reduzindo uma etapa manual.

## Mouse, teclado e densidade

O tema foi recalibrado para desktop:

| Elemento | Antes | Depois |
|---|---:|---:|
| Fonte global | 13 pt | 10 pt |
| Botão padrão | 52 px | 34 px |
| Entrada | 46 px | 32 px |
| Indicador checkbox/radio | 26 px | 16 px |
| Alça de slider | 34 px | 18 px |
| Barra de rolagem | 18 px | 12 px |

Botões de movimento e execução permanecem um pouco maiores que controles comuns. A barra da
página Programa foi dividida em duas linhas: execução e modos na primeira; arquivo, edição e
gravação na segunda. Isso mantém ações frequentes visíveis sem corte horizontal.

### Atalhos

| Atalho | Ação |
|---|---|
| `Esc` | Cancela/fecha o diálogo ou a edição atual; não aciona a parada de software |
| `Ctrl+1` | Início |
| `Ctrl+2` | Visão |
| `Ctrl+3` | Pegas |
| `Ctrl+4` | Programa |
| `Ctrl+5` | Pose cartesiana |
| `Ctrl+6` | Juntas |
| `Ctrl+7` | Gripper |
| `Ctrl+8` | Robô 3D |
| `Ctrl+9` | Segurança |
| `Ctrl+,` | Configurações |
| `Ctrl+L` | Mostrar/ocultar Eventos ROS |
| `Ctrl+Shift+A` | Abrir Alarmes |
| `F6` / `Shift+F6` | Alternar regiões de foco |
| `F1` | Ajuda de teclado |
| `Ctrl+Z` | Desfazer alteração no Programa |
| `Ctrl+Y` | Refazer alteração no Programa |

Movimento, Home, habilitação, desabilitação, execução real e parada de software não receberam
atalhos globais. Isso reduz acionamento acidental pelo teclado e preserva `Esc` para cancelar
diálogos e edições. Home seguro e Desabilitar exigem confirmação em todos os pontos de entrada.

## Arquivos alterados

- `rebotarm_gui/app.py`: shell desktop, navegação por tarefas, tela inicial, estado persistente,
  atalhos, menu avançado e painel de eventos;
- `rebotarm_gui/styles/dark.qss`: densidade desktop e estilos do novo shell;
- `tabs/vision_tab.py`: log opcional, painel lateral responsivo e textos mais curtos;
- `tabs/motion_tab.py`: barra de ações em duas linhas;
- `tabs/grasp_tab.py`: terminologia Programa;
- abas Sistema, Juntas, Pose, Gripper, Telemetria e MIT/Raw: redução de alturas explícitas;
- `widgets/joint_slider.py`: slider mais compacto para mouse;
- `test/test_main_window_navigation.py`: cobertura da hierarquia e do painel de eventos.

Rodada de melhorias por benchmark (ver seção homônima abaixo):

- `rebotarm_gui/kinematics.py` (novo): parsing de URDF, cinemática direta e limites de juntas;
- `rebotarm_gui/ros_bridge.py`: heartbeat de `arm_status` (`arm_alive`), feedback da action
  MoveToPose (`move_feedback`), métricas de tópicos, descoberta de endpoints,
  parada de software, limites URDF e FK do TCP;
- `rebotarm_gui/app.py`: botão de parada de software sem atalho, confirmação de Desabilitar,
  LED por heartbeat, modos Operador/Supervisor/Engenharia, dashboard e página Robô 3D;
- `tabs/robot_3d_tab.py` (novo): visualização 3D da postura do braço;
- `tabs/joints_tab.py`: jog press-and-hold e limites do URDF; `widgets/joint_slider.py`:
  botões de jog e range dinâmico;
- `tabs/pose_tab.py`: TCP ao vivo, progresso do movimento e jog cartesiano;
- `tabs/gripper_tab.py`: posição unificada em mm;
- `tabs/motion_tab.py` + `widgets/program_flow_widget.py` (novo): Freedrive, ensinar ponto e
  visão em fluxograma;
- `widgets/step_list_editor.py` + `widgets/trajectory_preview_dialog.py`:
  reordenação por arraste, atalhos contextuais, execução a partir da seleção e
  prévia 3D baseada nos opcodes compilados;
- `rebotarm_gui/settings.py`: perfil de usuário persistido e `settings.json` com 0600;
- `test/test_robot_3d_tab.py` (novo) e ampliação de `test_main_window_navigation.py`.
- `tabs/diagnostics_tab.py` e `test/test_diagnostics_tab.py`: painel somente
  leitura para heartbeat, frequência, idade/latência, serviços, actions e último erro.

## Validação

Validações previstas para cada alteração de interface:

```bash
colcon build --packages-select rebotarm_gui
source install/setup.bash
QT_QPA_PLATFORM=offscreen python3 -m pytest \
  src/rebotarm_gui/test/test_main_window_navigation.py \
  src/rebotarm_gui/test/test_vision_tab_calibration.py -v
```

Também deve ser executado o teste de integração com o mock para validar execução do Programa:

```bash
QT_QPA_PLATFORM=offscreen python3 -m pytest \
  src/rebotarm_gui/test/test_mock_integration.py -v
```

Na inspeção visual, verificar ao menos 1360 × 720 e 1920 × 1080, as páginas Início, Visão,
Pegas, Programa, Segurança e o menu Ferramentas e ajustes.

## Rodada de melhorias (benchmarks de mercado)

Implementadas após comparação com PolyScope X (UR), CRX Tablet TP (FANUC), DART (Doosan)
e FlexPendant (ABB):

- **Parada de software**: botão vermelho fixo solicita cancelamento de todas as actions
  rastreadas e `disable` (`RosBridge.emergency_stop`). O rótulo explicita que não é um E-stop físico
  e não há atalho global, preservando `Esc` para cancelar diálogos e edições.
- **Confirmações**: Desabilitar e Home seguro pedem confirmação em todas as páginas; zerar uma
  ou todas as juntas também exige confirmação.
- **LED por heartbeat**: o indicador de conexão passou a refletir o heartbeat de
  `/{ns}/arm_status` (timeout de 1 s), e não o simples `rclpy.init`.
- **Robô 3D**: nova página com a postura do braço em tempo real (cinemática direta do URDF
  via `/robot_description` + `/joint_states`, órbita e zoom).
- **Jog press-and-hold**: botões −/+ por junta (passo 1°/5°/10°) e jog cartesiano X/Y/Z
  (5/10/50 mm), nos padrões PolyScope/CRX.
- **Limites reais das juntas**: sliders da página Juntas usam os limites do URDF.
- **TCP ao vivo**: página Pose mostra a pose do TCP por FK e o progresso do MoveToPose
  (feedback da action, antes descartado).
- **Teach**: toggle Freedrive (compensação de gravidade) e "Ensinar ponto atual" na página
  Programa; visualização do programa em fluxograma (somente leitura), estilo TMflow.
- **Modos da interface**: Operador / Supervisor / Engenharia bloqueiam localmente páginas de
  engenharia (MIT/Raw, ferramentas), mas não autenticam o usuário; persistidos em `settings.json`.
- **Dashboard na página Início**: estado do braço, modo, programa e objetos detectados.
- **Higiene**: i18n PT-BR completo nas abas manuais, gripper unificado em mm, matplotlib/numpy
  declarados como dependências, `settings.json` com permissão 0600, código morto removido
  da aba Visão, line endings normalizados.

Arquivos novos: `kinematics.py` (URDF + FK), `tabs/robot_3d_tab.py`,
`widgets/program_flow_widget.py`, `test/test_robot_3d_tab.py`.

## Revisão de robustez operacional — 20 de julho de 2026

- **Prontidão centralizada**: conexão ROS, heartbeat recente, enable, estado do
  braço e descoberta do action server formam uma única política de prontidão.
- **Falha fechada por heartbeat**: ao expirar `arm_status`, a interface invalida
  o estado antigo, bloqueia comandos, desativa jog/freedrive/raw e interrompe
  execuções reais ativas.
- **Ciclo de vida das actions**: goals de pose, trajetória e gripper são
  rastreados desde a aceitação até o resultado. Aceitação expira em 3 s;
  movimento expira em duração + 5 s e gripper em 10 s.
- **Cancelamento global**: Parada de software cancela actions aceitas e também
  marca solicitações ainda pendentes para cancelamento imediato se forem
  aceitas tardiamente, depois solicita `disable`.
- **Resultado correto do gripper**: `reached_goal` e `stalled` passam a definir
  sucesso/falha, em vez de assumir sucesso quando o resultado não possui campo
  genérico `success`.
- **Layout mínimo**: abaixo de 1700 px o cabeçalho usa duas linhas; em
  1360 × 720 (mínimo suportado) nenhum comando é truncado e a página Início
  oferece rolagem.
- **Estado desabilitado**: botões, entradas, combos e sliders bloqueados possuem
  contraste próprio e não parecem acionáveis.
- **Cobertura**: testes verificam timeout, cancelamento, heartbeat, confirmações
  da página Sistema e geometria do cabeçalho compacto.
- **Auditoria operacional**: o operador fica identificável no rodapé da
  navegação; execuções e etapas geram eventos JSONL duráveis, e a exportação
  CSV está em **Programa → Mais ações**, fora da barra de execução frequente.
- **Diagnóstico ROS integrado**: a página em Ferramentas e ajustes calcula
  frequência em janela móvel de 5 s, idade desde a última mensagem e latência
  baseada em `header.stamp`; também mostra descoberta não bloqueante de todos
  os serviços/actions usados pela GUI e mantém o último erro da sessão.
- **Telemetria validada na entrada**: `JointState` incoerente (nomes vazios ou
  duplicados, vetores incompatíveis, `NaN` ou infinito) não substitui a última
  amostra segura; a bridge publica o motivo no log e no painel de diagnóstico.
- **Falhas ponta a ponta**: `test_fault_integration.py` sobe um mock isolado por
  cenário e confirma perda de heartbeat, resultado de action travado, rejeição
  de goal, desconexão durante movimento, telemetria inválida e cancelamento de
  um goal aceito depois da parada. O mock expõe opções de linha de comando para
  que os mesmos casos possam ser reproduzidos na bancada.
- **Editor produtivo**: etapas top-level e aninhadas podem ser reordenadas por
  arraste com undo; `Ctrl+D`/`Delete` operam somente quando a lista tem foco.
  A execução pode começar na etapa top-level selecionada, usando preflight e
  contagem de progresso apenas do trecho restante.
- **Prévia da trajetória**: um diálogo 3D reutiliza a compilação real de
  `MoveStep`, linha, arco e paletização, informa distância/duração e deixa
  explícita a natureza dinâmica de LOOP/IF. A barra da página Programa foi
  redistribuída e inspecionada em 1360 × 720 sem truncar comandos.
- **Acessibilidade**: todos os controles visíveis alcançáveis por teclado nas
  15 páginas possuem nome textual; formulários associam rótulo e campo, o foco
  de 2 px cobre todos os tipos de controle e alarmes/heartbeat usam anúncios da
  API de acessibilidade do Qt. LEDs combinam texto, forma e cor.
- **Contraste**: pares de texto ativos do tema escuro foram medidos com razão
  mínima de 4,5:1; o indicador de foco mantém pelo menos 3:1 contra o fundo.
- **Avaliação humana reproduzível**: `--usability-test` abre um painel com seis
  tarefas, cronômetro, erros, ajuda e exportação JSON anonimizada. O protocolo e
  a matriz de leitores estão em `ACCESSIBILITY_USABILITY.md`.

## Rodada de produtividade — 22 de julho de 2026

Quatro melhorias de alto impacto e baixo esforço, sem alterar o fluxo seguro
nem a semântica de execução:

- **Persistência de sessão** (`app.py`): geometria da janela, disposição dos
  docks (Eventos ROS, Alarmes) e última página são salvos ao fechar e
  restaurados na próxima abertura via `QSettings`. A restauração só ocorre no
  entry point real (`restore_session=True`); testes continuam abrindo em Início
  para permanecer determinísticos. Página restaurada respeita o perfil ativo
  (cai para Início se tiver sido bloqueada).
- **Programas recentes** (`motion_tab.py`): "Carregar" virou botão dividido —
  o corpo abre o diálogo, a seta lista os últimos 8 programas usados (mais
  recente primeiro), com reabertura em um clique. Entradas inexistentes são
  podadas. Histórico em `QSettings`, separado do sensível `settings.json`.
- **Iconografia** (`icons.py` + `icons/*.svg`): set SVG monocromático com
  degradação graciosa (sem ícone nunca quebra layout/testes). Aplicado à
  navegação, ao menu Ferramentas e às ações frequentes (Executar, Parar,
  Salvar, Carregar, Desfazer, Refazer, Ativar, Home seguro, Desabilitar).
- **Jog por teclado** (`joints_tab.py`, `pose_tab.py`): opt-in por aba
  ("Jog por teclado"), com escopo restrito à aba focada e que nunca captura
  teclas em campos de texto. Juntas: ↑/↓ move a junta ativa, ←/→ troca de
  junta. Pose: ←/→ X, ↑/↓ Y, Page Up/Down Z. Reaproveita o jog press-and-hold
  e a porta de prontidão; sem atalho global de movimento.

Arquivos novos: `rebotarm_gui/icons.py`, `rebotarm_gui/icons/*.svg`,
`test/test_keyboard_jog.py`. `setup.py` passou a empacotar os ícones.

## Rodada de evolução da GUI — 26 de julho de 2026

Implementação dos itens 2 a 7 do plano de evolução, preservando o override
seguro de velocidade (item 1) já integrado ao controlador.

### 2. Alarmes ativos no cabeçalho

- indicador persistente com quantidade, maior severidade e ação recomendada;
- não depende apenas de cor: usa texto, ícone, forma e nome acessível;
- tooltip detalhado (origem, horário, mensagem, ação);
- ordenação ativo → severidade → horário, coalescência por identidade estável;
- reconhecido-mas-ativo permanece; normalizado sai do contador mas fica no histórico;
- layout responsivo em 1360 × 720 e anúncio acessível para novos alarmes críticos.

Arquivos: `widgets/alarm.py`, `app.py`, `styles/dark.qss`, `styles/light.qss`,
`test/test_alarms.py`.

### 3. Separação operação/configuração na Segurança

- abas internas **Operação** e **Configuração**;
- Operação mostra status do monitoramento, modo, perfil ativo, torque, último
  evento, falha e ações operacionais permitidas;
- Configuração concentra importação, aplicação, reset, gravação supervisionada,
  geração de perfil e tolerâncias por etapa;
- mutações privilegiadas exigem re-autenticação Supervisor/Engenharia via callback
  e são registradas em auditoria; arquivos sensíveis gravados atomicamente com `0600`;
- perda/expiração de autorização bloqueia edição; validação pelo backend continua
  sendo a fonte de verdade.

Arquivos: `tabs/safety_tab.py`, `app.py`.

### 4. Avaliação de usabilidade instrumentada

- painel de sessão com início/pausa/retomada/conclusão/cancelamento;
- identificador anônimo, perfil de experiência por categorias, sem dados pessoais;
- métricas por tarefa: duração, conclusão, erros, ajuda, cancelamento,
  acionamento indevido de comando crítico;
- ambiente registrado (versão, resolução, escala, tema, SO, modo de entrada);
- notas sanitizadas opcionais, exportação atômica e modo demonstração marcado
  como **DADOS SINTÉTICOS**;
- agregação por mediana/IQR, separação estrita real/sintético, sem conclusão
  automática de "validação".

Arquivo: `widgets/operator_test_panel.py`, `test/test_accessibility.py`.

### 5. Validação assistida para leitor de tela

- modo de auditoria de acessibilidade ativado explicitamente;
- varredura das 15 páginas listando controles e verificando nome, descrição,
  rótulo associado, ordem de foco, alcançabilidade por teclado, estado e papel
  acessível;
- detecção de nomes vazios/genéricos/duplicados e geração de relatórios JSON e
  Markdown com versões de Qt, PySide6, SO, sessão gráfica e escala;
- roteiro manual para Orca, NVDA/Narrator e WSLg; nenhum leitor é aprovado
  automaticamente.

Arquivo: `accessibility.py`, `test/test_accessibility.py`.

### 6. Validação do programa antes de salvar

- validação estrutural (bloqueia salvar): esquema, IDs únicos, referências,
  consistência IF/ELSE, limites de LOOP, ausência de NaN/Inf, dimensões,
  velocidades, sinais/I/O, serialização, metadados e compatibilidade de versão;
- validação operacional (permite rascunho, bloqueia liberação): alcance, juntas,
  dinâmica, calibração, envelope de visão, colisão, MoveIt, ferramenta/payload,
  perfil de torque e Produção;
- gravação atômica com `.tmp` + `replace`, sem sobrescrever arquivo anterior se
  falhar; indicador de modificado preservado; auditoria da tentativa/resultado;
- integração com preflight, mas sem diálogo invasivo para salvamentos simples.

Arquivos: `motion/program_validator.py`, `tabs/motion_tab.py`,
`test/test_program_validator.py`.

### 7. Editor visual em diagrama de blocos

- o modelo de `motion/program.py` continua sendo a fonte de verdade;
- lista e diagrama editam o mesmo modelo; seleção sincronizada nos dois lados;
- arrastar blocos reordena etapas top-level, validando destino e impedindo ciclos
  ou auto-descendência (válido apenas dentro da raiz nesta rodada);
- undo/redo integrado com um snapshot por operação; `step_id` preservado;
- zoom (+/-, Ctrl+roda, 0 para ajustar), navegação por teclado, nomes acessíveis,
  etapas inválidas destacadas, etapa ativa destacada durante execução;
- edição bloqueada durante execução; progresso não gera undo.

Limitação conhecida: edição aninhada de LOOP/IF-ELSE por drag-and-drop para
 dentro/fora dos containers e edição de propriedades por bloco ainda não estão
 implementadas; essas operações continuam disponíveis na lista aninhada.

Arquivos: `widgets/program_flow_widget.py`, `widgets/step_list_editor.py`,
`tabs/motion_tab.py`.

## Próximas melhorias recomendadas

1. Integrar autenticação/autorização do controlador se os modos da interface
   precisarem representar uma fronteira real de acesso. O seletor local não é autenticação.
2. Editor visual completo de LOOP/IF-ELSE (mover blocos para dentro/fora de
   containers, editar propriedades por bloco).
3. Validação cinemática e de colisão obrigatória no MoveIt durante execução,
   com renderização da trajetória planejada.
4. Integração com OpenAI/LLM para geração de programas a partir de descrição textual.
5. Sincronização de programas entre múltiplas estações de operação.

## Gerenciador de calibração da célula — 24 de julho de 2026

A antiga tabela de quatro correspondências pixel→XY foi substituída por um
fluxo versionado de até oito amostras:

- coleta combinada por clique na imagem e captura XYZ do TCP;
- ajuste normalizado com rejeição de amostras discrepantes;
- resíduos por ponto, RMS, erro máximo, cobertura da imagem e condicionamento;
- mínimo de cinco amostras consistentes para aprovação;
- identificação de frame da câmera, base, TCP, ferramenta, payload, plano Z,
  resolução e validade;
- revisões JSON atômicas com permissão `0600` e SHA-256;
- aprovação restrita a Supervisor/Engenharia e perfil aprovado bloqueado para
  edição até a abertura de uma nova revisão;
- indicador resumido sempre visível na barra da página Visão;
- aplicação da matriz 3×3 completa ao projetor de poses, sem o truncamento
  legado para quatro pares;
- bloqueio do Modo Produção e da liberação/execução da receita quando o perfil
  estiver ausente, vencido, adulterado ou incompatível;
- vínculo do hash da calibração ao conteúdo assinado da receita liberada.

O ajuste implementado é a calibração planar câmera→`base_link` usada nas pegas
sobre a mesa. A identificação de TCP/ferramenta faz parte da compatibilidade do
perfil; calibração volumétrica 6D hand-eye e identificação de offset do TCP por
pivoteamento continuam sendo procedimentos próprios do controlador/metrologia.

## Envelope robusto de receitas guiadas por visão — 26 de julho de 2026

A geração de movimento na página Pegas passou a produzir uma receita com um
contrato explícito de variabilidade:

- etapas relativas ao objeto são identificadas, sem deslocar home, entrega ou
  outras poses fixas;
- a área operacional desenhada em pixels é projetada para `base_link` pela
  calibração ativa;
- o operador informa incerteza adicional XY/Z e o erro máximo medido da
  calibração é incorporado automaticamente;
- ausência de área operacional calibrada mantém o fluxo de comissionamento,
  mas bloqueia a liberação para Produção com orientação acionável;
- preflight, análise da cena e dinâmica das juntas usam as mesmas variantes
  determinísticas do envelope;
- a amostra limitante, o teto de velocidade e a assinatura do envelope são
  gravados dentro do conteúdo íntegro da receita.

O desenho privilegia falha fechada e tempo de cálculo limitado no Raspberry
Pi: a análise acontece na liberação e é revalidada antes da execução, não em
cada frame da câmera. A amostragem conservadora não é apresentada como prova
contínua; MoveIt continua responsável pelo planejamento e colisão durante cada
movimento real.

## Override dinâmico seguro de velocidade — 26 de julho de 2026

O slider global passou a publicar um comando versionado para o controlador e a
mostrar o valor solicitado e o efetivamente aplicado. O controlador:

- avança um único relógio virtual para todas as juntas, preservando o caminho;
- compõe o override com o governador existente de erro de seguimento;
- limita a taxa de mudança configurada pela margem instantânea de aceleração;
- permite redução sem credencial e exige capability para aumento;
- rejeita sequências obsoletas e falha para o mínimo em payload inválido;
- publica estado latched para reconexão e diagnóstico;
- mantém compatibilidade com controlador antigo pela escala de duração da GUI.

## Tokens semânticos de tema, status consciente do tema e i18n — 27 de julho de 2026

Três melhorias convergentes sobre consistência visual e acessibilidade:

### Paleta semântica como fonte única (`styles/theme_tokens.py`)

As cores de status/severidade que viviam como literais inline em
`setStyleSheet("color: #3fb950")` — calibradas só para o tema escuro — passaram
a ser definidas uma única vez por tema. O fragmento semântico
(`styles/theme.py::build_stylesheet`) é gerado a partir dos tokens e anexado ao
QSS base em tempo de carga, de modo que `QLabel[status="ok|warn|fail|info"]` e
`#HeaderAlarmIndicatorIcon[severity="..."]` seguem sempre o tema ativo. A ajuda
de runtime `styles/styling.py::apply_status` define a propriedade e repolisha o
widget, em vez de injetar cor.

- tema escuro reproduz exatamente a paleta anterior (sem mudança visual);
- tema claro usa cores escurecidas para manter WCAG-AA (≥ 4,5:1) sobre o fundo
  `#eef1f5`, corrigindo a regressão de contraste que existia ao alternar tema.

Conversão dos sítios inline: `widgets/trajectory_preview_dialog.py` (status de
folga/MoveIt/dinâmica), `tabs/diagnostics_tab.py` (erro de sessão),
`widgets/step_list_editor.py` (aviso de IF vazio) e `widgets/alarm.py` (ícone
do indicador de alarmes por severidade).

### Guarda de paridade entre temas (`test/test_theme_parity.py`)

O `light.qss` havia driftado do `dark.qss` (faltavam `#BreadcrumbSeparator` e as
regras `[sidebar="collapsed|expanded"]`). Além de equalizá-los, um teste agora
compara o conjunto de seletores dos dois arquivos e falha se um seletor for
adicionado a um tema e esquecido no outro — a sincronização manual passou a ser
verificada pelo CI. O teste também valida contraste WCAG de todos os tokens de
status/severidade e a fidelidade da paleta escura.

### Adoção efetiva do i18n (`i18n.py`)

A fundação `tr()` existia, mas só `app.py` a usava. Agora:

- `settings.py` persiste `language` (`pt` padrão, `en`);
- `app.run()` carrega `locale/<lang>.json` na inicialização via
  `load_translations`; português continua sendo o fallback em código, então
  testes que afirmam strings em PT permanecem verdes;
- `tabs/settings_tab.py` expõe um seletor de idioma (aplicado na próxima
  abertura);
- `locale/en.json` traduz os títulos/subtítulos das 15 páginas, consumidos
  pela navegação (`navigate()` já chamava `tr(f"page.{key}.title", ...)`).

Arquivos novos: `styles/theme_tokens.py`, `styles/theme.py`, `styles/styling.py`,
`locale/en.json`, `test/test_theme_parity.py`.
