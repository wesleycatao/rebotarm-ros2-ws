# rebotarm_vision

Pipeline ROS 2 de visão para localizar produtos, estimar sua orientação, medir
espaços livres e produzir candidatos de pega para o reBotArm.

> **Documentação relacionada:**
>
> - Instalação no Raspberry Pi 5: [`README_RASPBERRY_PI.md`](../../README_RASPBERRY_PI.md)
> - Uso da interface gráfica: [`src/rebotarm_gui/README.md`](../rebotarm_gui/README.md)

## Diretriz do produto: visão auto-calibrada por IA

Esta seção define o comportamento desejado para a visão computacional do
projeto. A configuração operacional deve ser **totalmente calibrada por IA**.
O operador não deve editar limiares de OpenCV, filtros de cor, geometria,
clearance ou orientação.

Na aba **Visão**, as únicas entradas operacionais devem ser:

1. uma imagem típica, carregada de arquivo ou capturada da própria webcam;
2. uma descrição textual dos produtos, do layout e das regiões de pega;
3. o botão **Auto-calibrar com IA**.

Os campos **URL da API**, **API key** e **modelo** ficam exclusivamente em
**Config. gerais > API de IA**. Eles não devem ficar misturados aos controles
de visão.

Parâmetros internos continuam existindo para que o pipeline seja reproduzível,
mas não são apresentados como ajustes ao usuário. Eles são gerados pela IA,
validados localmente, versionados como um perfil de calibração e aplicados de
forma atômica.

> **Estado da implementação:** a GUI operacional oculta os controles manuais de
> método, limitadores, cor, borda/brilho, forma e limites. Esses parâmetros
> permanecem internos, são preenchidos pela IA e podem ser salvos/carregados em
> perfis de visão com dois conjuntos de parâmetros: vertical e horizontal. A
> calibração propriamente dita é executada pelo serviço
> `~/calibrate` do `vision_calibration_node`, não pela GUI.

## Configuração externa YOLO e planejamento

A aba **Visão** também aceita um modelo YOLO pré-calibrado e um único arquivo
JSON operacional. Esse fluxo é destinado a células já calibradas, nas quais a
receita vem do integrador em vez de ser produzida pela auto-calibração por IA.
O exemplo completo está em
[`config/vision_pick_yolo_config.json`](config/vision_pick_yolo_config.json).

O schema `3` reúne:

- `detector`: backend, caminho do modelo, assertividade em percentual, tamanho
  de entrada e classes de interesse;
- `object`: disposição permitida (`any`, `horizontal` ou `vertical`) e atributo
  auxiliar da receita;
- `workspace`: polígono por vértices ou círculo por centro/raio, incluindo o
  clearance aplicado como margem interna;
- `grasp`: clearance geral e mínimos específicos para cada disposição;
- `pick_route`: estratégia (`score`, vizinho mais próximo ou varredura por
  eixo), quantidade máxima e origem da rota;
- `orientation_profiles`: perfis OpenCV opcionais usados como fallback.

Ao clicar em **Aplicar configuração**, a GUI atualiza o `segmentation_node`, o
`pick_scheduler_node` e a aba **Pegas**. Alterar `model_path` recarrega o YOLO
em tempo de execução; se o carregamento falhar, a alteração é rejeitada e o
modelo anterior permanece ativo. Caminhos relativos no JSON são resolvidos em
relação ao diretório do próprio arquivo.

## Experiência do usuário

### Configuração da API

A OpenAI é chamada apenas pelo nó `vision_calibration_node`. A chave e o
endpoint são configurados através de parâmetros ROS no namespace
`rebotarm_vision_calibration`:

| Parâmetro | Default | Descrição |
|-----------|---------|-----------|
| `api_key` | `""` | Chave da API. Recomenda-se injetar pela variável de ambiente `OPENAI_API_KEY` (o launch `vision.launch.py` faz isso automaticamente quando `use_calibration:=true`). |
| `base_url` | `https://api.openai.com/v1` | URL base de um endpoint compatível com a API Responses. |
| `model` | `gpt-5.6-terra` | Modelo multimodal capaz de analisar a imagem e retornar JSON estruturado. |
| `timeout_s` | `120` | Timeout por chamada à IA. |

A chave também pode vir de `OPENAI_API_KEY` se o parâmetro `api_key` estiver
vazio. Caso o usuário permita persistência na GUI, ela deve ser armazenada pelo
mecanismo seguro da plataforma; nunca deve ser gravada em `vision.yaml`,
presets, imagens de debug, mensagens ROS ou perfis de calibração.

### Fluxo de auto-calibração

1. O operador abre a aba **Visão**.
2. Seleciona **Carregar imagem** ou **Capturar frame da webcam**.
3. Descreve os produtos, o layout, a quantidade esperada quando conhecida e
   como cada produto pode ser pego.
4. Pressiona **Auto-calibrar com IA**.
5. A GUI mostra o progresso da iteração, sem expor os parâmetros internos.
6. Ao convergir, a GUI apresenta uma única imagem com todos os BBoxes,
   orientação, zona de pega, tolerância e clearances.
7. O operador pode **Aprovar calibração**, **Repetir** com outro frame ou
   complementar o texto. Aprovar a visão não executa movimento do braço.

Não devem existir presets manuais, sliders de threshold, seleção de HSV/Canny,
edição de kernels ou campos de clearance na interface de operação.

### Prompt do cenário

O campo deve ser multilinha e aceitar texto livre, mas o operador não precisa
conhecer OpenCV nem separar manualmente os objetos em pé/deitados. A entrada
recomendada é curta:

```text
25 garrafas PET transparentes e vazias.
A zona de pega segura fica no centro do corpo de cada garrafa.
```

Durante a chamada, o nó adiciona automaticamente a imagem, as dimensões do
frame, a configuração base em JSON e o schema completo de parâmetros
calibráveis. A IA deve reconhecer na imagem quais unidades estão em pé,
deitadas, horizontais, verticais ou ambíguas, identificar o polígono da área de
interesse em `scene.workspace_polygon` e retornar somente o JSON de
configuração já sintonizado. O backend copia esse polígono para
`workspace_limit_pixels` dos perfis e ativa a máscara de workspace.

Exemplo para caixas:

```text
12 caixas retangulares de papelão.
A zona de pega é a face superior central de cada caixa.
```

Se a descrição não trouxer quantidade, escala ou margem, a IA deve estimar o
valor e registrar essa decisão no relatório da calibração. A ausência desses
dados não deve reintroduzir parâmetros manuais.

## Arquitetura do pipeline

```text
 arquivo ou webcam              Config. gerais
         │                  URL + chave + modelo
         └──────────┬───────────────┘
                    ▼
         ┌────────────────────────────┐
 prompt ─▶│ vision_calibration_node    │◀──────────────┐
  imagem  │ serviço ~/calibrate        │               │
         └───────────┬────────────────┘               │
                     │ perfil candidato (JSON)        │ imagem anotada + métricas
                     ▼                                │
         ┌────────────────────────────┐               │
         │ validação OpenCV local     ├───────────────┘
         │ horizontal + vertical      │     loop limitado
         └───────────┬────────────────┘
                     │ perfil aprovado / não aprovado
                     ▼
/camera/image_raw ─▶ segmentation_node
                     ├─▶ /rebotarm/vision/debug_image
                     └─▶ /rebotarm/vision/objects_2d
                               │
                               ▼
                      object_pose_node
                     └─▶ /rebotarm/vision/objects
                               │
                               ▼
                      pick_scheduler_node
                     ├─▶ /rebotarm/vision/pick_queue
                     └─▶ /rebotarm/vision/selected_object
```

A API de IA participa somente da auto-calibração, executada pelo nó
`vision_calibration_node`. A GUI (ou qualquer cliente ROS) chama o serviço
`~/calibrate` (`rebotarm_msgs/srv/CalibrateVision`) passando a imagem e o
prompt; o nó orquestra o loop iterativo com a IA e a validação local. Depois
que o perfil é aprovado, a detecção normal roda localmente e não envia os
frames de produção para a API.

## Loop de auto-calibração

A auto-calibração é executada pelo nó `vision_calibration_node` e não é uma
única sugestão de parâmetros: ela roda um loop com limite de iterações, tempo
e segurança:

1. O cliente (GUI) chama o serviço `~/calibrate` com a imagem original e o
   prompt do cenário.
2. O nó chama a IA, que identifica os produtos e devolve uma anotação de
   referência: classe, BBox rotacionado, eixo, orientação, zona de pega e
   confiança.
3. A IA propõe dois perfis internos, um para objetos horizontais e outro para
   objetos verticais.
4. O nó aplica os perfis no `segmentation_node`, executa a segmentação local,
   combina os resultados horizontais/verticais e gera uma imagem anotada e
   métricas objetivas.
5. A IA compara a saída com a imagem original e sua anotação de referência,
   recebendo também o perfil candidato e as métricas, corrige os perfis e
   solicita uma nova execução.
6. O processo termina quando todos os critérios de aceitação forem satisfeitos
   ou quando o limite seguro (`max_iterations` / `max_time_s`) for atingido.

Durante o loop, o nó publica o progresso em `~/progress`
(`std_msgs/String`). O resultado final é retornado no serviço como JSON no
campo `profile_json`; o campo `report` traz um resumo legível para o operador.

### Critérios de convergência

Uma calibração só pode ser marcada como aprovada automaticamente quando:

- cada produto descrito possui exatamente um BBox;
- fundo, moldura, reflexos e componentes isolados não viram produtos;
- BBoxes não unem dois produtos nem cortam a região visível relevante;
- classe e orientação são coerentes com o prompt;
- a zona de pega fica contida no produto e respeita sua tolerância interna;
- clearances são calculados contra objetos vizinhos, obstáculos e limite útil;
- não existem duplicatas entre as rotinas horizontal e vertical;
- o perfil repete o resultado dentro da tolerância nas últimas iterações.

Se o loop não convergir, a GUI deve manter o último perfil apenas como
**não aprovado**, explicar a causa em linguagem simples e pedir uma imagem
melhor ou uma descrição adicional. O robô não pode usar esse perfil para
execução automática.

## Parâmetros calibrados internamente

A IA pode escolher e combinar todos os parâmetros relevantes, incluindo:

- balanço de branco, exposição normalizada, contraste, gamma e equalização;
- filtros e intervalos de cor em RGB, HSV e Lab, inclusive múltiplas faixas
  para objetos transparentes, reflexivos ou com mais de uma cor;
- brilho global/adaptativo, inversão de máscara, Canny e operações de borda;
- blur bilateral/Gaussiano e kernels/iterações de abertura e fechamento;
- área, largura, altura, razão do BBox, solidez, extent e circularidade;
- separação de objetos encostados, preenchimento de buracos e rejeição de
  reflexos, sombras e tampas isoladas;
- método de centro, eixo principal, sentido, yaw e confiança mínima;
- tolerância angular para orientação horizontal/vertical;
- NMS/união de resultados, distância entre centros e limiar de duplicidade;
- limite útil da bancada, obstáculos, margens e clearances mínimos;
- zona de pega, tolerância interna, corredor de aproximação e afastamento das
  partes proibidas do produto.

O perfil persistido deve guardar resolução de referência, hash da imagem,
versão do schema, modelo usado, data, prompt, parâmetros dos dois detectores,
métricas e status de aprovação. A API key nunca faz parte do perfil.

## Duas rotinas de orientação

“Horizontal” e “vertical” descrevem aqui o **eixo predominante do objeto na
imagem**, não necessariamente sua pose 3D.

- `horizontal`: eixo longo mais próximo do eixo X da imagem;
- `vertical`: eixo longo mais próximo do eixo Y da imagem;
- objetos diagonais: atribuídos ao eixo mais próximo;
- objetos quase quadrados ou sem direção confiável: marcados como `ambiguous`
  e não liberados para pega até que o prompt ou outra evidência resolva a pose.

A tolerância angular é calibrada pela IA. Internamente, o pipeline executa:

1. um perfil otimizado para candidatos horizontais;
2. um perfil otimizado para candidatos verticais;
3. uma fusão por BBox rotacionado/IoU e centro, removendo duplicatas;
4. uma classificação final usando o eixo do BBox, não a largura/altura do BBox
   alinhado à imagem.

As duas rotinas publicam na mesma lista e na mesma imagem de debug. Cores e
rótulos devem distinguir `horizontal`, `vertical` e `ambiguous`, mas a numeração
dos objetos é única.

Para distinguir **produto em pé** de **produto deitado no plano físico**, o
prompt deve dizer isso explicitamente. Uma câmera RGB superior pode não conter
informação suficiente; nesses casos o sistema deve usar pistas visuais
aprendidas, câmera lateral ou profundidade, e nunca inferir uma pose 3D apenas
pelo ângulo do BBox 2D.

## Zona de pega, tolerância e clearances

Há três conceitos distintos:

- **limite útil**: região da mesa/esteira onde um produto pode ser considerado;
- **zona de pega**: polígono ou faixa dentro do produto onde o TCP/gripper pode
  ser posicionado;
- **tolerância interna**: recuo mínimo da borda da zona de pega para absorver
  erro de visão e posicionamento.

A IA deve identificar, para cada classe de produto:

- polígono da zona de pega em coordenadas normalizadas do BBox rotacionado;
- partes proibidas, por exemplo tampa, gargalo, fundo, rótulo ou selo;
- ponto preferido e intervalo permitido ao longo dos eixos curto e longo;
- tolerância interna em percentual e em pixels na resolução de calibração;
- largura necessária do gripper e corredor livre de aproximação;
- clearance até produtos vizinhos, obstáculos e limite útil da bancada.

O resultado só é `pickable` quando a zona de pega erodida pela tolerância ainda
comporta o footprint do gripper e o corredor de aproximação está livre. A
imagem de debug deve desenhar BBox, eixo, classe, zona de pega, região tolerada,
ponto preferido e clearances.

## Contrato de saída da IA

A resposta da API deve usar JSON Schema estrito. Exemplo reduzido:

```json
{
  "scene": {
    "expected_objects": 22,
    "workspace_polygon": [[0.03, 0.05], [0.96, 0.05], [0.96, 0.94], [0.03, 0.94]]
  },
  "reference_objects": [
    {
      "class": "garrafa_pet",
      "bbox_rotated": {"cx": 0.25, "cy": 0.18, "w": 0.16, "h": 0.07, "angle_deg": 18.0},
      "orientation": "horizontal",
      "grasp_zone": [[-0.25, -0.35], [0.25, -0.35], [0.25, 0.35], [-0.25, 0.35]],
      "grasp_inset_ratio": 0.10,
      "confidence": 0.98
    }
  ],
  "profiles": {
    "horizontal": {"color_spaces": ["RGB", "HSV"], "parameters": {}},
    "vertical": {"color_spaces": ["RGB", "Lab"], "parameters": {}}
  },
  "merge": {"rotated_iou_threshold": 0.45},
  "iteration": {"number": 3, "converged": true, "notes": "22 objetos separados"}
}
```

Coordenadas da cena são normalizadas entre `0` e `1`. Coordenadas da zona de
pega são relativas ao BBox rotacionado, com origem no centro, para permanecerem
válidas em objetos horizontais, verticais e inclinados.

## Calibração geométrica e segurança

A IA calibra a **detecção visual**, mas uma única imagem RGB não determina com
segurança a escala métrica nem a transformação entre câmera e `base_link`.
Esses dados pertencem à instalação da célula e ficam fora dos ajustes do
operador de visão:

- intrínsecos e distorção da câmera;
- homografia ou transformação câmera → `base_link`;
- altura da mesa e dimensões do gripper;
- limites cartesianos e zonas proibidas do braço.

Eles devem ser configurados por um procedimento de comissionamento protegido,
armazenados separadamente e validados pelo controlador de segurança. A
auto-calibração de visão nunca altera limites mecânicos, nunca habilita
`auto_execute` e nunca movimenta o braço.

## Operação ROS 2

```bash
source /opt/ros/lyrical/setup.bash
source install/setup.bash
export ROS_LOG_DIR=/home/wesley/ros2_ws/log/ros
ros2 launch rebotarm_vision vision.launch.py
```

O launch sobe o `camera_node` (ou `image_file_node`), o `segmentation_node`,
o `object_pose_node`, o `pick_scheduler_node` e o `vision_calibration_node`.

Serviços principais:

| Serviço | Tipo | Função |
|---------|------|--------|
| `/rebotarm_segmentation/process_frame` | `std_srvs/srv/Trigger` | Executa um frame de segmentação sob demanda. |
| `/rebotarm_vision_calibration/calibrate` | `rebotarm_msgs/srv/CalibrateVision` | Inicia a auto-calibração iterativa por IA. |

Tópicos principais:

```text
/camera/image_raw
/camera/camera_info
/rebotarm/vision/debug_image
/rebotarm/vision/objects_2d
/rebotarm/vision/objects
/rebotarm/vision/pick_queue
/rebotarm/vision/selected_object
/rebotarm_vision_calibration/progress
```

O serviço `CalibrateVision` recebe:

- `image`: imagem `sensor_msgs/Image` de referência;
- `prompt`: descrição textual do cenário;
- `max_iterations` (default 15): limite de iterações do loop;
- `max_time_s` (default 120): tempo máximo de execução.

E retorna:

- `success` / `message`: status da chamada;
- `profile_json`: perfil calibrado serializado (pode estar aprovado ou não);
- `report`: relatório legível das iterações.

Perfis aprovados são persistidos em:

```text
~/.config/rebotarm_vision/profiles/
```

A API key nunca é gravada nesse diretório, nem em logs ou mensagens ROS.

Teste com a imagem padrão incluída no pacote. Quando `static_image_path` não é
informado, o nó tenta usar `test_images/default_vision_test.png`; se esse arquivo
ainda não existir no workspace, usa `test_images/garrafas.png` como fallback
técnico para manter o pipeline executável.

```bash
ros2 launch rebotarm_vision vision.launch.py \
  use_camera:=false \
  use_static_image:=true \
  use_executor:=false
```

Para trocar o padrão, substitua ou adicione a imagem de referência em:

```text
src/rebotarm_vision/test_images/default_vision_test.png
```

O modo operacional continua sob demanda: o sistema captura um frame em uma
etapa segura, detecta, calcula as pegas e mantém esse snapshot durante a coleta.
Uma nova imagem só deve substituir a anterior quando o planejamento solicitar
uma nova etapa de visão.

## Diagnóstico sem parâmetros manuais

Mensagens de erro devem orientar o operador por ação, sem pedir que ele ajuste
thresholds ou kernels:

| Situação | Orientação ao usuário |
|----------|-----------------------|
| Nenhum produto detectado | Verifique se a imagem corresponde ao prompt, melhore a iluminação ou capture outro frame e repita a auto-calibração. |
| Produtos unidos ou duplicados | Afaste os produtos quando possível ou forneça uma imagem mais nítida; o loop tentará um novo perfil. |
| Orientação ambígua | Detalhe no prompt como separar horizontal/vertical ou use outra vista da câmera. |
| Zona de pega insuficiente | Revise a parte permitida no prompt ou a disposição dos produtos; não force uma pega sem clearance. |
| Perfil não convergiu | Consulte o relatório da última iteração e forneça mais exemplos do cenário. |
| Pose XYZ indisponível | Execute ou restaure a calibração geométrica protegida da célula. |
| API indisponível | Verifique URL, chave, modelo e rede; o perfil aprovado anterior pode continuar em uso. |

O relatório técnico pode registrar os parâmetros internos para auditoria e
suporte, mas a tela operacional não deve transformá-los em controles editáveis.
