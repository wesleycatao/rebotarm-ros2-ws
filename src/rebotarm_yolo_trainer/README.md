# reBotArm YOLO Trainer

Pacote simples para treinar e exportar modelos YOLO leves para detecção de
objetos na área de trabalho do reBotArm.

## Escopo

- **Treinamento**: roda em PC normal (com ou sem GPU NVIDIA).
- **Inferência**: modelo exportado é carregado pelo `segmentation_node` do
  `rebotarm_vision`, que publica os objetos detectados para a GUI do braço.
- **Hardware alvo**: Raspberry Pi 5 (1 frame a cada 10 segundos é suficiente,
  então usamos o formato NCNN otimizado para ARM).

## Instalação no PC de treinamento

```bash
cd ~/ros2_ws
# Recomendável usar um venv para não poluir o sistema:
python3 -m venv ~/yolo_env
source ~/yolo_env/bin/activate
pip install "ultralytics>=8.3.0" pyyaml opencv-python

# Também instale o pacote no ambiente ROS (opcional, para ros2 run):
colcon build --packages-select rebotarm_yolo_trainer --symlink-install
```

## Preparar o dataset

O dataset deve estar no formato YOLO (detecção com bounding boxes):

```text
dataset/
├── data.yaml
├── images/
│   ├── train/
│   │   ├── img_001.jpg
│   │   └── ...
│   └── val/
│       ├── img_010.jpg
│       └── ...
└── labels/
    ├── train/
    │   ├── img_001.txt
    │   └── ...
    └── val/
        ├── img_010.txt
        └── ...
```

Cada arquivo `.txt` de label contém uma linha por objeto:

```text
<class_id> <cx> <cy> <w> <h>
```

Valores normalizados `[0, 1]` em relação à imagem.

Exemplo de `data.yaml`:

```yaml
path: /home/usuario/datasets/rebotarm_pecas
train: images/train
val: images/val
test: images/test

names:
  0: peca
```

### Dicas para o seu caso

Como todos os objetos são iguais, mas em orientações diferentes:

- Use **uma única classe** (ex: `peca`).
- Inclua fotos de objetos:
  - em pé
  - deitados
  - empilhados/parcialmente oclusos
  - em diferentes posições na mesa
  - com iluminação variada
- Anote **cada objeto visível** com um bbox próprio, mesmo quando parcialmente
tampado.

## Treinar

### Comando básico

```bash
ros2 run rebotarm_yolo_trainer yolo_train --dataset /caminho/para/data.yaml
```

### Recomendado para o Raspberry Pi 5

```bash
ros2 run rebotarm_yolo_trainer yolo_train \
  --dataset /caminho/para/data.yaml \
  --model yolo11n.pt \
  --imgsz 640 \
  --export-imgsz 320 \
  --epochs 150 \
  --device 0
```

Isso treina em 640px (mais preciso) e exporta para NCNN em 320px (rápido no Pi).

### Opções principais

| Flag | Descrição |
|------|-----------|
| `--dataset` | Caminho para o `data.yaml` (obrigatório) |
| `--model` | Modelo base: `yolo11n.pt`, `yolo8n.pt`, `yolov8n.pt` |
| `--imgsz` | Tamanho de treino (640 recomendado) |
| `--export-imgsz` | Tamanho de exportação (320 recomendado para Pi) |
| `--epochs` | Épocas de treino |
| `--device` | `0` GPU, `cpu` CPU, `mps` Apple Silicon |
| `--project` | Pasta onde os runs são salvos |
| `--no-orientation-augment` | Desabilita aumentações para orientação |

Os artefatos de treino ficam em `~/rebotarm_yolo_runs/train_<modelo>_<timestamp>/`.
A pasta `deploy/` dentro do run contém o modelo pronto para copiar para o Pi.

## Inspecionar dataset

```bash
ros2 run rebotarm_yolo_trainer yolo_inspect --dataset /caminho/para/data.yaml --split train
```

## Exportar um modelo já treinado

Se quiser re-exportar um `.pt` com outro tamanho de entrada:

```bash
ros2 run rebotarm_yolo_trainer yolo_export \
  --weights ~/rebotarm_yolo_runs/train_yolo11n_.../weights/best.pt \
  --imgsz 320
```

## Copiar para o Raspberry Pi 5

Após o treinamento, copie a pasta `deploy/` para o Pi:

```bash
# No PC
scp -r ~/rebotarm_yolo_runs/train_yolo11n_20260115_143000/deploy \
  pi@raspberrypi.local:~/rebotarm_modelos/
```

No launch do Pi, configure:

```yaml
rebotarm_segmentation:
  ros__parameters:
    backend: yolo
    model_path: /home/pi/rebotarm_modelos/deploy/best_ncnn_model
    image_size: 320
    confidence_threshold: 0.35
    process_mode: manual
```

Ou use o launch pronto:

```bash
ros2 launch rebotarm_vision vision_yolo.launch.py model_path:=/home/pi/rebotarm_modelos/deploy/best_ncnn_model image_size:=320
```

## Notas

- O `segmentation_node` do `rebotarm_vision` foi adaptado para funcionar tanto
  com modelos YOLO de **segmentação** quanto de **detecção** (bbox).
- Para detecção pura, o bbox é convertido em uma máscara retangular internamente,
  permitindo reaproveitar todo o pipeline de pose/orientação do reBotArm.
- A frequência de 1 frame a cada 10 segundos é tranquila para o Pi 5, mesmo em
  PyTorch puro. O NCNN deixa a inferência ainda mais rápida e leve em CPU.
