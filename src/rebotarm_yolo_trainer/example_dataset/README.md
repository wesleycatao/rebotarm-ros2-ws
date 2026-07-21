# Estrutura esperada do dataset

Crie uma pasta com a seguinte estrutura:

```text
rebotarm_pieces/
├── data.yaml          # use o exemplo ao lado como base
├── images/
│   ├── train/
│   └── val/
└── labels/
    ├── train/
    └── val/
```

Cada imagem em `images/train/` deve ter um arquivo `.txt` correspondente em
`labels/train/` com o mesmo nome base.

Exemplo de conteúdo de `labels/train/img_001.txt`:

```text
0 0.5123 0.4821 0.1234 0.3456
0 0.2345 0.6789 0.0987 0.1123
```

Cada linha representa um objeto:

```text
<class_id> <cx_norm> <cy_norm> <w_norm> <h_norm>
```

Use ferramentas como **Roboflow**, **LabelImg** ou **CVAT** para anotar.
