# Acessibilidade e avaliação de usabilidade

Este documento registra a implementação técnica, o protocolo de avaliação com
operadores e o estado real das evidências. Resultado humano não deve ser
preenchido por simulação ou por teste automatizado.

## Linha de base implementada

- navegação sequencial por `Tab` e `Shift+Tab` em todos os controles operáveis;
- `F6` e `Shift+F6` alternam entre navegação, conteúdo, comandos globais,
  barra de status e painéis abertos;
- atalhos de página levam o foco ao primeiro controle da página;
- `F1` abre a referência de teclado e `Ctrl+Shift+A` abre Alarmes;
- foco de 2 px em alto contraste para botões, campos, combos, caixas de
  seleção, listas, tabelas, abas e sliders;
- campos de formulários associados aos seus rótulos e controles sem texto com
  nomes acessíveis explícitos;
- páginas, tabelas, câmera, gráficos e indicadores possuem nome/descrição para
  tecnologia assistiva;
- alarmes e perda de heartbeat geram anúncios pela API de acessibilidade do Qt;
- estado não depende apenas de cor: há texto, severidade e, nos LEDs, forma
  distinta para cada classe de estado;
- pares de texto do tema escuro usados na interface atendem contraste de 4,5:1;
  o indicador de foco excede 3:1 contra o fundo.

Os testes automatizados cobrem a presença de nomes em todos os controles
visíveis e alcançáveis por teclado nas 15 páginas. Isso é uma proteção de
regressão, não substitui NVDA, Orca ou outro leitor de tela real.

## Como conduzir uma sessão cronometrada

Use preferencialmente o mock e dados de teste. Movimento físico só pode ser
incluído por pessoal autorizado, depois da avaliação de risco da célula.

```bash
source /opt/ros/lyrical/setup.bash
source install/setup.bash

# Terminal 1: controlador simulado
ros2 run rebotarm_gui mock_arm

# Terminal 2: GUI com painel moderado de avaliação
ros2 run rebotarm_gui rebotarm_gui --usability-test
```

O painel flutuante pede apenas um código anônimo, apresenta uma tarefa por vez e
registra duração, sucesso/falha, erros observados, ajuda do facilitador e notas.
Ao fim, exporta JSON versionado. Não registre nome, e-mail, voz, imagem ou outro
dado pessoal no campo de notas.

Para a última tarefa, o facilitador pode reiniciar o mock com perda programada
de heartbeat:

```bash
ros2 run rebotarm_gui mock_arm --heartbeat-stop-after 5
```

### Tarefas

1. identificar o estado da célula e abrir Diagnóstico ROS;
2. processar um frame e informar a quantidade de objetos válidos;
3. revisar uma estratégia de pega e enviá-la ao Programa;
4. validar e executar o programa em Simulação;
5. localizar Home seguro, ler a confirmação e cancelar;
6. localizar a origem e a ação recomendada de uma falha injetada.

O cronômetro começa somente em **Iniciar tarefa** e termina em **Concluir com
sucesso** ou **Encerrar com falha**. O facilitador não orienta enquanto a tarefa
está em andamento; qualquer ajuda solicitada é registrada.

## Amostra e critérios

Para uma rodada formativa, recrute pelo menos cinco pessoas que representem os
perfis Operador e Supervisor. Inclua experiência variada e, quando fizer parte
do público real, participantes que usem teclado sem mouse, alto contraste ou
leitor de tela.

Critérios iniciais para decisão:

- zero acionamento não intencional de movimento ou comando crítico;
- 100% dos participantes encontram a parada de software e entendem que ela não
  substitui o E-stop físico;
- pelo menos 90% de conclusão sem ajuda nas tarefas não críticas;
- 100% interpretam a falha sem depender apenas da cor;
- nenhum bloqueio de teclado ou leitor de tela;
- tempos devem ser comparados por mediana e faixa, nunca apenas por média.

Uma falha de segurança, bloqueio de teclado ou informação inacessível é
prioridade de correção mesmo que a taxa agregada seja alta.

## Matriz para tecnologia assistiva

Execute no ambiente de implantação real e registre versões:

| Ambiente | Entrada | Verificações |
|---|---|---|
| Linux + Orca/AT-SPI | somente teclado | ordem, nome, papel, estado, alarmes e diálogos |
| Windows + leitor adotado pela operação | somente teclado | mesmas verificações; validar especialmente a integração quando a GUI rodar via WSLg |
| Linux/Windows sem leitor | teclado e mouse | foco visível, zoom/escala, contraste e ausência de texto cortado |

Não assuma que a ponte de acessibilidade do WSLg equivale a uma aplicação Qt
nativa. Se leitor de tela for requisito operacional, a combinação exata de SO,
Qt e leitor precisa constar no relatório.

## Registro de execução humana

Estado em 20/07/2026: **aguardando participantes reais**. O painel, o protocolo
e os testes automatizados estão prontos; nenhuma métrica humana foi fabricada.

Após a rodada, mantenha os JSONs fora do repositório se contiverem comentários
que possam identificar alguém. Registre aqui apenas dados agregados:

| Data | Ambiente | Participantes | Conclusão sem ajuda | Falhas críticas | Mediana por tarefa | Decisão |
|---|---|---:|---:|---:|---|---|
| Pendente | Pendente | 0 | — | — | — | Aguardando sessão moderada |
