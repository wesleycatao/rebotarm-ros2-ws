# Registro de auditoria de execução

## Objetivo

A página **Programa** mantém um trilho persistente e anexável de todas as
execuções confirmadas. O registro cobre início e fim da execução, operador,
programa, etapa, parâmetros, resultado, duração, pedidos de cancelamento e
falhas. O movimento não começa se o evento inicial não puder ser persistido.

## Identificação do operador

O campo **Operador da auditoria** fica no rodapé da navegação principal e é
salvo nas configurações da GUI. Ele inicia com o usuário da sessão do sistema
operacional e deve ser atualizado quando houver troca de operador. O seletor
**Modo da interface** não autentica nem identifica uma pessoa e, portanto, não
substitui esse campo.

## Armazenamento JSONL

Por padrão, o arquivo é:

```text
~/.local/state/rebotarm_gui/audit/execution.jsonl
```

Se `XDG_STATE_HOME` estiver definido, a raiz passa a ser esse diretório. A
pasta é criada com modo `0700` e o arquivo com `0600`. Cada linha é um objeto
JSON independente; uma última linha incompleta, por exemplo após falta de
energia, é ignorada na exportação sem invalidar eventos anteriores.

O esquema atual é `schema_version: 1`. Os tipos de evento são:

| Evento | Conteúdo principal |
|---|---|
| `run_start` | início UTC, operador, programa, hash, modo e opções globais |
| `step_start` | etapa, tipo, índice, sequência e parâmetros efetivos |
| `step_end` | resultado, início/fim e duração da etapa |
| `cancel_requested` | origem do cancelamento e etapa ativa |
| `failure` | mensagem e contexto da falha |
| `run_end` | resultado final, início/fim e duração total |

Todos os eventos de uma execução compartilham um UUID em `run_id`. O
`program_hash` é SHA-256 do programa compilado para aquela execução. O
`step_index` é baseado em zero; `step_sequence` é baseado em um e distingue
repetições da mesma etapa em loops.

Os valores de `result` incluem `started`, `success`, `completed`, `dispatched`,
`canceled`, `failure` e `requested`. Uma saída digital real usa `dispatched`,
pois o comando existente é fire-and-forget e não fornece confirmação de estado;
na simulação o resultado é `completed`.

## Exportação CSV

Na página **Programa**, abra **Mais ações → Exportar auditoria CSV...**. A
exportação contém todos os eventos JSONL válidos. Estruturas como pose e opções
de sinal permanecem como JSON na coluna `parameters`, evitando perda de dados.
O CSV também recebe modo `0600`, e a GUI impede selecionar o próprio JSONL como
destino para evitar sobrescrever a fonte.

O CSV é uma cópia para análise; o JSONL original continua sendo a fonte de
auditoria. Preserve as permissões de acesso ao compartilhar ou arquivar ambos,
pois nomes de operadores, posições e parâmetros de processo podem ser dados
sensíveis da operação.

## Resultados e cancelamentos

- conclusão normal gera `step_end: success` e `run_end: success`;
- **Parar** gera `cancel_requested` com origem `operator`;
- perda de heartbeat gera origem `heartbeat`;
- **Parada de software** gera origem `software_stop` antes do cancelamento das
  actions e da desabilitação;
- rejeição, timeout ou erro de action gera `failure`, `step_end: failure` e
  `run_end: failure`;
- a duração usa relógio monotônico; timestamps legíveis são gravados em UTC.

## Operação e retenção

O arquivo é anexável e não é apagado nem rotacionado pela GUI. Defina a política
de retenção, cópia de segurança e assinatura externa adequada ao local. Antes de
arquivar ou truncar, feche a GUI e preserve uma cópia íntegra. Nunca edite linhas
no arquivo ativo durante uma execução.

Para diagnosticar falha de gravação, verifique espaço em disco, propriedade e
permissões do diretório. A GUI bloqueia a execução e mostra o caminho e o erro do
sistema quando não consegue gravar `run_start` ou o início da próxima etapa.
