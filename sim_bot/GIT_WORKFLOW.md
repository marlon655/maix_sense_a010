Fluxo de trabalho com Git
=========================

Este projeto usa a branch `main` como branch principal estavel e a branch
`dev` como branch de desenvolvimento.

Para alteracoes do dia a dia, crie uma branch nova a partir da `dev`, faca o
commit nela e abra um Pull Request para a `dev`.


1. Criar uma branch nova a partir da dev
---------------------------------------

Atualize a branch `dev` local:

```bash
git fetch origin
git checkout dev
git pull origin dev
```

Crie uma branch nova para a sua alteracao:

```bash
git checkout -b feature/teste-pr-dev
```

Use um nome que descreva a alteracao. Exemplos:

```text
feature/ajuste-mapa
feature/corrige-launch-nav
feature/documenta-fluxo-git
```


2. Fazer commit das alteracoes
------------------------------

Depois de alterar os arquivos, confira o status:

```bash
git status
```

Adicione apenas os arquivos que devem entrar no commit:

```bash
git add README.md
```

Crie o commit:

```bash
git commit -m "Documenta fluxo de pull request"
```

OBS: Se tiver certeza que todas as alteracoes listadas no `git status` devem entrar
no commit, tambem pode usar:

```bash
git add .
```


3. Enviar a branch para o GitHub
--------------------------------

```bash
git push -u origin feature/teste-pr-dev
```

Depois, no GitHub, abra o Pull Request com:

```text
base: dev
compare: feature/teste-pr-dev
```

Isso significa que a branch `feature/teste-pr-dev` sera revisada e integrada
na branch `dev`.


4. Levar a dev para a main
--------------------------

Quando as alteracoes da `dev` estiverem testadas e prontas para virar versao
estavel, abra outro Pull Request:

```text
base: main
compare: dev
```

Depois do merge, a `main` tera as alteracoes aprovadas da `dev`.


5. Apagar a branch depois do merge
----------------------------------

Apague a branch de feature somente depois que o Pull Request dela ja tiver sido
mergeado.

Atualize a `dev`:

```bash
git checkout dev
git pull origin dev
```

Apague a branch local:

```bash
git branch -d feature/teste-pr-dev
```

Apague a branch remota no GitHub:

```bash
git push origin --delete feature/teste-pr-dev
```

Se o Git reclamar que a branch local nao foi mergeada, confira primeiro no
GitHub se o Pull Request foi realmente mergeado. Se foi mergeado e voce tem
certeza que pode apagar:

```bash
git branch -D feature/teste-pr-dev
```


Observacoes importantes
-----------------------

- Nao trabalhe direto na `main`.
- Para alteracoes comuns, abra Pull Request para `dev`.
- Para versao estavel, abra Pull Request de `dev` para `main`.
- Antes de usar `git add .`, rode `git status` para conferir se nao existem
  arquivos indesejados.
- Pastas como `build/`, `install/`, `log/` e `__pycache__/` devem ficar fora do
  commit.
