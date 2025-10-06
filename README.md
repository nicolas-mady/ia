# Dupla
- Guilherme Louro - 22351621
- Nicolas Mady - 22352932

# Pré-requisito
Ter o `swipl` instalado

# Como rodar
```sh
$ swipl blocks.pro
```

```prolog
?- s1_start(S), s1_goal(G), a_star_plan(S, G, Plan).
```

```prolog
?- s2_start(S), s2_goal(G), a_star_plan(S, G, Plan).
```

```prolog
?- s3_start(S), s3_goal(G), a_star_plan(S, G, Plan).
```