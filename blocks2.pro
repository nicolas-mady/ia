% blocos existentes
block(a).  
block(b).  
block(c).  
block(d).

% posições na mesa
on(c, table).
on(d, table).

% relações de empilhamento
on(a, c).
on(b, c).

% blocos livres no topo
clear(a).
clear(b).
clear(d).

% (opcional) adjacência lateral
left_of(a, b).
right_of(b, a).

% (opcional) posições na mesa
pos(c, 0).   % posição inicial do bloco c
pos(d, 3).   % posição inicial do bloco d
% ------------------------------
% MUNDO DOS BLOCOS (STRIPS)
% ------------------------------

% ----- Definição das ações -----
action(
    move(X, From, To),
    [on(X, From), X \= To],
    [on(X, To), clear(From)],
    [on(X, From), clear(To)]
).

% ----- Estado inicial -----
state_init([
    on(a, c),
    on(b, c),
    on(c, table),
    on(d, table),
    clear(a),
    clear(b),
    clear(d)
]).

% ----- Estado meta -----
state_goal([
    on(a, b),
    on(b, c),
    on(c, d),
    on(d, table),
    clear(a)
]).

% ----- Checar se algo está livre -----
is_clear(X, State) :-
    member(clear(X), State).

% ----- “clear_or_table(To)” -----
is_clear_or_table(table, _).
is_clear_or_table(X, State) :-
    is_clear(X, State).

% ----- Aplicar ação -----
apply(move(X, From, To), State, NewState) :-
    member(on(X, From), State),
    is_clear(X, State),
    is_clear_or_table(To, State),
    X \= To,
    subtract(State, [on(X, From)], Temp1),
    ( To == table ->
        Temp2 = Temp1
    ;   subtract(Temp1, [clear(To)], Temp2)
    ),
    union(Temp2, [on(X, To), clear(From)], NewState).

% ----- Planejamento -----
plan(State, Goal, []) :-
    subset(Goal, State).

plan(State, Goal, [Action | Rest]) :-
    apply(Action, State, NewState),
    plan(NewState, Goal, Rest).

% ----- Versão com impressão -----
plan_verbose(State, Goal, []) :-
    subset(Goal, State),
    writeln('Meta atingida!'), !.

plan_verbose(State, Goal, [Action | Rest]) :-
    apply(Action, State, NewState),
    writeln(apply(Action)),
    plan_verbose(NewState, Goal, Rest).
