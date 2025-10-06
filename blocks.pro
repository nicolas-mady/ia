block(a).
block(b).
block(c).
block(d).
table(t).

size(a, 1).
size(b, 1).
size(c, 2).
size(d, 3).
size(t, 7).

slot(0).
slot(1).
slot(2).
slot(3).
slot(4).
slot(5).
slot(6).

% -----------------------------------------------------------------------------
% PARTE 2: Predicados Auxiliares (raciocínio espac)
% -----------------------------------------------------------------------------

% posição absoluta X (esquerda) de um bloco
absolute_pos(Block, State, X) :-
    member(pos(Block, table(X)), State), !.
absolute_pos(Block, State, X) :-
    member(pos(Block, on(Support)), State),
    absolute_pos(Support, State, X).

% slots ocupados por um bloco
busy_slots(Block, State, Slots) :-
    absolute_pos(Block, State, X),
    size(Block, W),
    Xend is X + W - 1,
    findall(S, between(X, Xend, S), Slots).

% slot livre?
is_free(Slot, State) :-
    \+ ( member(pos(B,_), State),
         busy_slots(B, State, Busy),
         member(Slot, Busy)
       ).

% faixa de slots livre?
is_range_free([], _).
is_range_free([S|Rest], State) :-
    is_free(S, State),
    is_range_free(Rest, State).

% -----------------------------------------------------------------------------
% PARTE 3: Operadores de Ação
% -----------------------------------------------------------------------------

% possíveis ações (enumeradas)
action(State, move(Block, table(X)), NewState) :-
    block(Block),
    slot(X),
    Action = move(Block, table(X)),
    possible(Action, State),
    apply(State, [Action], NewState).

action(State, move(Block, on(Target)), NewState) :-
    block(Block), block(Target),
    Block \== Target,
    Action = move(Block, on(Target)),
    possible(Action, State),
    apply(State, [Action], NewState).

% condições das ações
can(move(Block, on(Target)), [clear(Block), clear(Target), neq(Block,Target), stable(Block,Target), pos(Block,_), pos(Target,_)]).
can(move(Block, table(X)), [clear(Block), pos(Block,_), space_available(Block,X), slot(X)]).

% efeitos
adds(move(Block, Dest), [pos(Block, Dest), clear(Block)]).

deletes(move(Block, on(Target)), State, [pos(Block, OldPos), clear(Target)]) :-
    member(pos(Block, OldPos), State).
deletes(move(Block, table(_)), State, [pos(Block, OldPos)]) :-
    member(pos(Block, OldPos), State).

% -----------------------------------------------------------------------------
% PARTE 4: Predicados de verificação
% -----------------------------------------------------------------------------

% pré-condições avaliáveis
holds(_, neq(A,B)) :- A \== B.
holds(_, stable(B1,B2)) :- size(B1,W1), size(B2,W2), W1 =< W2.
holds(State, space_available(Block,X)) :-
    size(Block,W),
    Xend is X + W - 1,
    size(t, TW), X >= 0, Xend < TW,
    findall(S, between(X, Xend, S), Slots),
    is_range_free(Slots, State).
holds(State, pos(B,Arg)) :- member(pos(B,Arg), State).
holds(State, clear(B))   :- member(clear(B), State).
holds(_, slot(X))  :- slot(X).

% ação executável?
possible(Action, State) :-
    can(Action, Preconditions),
    forall(member(P, Preconditions), holds(State, P)).

% aplicar lista de ações a um estado
apply(State, [], State).
apply(State, [Action|Rest], NewState) :-
    deletes(Action, State, Del),
    adds(Action, Add),
    ( member(pos(_, on(OldSupport)), Del) ->
        AddFinal = [clear(OldSupport)|Add]
    ; AddFinal = Add
    ),
    subtract(State, Del, Tmp),
    append(Tmp, AddFinal, S),
    sort(S, Mid),
    apply(Mid, Rest, NewState).

% metas satisfeitas?
satisfied(State, Goals) :-
    forall(member(G, Goals), member(G, State)).

% -----------------------------------------------------------------------------
% PARTE 5: Planejador A*
% -----------------------------------------------------------------------------

% nó: [State, Plan, G, H, F]

a_star_plan(start, Goals, Plan) :-
    heuristic(start, Goals, H0),
    astar([[start, [], 0, H0, H0]], [], Goals, RevPlan),
    reverse(RevPlan, Plan).

astar([[State, Plan, _, _, _]|_], _, Goals, Plan) :-
    satisfied(State, Goals), !.
astar([[State, Plan, G, _, _]|Rest], Closed, Goals, Sol) :-
    memberchk(State, Closed), !,
    astar(Rest, Closed, Goals, Sol).
astar([[State, Plan, G, _, _]|Rest], Closed, Goals, Sol) :-
    findall([NewState, [Act|Plan], G2, H2, F2],
            ( action(State, Act, NewState),
              \+ memberchk(NewState, Closed),
              G2 is G + 1,
              heuristic(NewState, Goals, H2),
              F2 is G2 + H2 ),
            Succ),
    append(Rest, Succ, Open1),
    sort_nodes(Open1, Open2),
    astar(Open2, [State|Closed], Goals, Sol).

% Ordena a lista de nós pelo valor de F (5º elemento)
sort_nodes(Nodes, Sorted) :-
    map_list_to_pairs(node_key, Nodes, Pairs),
    keysort(Pairs, SortedPairs),
    pairs_values(SortedPairs, Sorted).

node_key([_,_,_,_,F], F).

% heurística: nº de objetivos não satisfeitos
heuristic(State, Goals, H) :-
    include(is_satisfied(State), Goals, SatList),
    length(Goals, Tot),
    length(SatList, Sat),
    H is Tot - Sat.
is_satisfied(State, Goal) :- member(Goal, State).
% -----------------------------------------------------------------------------
% PARTE 6: Situações (iniciais e finais)
% -----------------------------------------------------------------------------

% Situação 1
s1_start([pos(d,table(3)), pos(c,on(d)), pos(a,on(c)), pos(b,on(a)), clear(b)]).
s1_goal([pos(c,table(0)), pos(d,table(2)), pos(a,on(c)), pos(b,table(5))]).

% Situação 2
s2_start([pos(c,table(0)), pos(a,on(c)), pos(b,on(a)), pos(d,table(3)), clear(b), clear(d)]).
s2_goal([pos(c,table(0)), pos(d,table(3)), pos(b,table(2)), pos(a,on(b))]).

% Situação 3
s3_start([pos(c,table(0)), pos(a,table(3)), pos(b,table(5)), pos(d,on(a)), clear(c), clear(b), clear(d)]).
s3_goal([pos(c,table(0)), pos(a,on(c)), pos(d,table(2)), pos(b,table(5))]).

% -----------------------------------------------------------------------------
% Consultas exemplo:
% s1_start(S), s1_goal(G), a_star_plan(S,G,Plan).
% s2_start(S), s2_goal(G), a_star_plan(S,G,Plan).
% s3_start(S), s3_goal(G), a_star_plan(S,G,Plan).
% -----------------------------------------------------------------------------
