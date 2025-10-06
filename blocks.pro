
% -----------------------------------------------------------------------------
% blocks_fixed.pro
% A* planner for variable-size blocks world with table slots (SWI-Prolog 9.x)
% -----------------------------------------------------------------------------

:- use_module(library(lists)).

% -------------------------
% Domain (static facts)
% -------------------------
block(a). block(b). block(c). block(d).

size(a,1). size(b,1). size(c,2). size(d,3).

% table slots 0..6; width = 7
slot(0). slot(1). slot(2). slot(3). slot(4). slot(5). slot(6).
table_width(7).

% -------------------------
% Derived spatial reasoning
% -------------------------

% absolute_pos(Block, State, X) : leftmost absolute coordinate of Block
absolute_pos(Block, State, X) :-
    member(pos(Block, table(X)), State), !.
absolute_pos(Block, State, X) :-
    member(pos(Block, on(Support)), State),
    absolute_pos(Support, State, X).

% busy_slots(Block, State, Slots) : list of occupied slots by Block
busy_slots(Block, State, Slots) :-
    absolute_pos(Block, State, X),
    size(Block, W),
    Xend is X + W - 1,
    findall(S, between(X, Xend, S), Slots).

% is_free(Slot, State) : true if no block covers Slot in State
is_free(Slot, State) :-
    \+ ( member(pos(B,_), State),
         busy_slots(B, State, Busy),
         member(Slot, Busy)
       ).

% is_range_free(Slots, State) : all slots in list are free
is_range_free([], _).
is_range_free([S|Rest], State) :-
    is_free(S, State),
    is_range_free(Rest, State).

% -------------------------
% Operators
% -------------------------

% Enumerate concrete actions and apply when possible
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

% Operator schemas
can(move(Block, on(Target)),
    [ clear(Block), clear(Target), neq(Block,Target), stable(Block,Target), pos(Block,_), pos(Target,_) ]).

can(move(Block, table(X)),
    [ clear(Block), pos(Block,_), space_available(Block,X), slot(X) ]).

% Add and delete lists
adds(move(Block, Dest), [pos(Block, Dest), clear(Block)]).

deletes(move(Block, on(Target)), State, [pos(Block, OldPos), clear(Target)]) :-
    member(pos(Block, OldPos), State).
deletes(move(Block, table(_)), State, [pos(Block, OldPos)]) :-
    member(pos(Block, OldPos), State).

% -------------------------
% Preconditions evaluation
% -------------------------

holds(_, neq(A,B)) :- A \== B.
holds(_, stable(B1,B2)) :- size(B1,W1), size(B2,W2), W1 =< W2.
holds(State, space_available(Block,X)) :-
    integer(X),
    size(Block,W),
    Xend is X + W - 1,
    table_width(TW),
    X >= 0, Xend < TW,
    findall(S, between(X, Xend, S), Slots),
    is_range_free(Slots, State).
holds(State, pos(B,Arg)) :- member(pos(B,Arg), State).
holds(State, clear(B))   :- member(clear(B), State).
holds(_, slot(X))        :- slot(X).

possible(Action, State) :-
    can(Action, Preconditions),
    forall(member(P, Preconditions), holds(State, P)).

% -------------------------
% Apply actions (progression)
% -------------------------
apply(State, [], State).
apply(State, [Action|Rest], NewState) :-
    deletes(Action, State, Del),
    adds(Action, Add),
    ( member(pos(_, on(OldSupport)), Del) ->
        AddFinal = [clear(OldSupport)|Add]
    ;   AddFinal = Add
    ),
    subtract(State, Del, Tmp),
    append(Tmp, AddFinal, S),
    sort(S, Mid),
    apply(Mid, Rest, NewState).

% -------------------------
% Goal satisfaction
% -------------------------
satisfied(State, Goals) :-
    subset_list(Goals, State).

subset_list([], _).
subset_list([H|T], set) :- !, fail. % guard (should not hit)
subset_list([H|T], State) :- member(H, State), subset_list(T, State).

% -------------------------
% A* search
% -------------------------
% Node = [State, Plan, G, H, F]

a_star_plan(Initial, Goals, Plan) :-
    heuristic(Initial, Goals, H0),
    astar([[Initial, [], 0, H0, H0]], [], Goals, RevPlan),
    reverse(RevPlan, Plan).

astar([[State, Plan, _, _, _]|_], _, Goals, Plan) :-
    satisfied(State, Goals), !.
astar([[State, _, _, _, _]|Rest], Closed, Goals, Sol) :-
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

% Order open list by F (5th element)
sort_nodes(Nodes, Sorted) :-
    map_list_to_pairs(node_key, Nodes, Pairs),
    keysort(Pairs, SortedPairs),
    pairs_values(SortedPairs, Sorted).

node_key([_,_,_,_,F], F).

% Heuristic: count of unsatisfied goal atoms
heuristic(State, Goals, H) :-
    include(is_satisfied(State), Goals, SatList),
    length(Goals, Total),
    length(SatList, Satisfied),
    H is Total - Satisfied.

is_satisfied(State, Goal) :- member(Goal, State).

% -------------------------
% Scenarios (situations)
% -------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Situation 1
s1_start([
    pos(c,table(0)),
    pos(a,table(3)),
    pos(b,table(5)),
    pos(d,on(a)),
    pos(d,on(b)),
    clear(c),
    clear(d)
]).

s1_goal([
    pos(c,table(0)),
    pos(d,table(2)),
    pos(a,on(c)),
    pos(b,table(5)),
    clear(b),
    clear(d)
]).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Situation 2
s2_start([
    pos(c,table(0)),
    pos(a,on(c)),
    pos(b,on(c)),
    pos(d,table(3)),
    clear(a),
    clear(b),
    clear(d)
]).

s2_goal([
    pos(d,table(3)),
    pos(c,on(d)),
    pos(a,on(c)),
    pos(b,on(c)),
    clear(a),
    clear(b)
]).
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Situation 3
s3_start([
    pos(c,table(0)),
    pos(a,table(3)),
    pos(b,table(5)),
    pos(d,on(a)),
    pos(d,on(b)),
    clear(c),
    clear(d)
]).

s3_goal([
    pos(c,table(0)),
    pos(a,on(c)),
    pos(b,on(c)),
    pos(d,table(3)),
    clear(a),
    clear(b),
    clear(d)
]).
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% s1_start(S), s1_goal(G), a_star_plan(S,G,Plan).
% s2_start(S), s2_goal(G), a_star_plan(S,G,Plan).
% s3_start(S), s3_goal(G), a_star_plan(S,G,Plan).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Ajustes de meta e operadores
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Checagem de meta por subconjunto
satisfied(State, Goals) :-
    subset_state(Goals, State).

subset_state([], _).
subset_state([H|T], S) :-
    member(H, S),
    subset_state(T, S).

% Operador de ação
action(State, move(B, From, To), NewState) :-
    % B está na posição From e está livre
    member(pos(B, From), State),
    member(clear(B), State),

    % Condição do destino
    (   To = (table K)
    ->  \+ member(pos(_, (table K)), State)   % mesa vazia
    ;   To = on(Y),
        Y \= B,
        member(clear(Y), State)               % topo livre
    ),

    % Remover fatos antigos
    delete(State, pos(B, From), S1),
    (To = (table _) -> S2 = S1 ; delete(S1, clear(To), S2)),

    % Liberar origem se não for mesa
    (From = (table _) -> S3 = S2 ; S3 = [clear(From)|S2]),

    % Inserir novos fatos
    NewState = [pos(B, To), clear(B)|S3].


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Situações do trabalho
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Situação 1 — S0 -> Sf4
s1_start([pos(a,(table 0)), pos(b,(table 1)), pos(c,on(d)), pos(d,(table 2)),
          clear(a), clear(b), clear(c)]).
s1_goal([pos(a,on(c)), pos(c,(table 0)), pos(b,on(d)), pos(d,(table 1)),
         clear(a), clear(b)]).

% Situação 2 — S0 -> S5
s2_start([pos(a,(table 0)), pos(b,on(c)), pos(c,(table 1)), pos(d,(table 2)),
          clear(a), clear(b), clear(d)]).
s2_goal([pos(a,on(c)), pos(c,(table 0)), pos(b,on(d)), pos(d,(table 1)),
         clear(a), clear(b)]).

% Situação 3 — S0 -> S7
s3_start([pos(a,(table 0)), pos(b,(table 1)), pos(c,on(d)), pos(d,(table 2)),
          clear(a), clear(b), clear(c)]).
s3_goal([pos(a,on(c)), pos(c,(table 0)), pos(b,on(d)), pos(d,(table 1)),
         clear(a), clear(b)]).
