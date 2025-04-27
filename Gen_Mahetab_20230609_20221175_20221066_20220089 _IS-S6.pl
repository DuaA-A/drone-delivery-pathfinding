%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Problem 1 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Dynamic declarations
:- dynamic(grid_size/2).
:- dynamic(obstacle/2).
:- dynamic(delivery_point/2).
:- dynamic(start_point/1).

%%%%%%%%%%%%%%%%%%%%%% Move Definition %%%%%%%%%%%%%%%%%%%%%%

move((X, Y), (NewX, Y)) :-
    NewX is X + 1,
    grid_size(MaxX, _),
    NewX =< MaxX,
    \+ obstacle(NewX, Y).

move((X, Y), (NewX, Y)) :-
    NewX is X - 1,
    NewX > 0,
    \+ obstacle(NewX, Y).

move((X, Y), (X, NewY)) :-
    NewY is Y + 1,
    grid_size(_, MaxY),
    NewY =< MaxY,
    \+ obstacle(X, NewY).

move((X, Y), (X, NewY)) :-
    NewY is Y - 1,
    NewY > 0,
    \+ obstacle(X, NewY).

%%%%%%%%%%%%%%%%%%%%%% DFS Search %%%%%%%%%%%%%%%%%%%%%%

is_delivery_point(R, C) :-
    delivery_point(R, C),
    !.
is_delivery_point(_, _) :-
    false.

dfs((R, C), Visited, DeliveryCount, PathSoFar, BestDeliveryCount, BestPath) :-
    \+ member((R, C), Visited),
    is_delivery_point(R, C),
    NewDeliveryCount is DeliveryCount + 1,
    append(Visited, [(R, C)], NewVisited),
    append(PathSoFar, [(R, C)], NewPath),
    findall((NewR, NewC),
            (move((R, C), (NewR, NewC)),
                \+ member((NewR, NewC), NewVisited)),
            Neighbors),
    dfs_neighbors(Neighbors, NewVisited, NewDeliveryCount, NewPath, BestDeliveryCount, BestPath).

dfs((R, C), Visited, DeliveryCount, PathSoFar, BestDeliveryCount, BestPath) :-
    \+ member((R, C), Visited),
    \+ is_delivery_point(R, C),
    NewDeliveryCount is DeliveryCount,
    append(Visited, [(R, C)], NewVisited),
    append(PathSoFar, [(R, C)], NewPath),
    findall((NewR, NewC),
            (move((R, C), (NewR, NewC)),
                \+ member((NewR, NewC), NewVisited)),
            Neighbors),
    dfs_neighbors(Neighbors, NewVisited, NewDeliveryCount, NewPath, BestDeliveryCount, BestPath).

dfs(_, _, DeliveryCount, PathSoFar, DeliveryCount, PathSoFar).

dfs_neighbors([], _, DeliveryCount, PathSoFar, DeliveryCount, PathSoFar).

dfs_neighbors([(NewR, NewC)|Rest], Visited, DeliveryCount, PathSoFar, BestDeliveryCount, BestPath) :-
    dfs((NewR, NewC), Visited, DeliveryCount, PathSoFar, SubDeliveryCount, SubPath),
    dfs_neighbors(Rest, Visited, DeliveryCount, PathSoFar, RestDeliveryCount, RestPath),
    select_best(SubDeliveryCount, SubPath, RestDeliveryCount, RestPath, BestDeliveryCount, BestPath).

select_best(SubDeliveryCount, SubPath, RestDeliveryCount, RestPath, BestDeliveryCount, BestPath) :-
    SubDeliveryCount >= RestDeliveryCount,
    BestDeliveryCount = SubDeliveryCount,
    BestPath = SubPath.
select_best(SubDeliveryCount, SubPath, RestDeliveryCount, RestPath, BestDeliveryCount, BestPath) :-
    SubDeliveryCount < RestDeliveryCount,
    BestDeliveryCount = RestDeliveryCount,
    BestPath = RestPath.

%%%%%%%%%%%%%%%%%%%%%% Solve Function %%%%%%%%%%%%%%%%%%%%%%

solve :-
    retractall(grid_size(_, _)),
    retractall(obstacle(_, _)),
    retractall(delivery_point(_, _)),
    retractall(start_point(_)),

    write('Enter grid width: '), read(W),
    write('Enter grid height: '), read(H),
    assert(grid_size(W, H)),

    write('Enter start point as (X,Y): '), read((SX, SY)),
    assert(start_point((SX, SY))),

    write('Enter obstacles list as [(X1,Y1),(X2,Y2),...]: '), read(Obstacles),
    assert_obstacles(Obstacles),

    write('Enter delivery points list as [(X1,Y1),(X2,Y2),...]: '), read(DeliveryPoints),
    assert_delivery_points(DeliveryPoints),

    start_point(Start),
    dfs(Start, [], 0, [], MaxDeliveryCount, BestPath),

    nl, write('Drone Route:'), nl,
    write('Maximum delivery points visited: '), write(MaxDeliveryCount), nl,
    write('Path taken: '), write(BestPath), nl,
    print_grid(Start, []),

    write('Steps:'), nl,
    simulate_path(BestPath, []).

%%%%%%%%%%%%%%%%%%%%%% Printing Utilities %%%%%%%%%%%%%%%%%%%%%%

simulate_path([], _).
simulate_path([Pos], PathSoFar) :-
    append(PathSoFar, [Pos], FinalPath),
    nl, write('Final:'), nl,
    print_grid(Pos, FinalPath),
    nl.

simulate_path([Pos|Rest], PathSoFar) :-
    append(PathSoFar, [Pos], NewPath),
    print_grid(Pos, NewPath),
    nl,
    simulate_path(Rest, NewPath).

print_grid(CurrentPos, Path) :-
    grid_size(MaxX, MaxY),
    forall(between(1, MaxX, X0),
        (
            X is X0,
            forall(between(1, MaxY, Y),
                print_cell(X, Y, CurrentPos, Path)
            ),
            nl
        )
    ).

print_cell(X, Y, CurrentPos, _) :-
    (X, Y) = CurrentPos,
    !,
    write(' d ').

print_cell(X, Y, _, Path) :-
    member((X, Y), Path),
    !,
    write(' * ').

print_cell(X, Y, _, _) :-
    obstacle(X, Y),
    !,
    write(' O ').

print_cell(X, Y, _, _) :-
    delivery_point(X, Y),
    !,
    write(' P ').

print_cell(_, _, _, _) :-
    write(' - ').

%%%%%%%%%%%%%%%%%%%%%% Utility Predicates %%%%%%%%%%%%%%%%%%%%%%

assert_obstacles([]).
assert_obstacles([(X,Y)|Rest]) :-
    assert(obstacle(X,Y)),
    assert_obstacles(Rest).

assert_delivery_points([]).
assert_delivery_points([(X,Y)|Rest]) :-
    assert(delivery_point(X,Y)),
    assert_delivery_points(Rest).

select_best_path([], ([], [])).
select_best_path([(Path, Moves)|Rest], Best) :-
    count_delivery_points(Path, Points),
    select_best_path(Rest, (BestPath, BestMoves)),
    count_delivery_points(BestPath, BestPoints),
    compare_paths(Points, Path, Moves, BestPoints, BestPath, BestMoves, Best).

compare_paths(Points, Path, Moves, BestPoints, _, _, (Path, Moves)) :-
    Points > BestPoints,
    !.
compare_paths(_, _, _, _, BestPath, BestMoves, (BestPath, BestMoves)).

count_delivery_points([], 0).
count_delivery_points([(X,Y)|Rest], Count) :-
    count_delivery_points(Rest, RestCount),
    delivery_point_check(X, Y, RestCount, Count).

delivery_point_check(X, Y, RestCount, Count) :-
    delivery_point(X, Y),
    !,
    Count is RestCount + 1.
delivery_point_check(_, _, RestCount, RestCount).

subset([], _).
subset([H|T], List) :-
    member(H, List),
    subset(T, List).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Problem 2%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% ----------------- Grid Definition -----------------
grid([
    [d, e, e, p],
    [e, o, e, e],
    [e, r, o, p],
    [e, e, e, e]
]).

% ----------------- Grid Helpers -----------------
start((X,Y)) :-
    grid(Grid),
    nth0(X, Grid, Row),
    nth0(Y, Row, d).

is_delivery_point((X,Y)) :-
    grid(Grid),
    nth0(X, Grid, Row),
    nth0(Y, Row, p).

is_obstacle((X,Y)) :-
    grid(Grid),
    nth0(X, Grid, Row),
    nth0(Y, Row, o).

is_recharge((X,Y)) :-
    grid(Grid),
    nth0(X, Grid, Row),
    nth0(Y, Row, r).

grid_size(W, H) :-
    grid(Grid),
    length(Grid, W),
    Grid = [Row|_],
    length(Row, H).

% ----------------- Movement and Validation -----------------
move((X,Y), (NX,Y)) :- NX is X + 1.
move((X,Y), (NX,Y)) :- NX is X - 1.
move((X,Y), (X,NY)) :- NY is Y + 1.
move((X,Y), (X,NY)) :- NY is Y - 1.

valid((X,Y)) :-
    grid_size(W, H),
    X >= 0, Y >= 0,
    X < W, Y < H,
    \+ is_obstacle((X,Y)).

% ----------------- Heuristic -----------------
heuristic((X1,Y1), (X2,Y2), H) :-
    H is abs(X1 - X2) + abs(Y1 - Y2).

estimate_heuristic(_, [], 0).
estimate_heuristic(Pos, Goals, H) :-
    findall(D, (member(G, Goals), heuristic(Pos, G, D)), Distances),
    min_list(Distances, H).

all_delivery_points(Points) :-
    findall((X,Y), is_delivery_point((X,Y)), Points).

full_energy(6).  % Default energy capacity

% ----------------- A* Search -----------------
astar :-
    start(Start),
    all_delivery_points(Goals),
    full_energy(Energy),
    astar_search([(0, Start, [], Goals, Energy)], []).

astar_search([(_, Pos, Path, [], Energy)|_], _) :-
    reverse([(Pos, Energy)|Path], FinalPath),
    write('Drone Path:'), nl,
    print_steps(FinalPath),
    nl, write('Final Grid:'), nl,
    extract_positions(FinalPath, PosList),
    draw_grid(PosList).

astar_search([(_, Pos, Path, Goals, Energy)|Rest], Visited) :-
    findall(
        (F, Next, [(Pos, Energy)|Path], NewGoals, NewEnergy),
        (
            move(Pos, Next),
            valid(Next),
            \+ member((Next, _), [(Pos, Energy)|Path]),
            Energy > 0,
            (
                member(Next, Goals) -> delete(Goals, Next, NewGoals)
                ; NewGoals = Goals
            ),
            (
                is_recharge(Next) -> full_energy(Full), NewEnergy = Full
                ; NewEnergy is Energy - 1
            ),
            length([(Pos, Energy)|Path], G),
            estimate_heuristic(Next, NewGoals, H),
            F is G + H
        ),
        Children
    ),
    append(Rest, Children, NewOpen),
    sort(NewOpen, SortedOpen),
    astar_search(SortedOpen, [(Pos, Energy)|Visited]).

% ----------------- Step-by-Step Output -----------------
print_steps([]).
print_steps([(Pos, Energy)|Rest]) :-
    print_step_info(1, (Pos, Energy), Rest).

print_step_info(_, _, []).
print_step_info(N, (Pos, Energy), [(NextPos, NextEnergy)|Rest]) :-
    (
        is_recharge(NextPos) ->
            format('Step ~w: ~w, Energy: ~w~n', [N, Pos, Energy]),
            N1 is N + 1,
            format('Step ~w: ~w (Recharge), Energy: ~w -> ~w~n', [N1, NextPos, Energy - 1, NextEnergy])
        ;
            format('Step ~w: ~w, Energy: ~w~n', [N, Pos, Energy]),
            N1 is N + 1
    ),
    print_step_info(N1, (NextPos, NextEnergy), Rest).

extract_positions([], []).
extract_positions([(P,_)|Rest], [P|RestP]) :- extract_positions(Rest, RestP).

% ----------------- Grid Drawing -----------------
draw_grid(Path) :-
    grid(Grid),
    draw_rows(Grid, 0, Path).

draw_rows([], _, _).
draw_rows([Row|Rest], X, Path) :-
    draw_row(Row, X, 0, Path), nl,
    X1 is X + 1,
    draw_rows(Rest, X1, Path).

draw_row([], _, _, _).
draw_row([Cell|Rest], X, Y, Path) :-
    (member((X,Y), Path) -> write(' * ')
    ; Cell = d -> write(' D ')
    ; Cell = p -> write(' P ')
    ; Cell = o -> write(' O ')
    ; Cell = r -> write(' R ')
    ; write(' . ')),
    Y1 is Y + 1,
    draw_row(Rest, X, Y1, Path).
