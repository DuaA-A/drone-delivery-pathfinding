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
