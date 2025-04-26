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

dfs(Path, Path, _, []) :-
    path_completed(Path),
    !.

dfs(CurrentPath, FinalPath, Visited, [Move|Moves]) :-
    CurrentPath = [CurrentPos|_],
    move(CurrentPos, NextPos),
    \+ member(NextPos, Visited),
    dfs([NextPos|CurrentPath], FinalPath, [NextPos|Visited], Moves).

path_completed(Path) :-
    findall((X,Y), delivery_point(X,Y), Deliveries),
    subset(Deliveries, Path).

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
    findall((P, Moves),
        dfs([Start], P, [Start], Moves),
        AllPaths),
    select_best_path(AllPaths, (BestPath, _)),
    reverse(BestPath, RealPath),

    nl, write('Drone Route:'), nl,
    print_grid(Start, []),

    write('Steps:'), nl,
    simulate_path(RealPath, []).


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
    forall(between(1, MaxX, X0),   % Iterate over X (columns)
        (
            X is X0,
            forall(between(1, MaxY, Y),   % Iterate over Y (rows)
                (
                    ( ( (X,Y) = CurrentPos -> write(' d ')  
                    ; member((X,Y), Path) -> write(' * ')  
                    ; obstacle(X,Y)        -> write(' O ')      
                    ; delivery_point(X,Y)  -> write(' P ')    
                    ; write(' - ')                         
                    )
                    )
                )
            ),
            nl
        )
    ).


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
    ( Points > BestPoints -> Best = (Path, Moves) ; Best = (BestPath, BestMoves) ).

count_delivery_points([], 0).
count_delivery_points([(X,Y)|Rest], Count) :-
    count_delivery_points(Rest, RestCount),
    (delivery_point(X,Y) -> Count is RestCount + 1 ; Count is RestCount).

subset([], _).
subset([H|T], List) :-
    member(H, List),
    subset(T, List).
