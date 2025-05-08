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

% Check if a position is a delivery point
is_delivery_point(R, C) :-
    delivery_point(R, C),
    !.
is_delivery_point(_, _) :-
    false.

% DFS to explore the grid, tracking the path
dfs((R, C), Visited, DeliveryCount, PathSoFar, BestDeliveryCount, BestPath) :-
    % Mark current position as visited
    \+ member((R, C), Visited),
    % Increment delivery count if at a delivery point
    is_delivery_point(R, C),
    NewDeliveryCount is DeliveryCount + 1,
    % Add current position to visited list and path
    append(Visited, [(R, C)], NewVisited),
    append(PathSoFar, [(R, C)], NewPath),
    % Explore neighbors
    findall((NewR, NewC),
            (move((R, C), (NewR, NewC)),
                \+ member((NewR, NewC), NewVisited)),
            Neighbors),
    dfs_neighbors(Neighbors, NewVisited, NewDeliveryCount, NewPath, BestDeliveryCount, BestPath).

% Case when current position is not a delivery point
dfs((R, C), Visited, DeliveryCount, PathSoFar, BestDeliveryCount, BestPath) :-
    % Mark current position as visited
    \+ member((R, C), Visited),
    \+ is_delivery_point(R, C),
    NewDeliveryCount is DeliveryCount,
    % Add current position to visited list and path
    append(Visited, [(R, C)], NewVisited),
    append(PathSoFar, [(R, C)], NewPath),
    % Explore neighbors
    findall((NewR, NewC),
            (move((R, C), (NewR, NewC)),
                \+ member((NewR, NewC), NewVisited)),
            Neighbors),
    dfs_neighbors(Neighbors, NewVisited, NewDeliveryCount, NewPath, BestDeliveryCount, BestPath).

% Base case:  if no valid move can be made anymore  (stuck)
dfs(_, _, DeliveryCount, PathSoFar, DeliveryCount, PathSoFar).

% Helper to explore all neighbors
%base case : if neighbors list is empty
dfs_neighbors([], _, DeliveryCount, PathSoFar, DeliveryCount, PathSoFar).

dfs_neighbors([(NewR, NewC)|Rest], Visited, DeliveryCount, PathSoFar, BestDeliveryCount, BestPath) :-
    dfs((NewR, NewC), Visited, DeliveryCount, PathSoFar, SubDeliveryCount, SubPath),
    dfs_neighbors(Rest, Visited, DeliveryCount, PathSoFar, RestDeliveryCount, RestPath),
    select_best(SubDeliveryCount, SubPath, RestDeliveryCount, RestPath, BestDeliveryCount, BestPath).

% Helper to select the best path based on delivery count
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

% Helper predicates for print_cell ;
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

# % Refactored select_best_path
# select_best_path([], ([], [])).
# select_best_path([(Path, Moves)|Rest], Best) :-
#     count_delivery_points(Path, Points),
#     select_best_path(Rest, (BestPath, BestMoves)),
#     count_delivery_points(BestPath, BestPoints),
#     compare_paths(Points, Path, Moves, BestPoints, BestPath, BestMoves, Best).

# % Helper to compare paths
# compare_paths(Points, Path, Moves, BestPoints, _, _, (Path, Moves)) :-
#     Points > BestPoints,
#     !.
# compare_paths(_, _, _, _, BestPath, BestMoves, (BestPath, BestMoves)).

# % count_delivery_points
# count_delivery_points([], 0).
# count_delivery_points([(X,Y)|Rest], Count) :-
#     count_delivery_points(Rest, RestCount),
#     delivery_point_check(X, Y, RestCount, Count).

# % Helper to check delivery point
# delivery_point_check(X, Y, RestCount, Count) :-
#     delivery_point(X, Y),
#     !,
#     Count is RestCount + 1.
# delivery_point_check(_, _, RestCount, RestCount).

# subset([], _).
# subset([H|T], List) :-
#     member(H, List),
#     subset(T, List).

















