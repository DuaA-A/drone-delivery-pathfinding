grid_size(5, 5).

% --- Obstacles ---
obstacle(1, 5).
obstacle(2, 2).
obstacle(3, 3).
obstacle(4, 2).
obstacle(5, 4).

% --- Delivery Points ---
delivery_point(1, 3).
delivery_point(2, 5).
delivery_point(3, 4).
delivery_point(4, 1).
delivery_point(5, 3).

% --- Start Point ---
start_point((1, 1)).


% Move right
move((X, Y), (NewX, Y)) :-
    NewX is X + 1,
    grid_size(MaxX, _),
    NewX =< MaxX,
    \+ obstacle(NewX, Y).

% Move left
move((X, Y), (NewX, Y)) :-
    NewX is X - 1,
    NewX > 0,
    \+ obstacle(NewX, Y).

% Move down
move((X, Y), (X, NewY)) :-
    NewY is Y + 1,
    grid_size(_, MaxY),
    NewY =< MaxY,
    \+ obstacle(X, NewY).

% Move up
move((X, Y), (X, NewY)) :-
    NewY is Y - 1,
    NewY > 0,
    \+ obstacle(X, NewY).


%%%%%%%%%%%%%%%%%BFS Search%%%%%%%%%%%%%%%%%%%

% Base case: Queue is empty, return accumulated paths
bfs([], _, AllPaths, AllPaths).

% recursive case: track closed list, and collect stuck paths
bfs([(CurrentPos, PathSoFar)|RestQueue], Closed, AccumulatedPaths, AllPaths) :-
    \+ member(CurrentPos, Closed), % dont check the position is already in the closed list(explored)
    findall( %generates all possible next positions
        (NextPos, [NextPos|PathSoFar]),
        (
            move(CurrentPos, NextPos),
            \+ member(NextPos, PathSoFar), % Prevent revisiting cells in the same path
            \+ member(NextPos, RestQueue), % NextPos is not already in the queue (open list)
            \+ member(NextPos, Closed) %NextPos is not in the Closed list (explored)
        ),
        Children
    ),
    append(RestQueue, Children, NewQueue),  %put childrens on the right of the open list (NewQueue)
    update_accumulated_paths(PathSoFar, AccumulatedPaths, NewAccumulatedPaths),%add the current path to the accumlated path or not
    bfs(NewQueue, [CurrentPos|Closed], NewAccumulatedPaths, AllPaths).

% Update accumulated paths based on stuck condition
update_accumulated_paths(PathSoFar, AccumulatedPaths, [PathSoFar|AccumulatedPaths]) :-
    is_stuck(PathSoFar).

update_accumulated_paths(PathSoFar, AccumulatedPaths, AccumulatedPaths) :-
    \+ is_stuck(PathSoFar).


% Check If drone is stuck (no further moves possible)
is_stuck(Path) :-
    last(Path, LastPos),
    \+ (
        move(LastPos, NextPos),
        \+ member(NextPos, Path)
    ).
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Path Processing and Evaluation
% Prepare paths: Reverse and count delivery points
prepare_paths([], []).

prepare_paths([Path|Rest], [(RealPath, Points)|PreparedRest]) :-
    reverse(Path, RealPath),
    count_delivery_points(RealPath, 0, Points),
    prepare_paths(Rest, PreparedRest).

% Count delivery points in a path
count_delivery_points([], Acc, Acc).

% Case 1: Current position is a delivery point
count_delivery_points([(X, Y)|Rest], Acc, TotalPoints) :-
    delivery_point(X, Y),
    NewAcc is Acc + 1,
    count_delivery_points(Rest, NewAcc, TotalPoints).

% Case 2: Current position is not a delivery point
count_delivery_points([(X, Y)|Rest], Acc, TotalPoints) :-
    \+ delivery_point(X, Y),
    count_delivery_points(Rest, Acc, TotalPoints).

% Find paths with maximum delivery points among stuck paths
find_max_delivery_paths([], []). % Base case: No paths, return empty list

find_max_delivery_paths(Paths, MaxPaths) :-
    include(is_stuck_with_path, Paths, StuckPaths),
    findall(Points, member((_, Points), StuckPaths), PointsList),
    max_list(PointsList, MaxPoints),
    findall((Path, MaxPoints), member((Path, MaxPoints), StuckPaths), MaxPaths).

% Helper for include: Check if path is stuck
is_stuck_with_path((Path, _)) :-
    is_stuck(Path).

% Path Printing
print_max_paths([]).

print_max_paths([(Path, Points)|Rest]) :-
    write('--- Best Path Found ---'),
    nl,
    write('Path:'),
    nl,
    print_path(Path),
    write('Delivery points collected: '),
    write(Points),
    nl,
    nl,
    print_max_paths(Rest).

print_path([]).

print_path([(X, Y)|Rest]) :-
    write(X),
    write(','),
    write(Y),
    nl,
    print_path(Rest).

start :-
    start_point(Start),
    bfs([(Start, [Start])], [], [], AllPaths),
    prepare_paths(AllPaths, PreparedPaths),
    find_max_delivery_paths(PreparedPaths, MaxPaths),
    print_max_paths(MaxPaths).















