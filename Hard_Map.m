function map = Hard_Map()
% HARD_MAP
% Complex environment with narrow corridors
% Designed to test intelligent navigation

% Map boundary
map.xlim = [0 20];
map.ylim = [0 20];

% Start and goal positions
map.start = [2 10];
map.goal  = [18 10];

% Obstacles [x y width height]
map.obstacles = [
    % Left block
    5   0   1   7;
    5   13  1   7;

    % Middle corridor walls
    9   4   1   12;
    12  0   1   7;
    12  13  1   7;

    % Right block
    15  4   1   12
];

map.name = 'Hard Map';

end
