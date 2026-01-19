function map = Easy_Map()

% Map boundary
map.xlim = [0 20];
map.ylim = [0 20];

% Start and goal
map.start = [2 2];
map.goal  = [18 18];

% Obstacles [x y width height]
map.obstacles = [
    6   6   3   3;
    11  10  3   3
];

map.name = 'Easy Map';

end
function [outputArg1,outputArg2] = untitled(inputArg1,inputArg2)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
outputArg1 = inputArg1;
outputArg2 = inputArg2;
end

