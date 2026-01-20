function [results] = FuzzyNavigation(mapChoice)
% Fuzzy Logic Based Robot Navigation
% mapChoice: 1 for Easy_Map, 2 for Hard_Map

if nargin < 1
    mapChoice = 1;
end

% Load map
if mapChoice == 1
    map = Easy_Map();
else
    map = Hard_Map();
end

% Initialize robot parameters
robot.pos = map.start;
robot.step_size = 0.3;
robot.sensor_range = 3.0;

% Create Fuzzy Inference System
fis = createFuzzySystem();

% Navigation parameters
max_steps = 1000;
goal_threshold = 0.5;
path = robot.pos;
collisions = 0;
steps = 0;

% Simulation
figure('Name', ['Fuzzy Navigation - ' map.name], 'Position', [100 100 800 600]);
hold on;
grid on;
axis equal;
xlim(map.xlim);
ylim(map.ylim);

% Draw obstacles
for i = 1:size(map.obstacles, 1)
    rectangle('Position', map.obstacles(i,:), 'FaceColor', [0.3 0.3 0.3]);
end

% Draw start and goal
plot(map.start(1), map.start(2), 'go', 'MarkerSize', 12, 'LineWidth', 2);
plot(map.goal(1), map.goal(2), 'r*', 'MarkerSize', 15, 'LineWidth', 2);

% Robot trajectory
h_robot = plot(robot.pos(1), robot.pos(2), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
h_path = plot(path(:,1), path(:,2), 'b-', 'LineWidth', 1.5);

reached_goal = false;

while steps < max_steps
    steps = steps + 1;
    
    % Calculate distances to obstacles
    [dist_left, dist_front, dist_right] = getSensorReadings(robot, map);
    
    % Calculate angle to goal
    dx = map.goal(1) - robot.pos(1);
    dy = map.goal(2) - robot.pos(2);
    angle_to_goal = atan2(dy, dx);
    current_heading = 0; % Assume initial heading is 0
    
    % Normalize angle difference to [-pi, pi]
    angle_diff = wrapToPi(angle_to_goal - current_heading);
    angle_diff_deg = rad2deg(angle_diff);
    
    % Distance to goal
    dist_to_goal = sqrt(dx^2 + dy^2);
    
    % Fuzzy inference
    [turn_angle, speed] = evaluateFuzzy(fis, dist_left, dist_front, dist_right, ...
                                         angle_diff_deg, dist_to_goal);
    
    % Update robot heading and position
    current_heading = current_heading + deg2rad(turn_angle);
    move_dist = robot.step_size * speed;
    
    new_pos = robot.pos + move_dist * [cos(angle_to_goal), sin(angle_to_goal)];
    
    % Check collision
    if checkCollision(new_pos, map)
        collisions = collisions + 1;
        % Try alternative direction
        new_pos = robot.pos + move_dist * [cos(angle_to_goal + deg2rad(turn_angle)), ...
                                            sin(angle_to_goal + deg2rad(turn_angle))];
        if checkCollision(new_pos, map)
            new_pos = robot.pos; % Stay in place
        end
    end
    
    robot.pos = new_pos;
    path = [path; robot.pos];
    
    % Update visualization
    set(h_robot, 'XData', robot.pos(1), 'YData', robot.pos(2));
    set(h_path, 'XData', path(:,1), 'YData', path(:,2));
    drawnow;
    
    % Check if goal reached
    if dist_to_goal < goal_threshold
        reached_goal = true;
        break;
    end
end

% Calculate performance metrics
results.success = reached_goal;
results.path_length = calculatePathLength(path);
results.smoothness = calculateSmoothness(path);
results.collisions = collisions;
results.steps = steps;
results.path = path;

% Display results
title(sprintf('%s\nSuccess: %d, Steps: %d, Length: %.2f, Smoothness: %.2f, Collisions: %d', ...
      map.name, results.success, results.steps, results.path_length, ...
      results.smoothness, results.collisions));
legend('Obstacles', 'Start', 'Goal', 'Robot', 'Path', 'Location', 'best');

fprintf('\n=== Fuzzy Navigation Results ===\n');
fprintf('Map: %s\n', map.name);
fprintf('Goal Reached: %d\n', results.success);
fprintf('Steps: %d\n', results.steps);
fprintf('Path Length: %.2f\n', results.path_length);
fprintf('Path Smoothness: %.2f\n', results.smoothness);
fprintf('Collisions: %d\n', results.collisions);
fprintf('================================\n\n');

end

function fis = createFuzzySystem()
% Create Fuzzy Inference System for navigation

fis = mamfis('Name', 'RobotNavigation');

% Input 1: Distance Left
fis = addInput(fis, [0 5], 'Name', 'DistLeft');
fis = addMF(fis, 'DistLeft', 'trapmf', [0 0 0.5 1.5], 'Name', 'Near');
fis = addMF(fis, 'DistLeft', 'trimf', [1 2 3], 'Name', 'Medium');
fis = addMF(fis, 'DistLeft', 'trapmf', [2.5 3.5 5 5], 'Name', 'Far');

% Input 2: Distance Front
fis = addInput(fis, [0 5], 'Name', 'DistFront');
fis = addMF(fis, 'DistFront', 'trapmf', [0 0 0.5 1.5], 'Name', 'Near');
fis = addMF(fis, 'DistFront', 'trimf', [1 2 3], 'Name', 'Medium');
fis = addMF(fis, 'DistFront', 'trapmf', [2.5 3.5 5 5], 'Name', 'Far');

% Input 3: Distance Right
fis = addInput(fis, [0 5], 'Name', 'DistRight');
fis = addMF(fis, 'DistRight', 'trapmf', [0 0 0.5 1.5], 'Name', 'Near');
fis = addMF(fis, 'DistRight', 'trimf', [1 2 3], 'Name', 'Medium');
fis = addMF(fis, 'DistRight', 'trapmf', [2.5 3.5 5 5], 'Name', 'Far');

% Input 4: Angle to Goal
fis = addInput(fis, [-180 180], 'Name', 'AngleToGoal');
fis = addMF(fis, 'AngleToGoal', 'trapmf', [-180 -180 -60 -20], 'Name', 'Left');
fis = addMF(fis, 'AngleToGoal', 'trimf', [-30 0 30], 'Name', 'Straight');
fis = addMF(fis, 'AngleToGoal', 'trapmf', [20 60 180 180], 'Name', 'Right');

% Input 5: Distance to Goal
fis = addInput(fis, [0 30], 'Name', 'DistGoal');
fis = addMF(fis, 'DistGoal', 'trapmf', [0 0 1 3], 'Name', 'Near');
fis = addMF(fis, 'DistGoal', 'trimf', [2 8 15], 'Name', 'Medium');
fis = addMF(fis, 'DistGoal', 'trapmf', [12 20 30 30], 'Name', 'Far');

% Output 1: Turn Angle
fis = addOutput(fis, [-90 90], 'Name', 'TurnAngle');
fis = addMF(fis, 'TurnAngle', 'trimf', [-90 -60 -30], 'Name', 'SharpLeft');
fis = addMF(fis, 'TurnAngle', 'trimf', [-45 -20 0], 'Name', 'Left');
fis = addMF(fis, 'TurnAngle', 'trimf', [-10 0 10], 'Name', 'Straight');
fis = addMF(fis, 'TurnAngle', 'trimf', [0 20 45], 'Name', 'Right');
fis = addMF(fis, 'TurnAngle', 'trimf', [30 60 90], 'Name', 'SharpRight');

% Output 2: Speed
fis = addOutput(fis, [0 1], 'Name', 'Speed');
fis = addMF(fis, 'Speed', 'trimf', [0 0 0.3], 'Name', 'Slow');
fis = addMF(fis, 'Speed', 'trimf', [0.2 0.5 0.8], 'Name', 'Medium');
fis = addMF(fis, 'Speed', 'trimf', [0.7 1 1], 'Name', 'Fast');

% Rules
rules = [
    % Obstacle avoidance rules
    "DistFront==Near => TurnAngle=SharpLeft Speed=Slow"
    "DistFront==Near & DistLeft==Far => TurnAngle=Left Speed=Slow"
    "DistFront==Near & DistRight==Far => TurnAngle=Right Speed=Slow"
    
    % Goal seeking rules
    "DistFront==Far & AngleToGoal==Straight => TurnAngle=Straight Speed=Fast"
    "DistFront==Far & AngleToGoal==Left => TurnAngle=Left Speed=Medium"
    "DistFront==Far & AngleToGoal==Right => TurnAngle=Right Speed=Medium"
    
    % Medium distance obstacle handling
    "DistFront==Medium & AngleToGoal==Straight => TurnAngle=Straight Speed=Medium"
    "DistFront==Medium & DistLeft==Far => TurnAngle=Left Speed=Medium"
    "DistFront==Medium & DistRight==Far => TurnAngle=Right Speed=Medium"
    
    % Near goal behavior
    "DistGoal==Near => Speed=Slow"
    "DistGoal==Far & DistFront==Far => Speed=Fast"
];

fis = addRule(fis, rules);

end

function [dist_left, dist_front, dist_right] = getSensorReadings(robot, map)
% Simulate sensor readings

sensor_angles = [-pi/2, 0, pi/2]; % Left, Front, Right
distances = [5, 5, 5]; % Default max range

for i = 1:3
    angle = sensor_angles(i);
    min_dist = 5;
    
    for obs_idx = 1:size(map.obstacles, 1)
        obs = map.obstacles(obs_idx, :);
        
        % Check intersection with obstacle
        for d = 0:0.1:5
            check_point = robot.pos + d * [cos(angle), sin(angle)];
            
            if check_point(1) >= obs(1) && check_point(1) <= obs(1) + obs(3) && ...
               check_point(2) >= obs(2) && check_point(2) <= obs(2) + obs(4)
                min_dist = min(min_dist, d);
                break;
            end
        end
    end
    
    distances(i) = min_dist;
end

dist_left = distances(1);
dist_front = distances(2);
dist_right = distances(3);

end

function [turn_angle, speed] = evaluateFuzzy(fis, dist_left, dist_front, dist_right, angle_diff, dist_goal)
% Evaluate fuzzy system

output = evalfis(fis, [dist_left, dist_front, dist_right, angle_diff, dist_goal]);
turn_angle = output(1);
speed = output(2);

end

function collision = checkCollision(pos, map)
% Check if position collides with obstacles

collision = false;

% Check boundaries
if pos(1) < map.xlim(1) || pos(1) > map.xlim(2) || ...
   pos(2) < map.ylim(1) || pos(2) > map.ylim(2)
    collision = true;
    return;
end

% Check obstacles
for i = 1:size(map.obstacles, 1)
    obs = map.obstacles(i, :);
    if pos(1) >= obs(1) && pos(1) <= obs(1) + obs(3) && ...
       pos(2) >= obs(2) && pos(2) <= obs(2) + obs(4)
        collision = true;
        return;
    end
end

end

function length = calculatePathLength(path)
% Calculate total path length

length = 0;
for i = 2:size(path, 1)
    length = length + norm(path(i,:) - path(i-1,:));
end

end

function smoothness = calculateSmoothness(path)
% Calculate path smoothness (lower is smoother)
% Based on sum of absolute angular changes

if size(path, 1) < 3
    smoothness = 0;
    return;
end

total_angle_change = 0;

for i = 2:size(path, 1)-1
    v1 = path(i,:) - path(i-1,:);
    v2 = path(i+1,:) - path(i,:);
    
    if norm(v1) > 0 && norm(v2) > 0
        angle = acos(dot(v1, v2) / (norm(v1) * norm(v2)));
        total_angle_change = total_angle_change + abs(angle);
    end
end

smoothness = total_angle_change;

end