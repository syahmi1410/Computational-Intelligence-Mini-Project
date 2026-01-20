% Fuzzy Logic System Visualization
% Shows membership functions and rules
clear all; close all; clc;

fprintf('==========================================\n');
fprintf('FUZZY LOGIC SYSTEM VISUALIZATION\n');
fprintf('==========================================\n\n');

%% Create Fuzzy Inference System
fis = create_fuzzy_navigation_system();

%% Display System Information
fprintf('Fuzzy System: %s\n', fis.Name);
fprintf('Number of Inputs: 2\n');
fprintf('Number of Outputs: 1\n');
fprintf('Number of Rules: 9\n\n');

%% Display Fuzzy Rules
fprintf('FUZZY RULES:\n');
fprintf('============\n\n');

fprintf('Rule 1: IF GoalDistance is Near AND ObstacleDistance is Safe\n');
fprintf('        THEN MovementUrgency is Aggressive\n\n');

fprintf('Rule 2: IF GoalDistance is Near AND ObstacleDistance is Close\n');
fprintf('        THEN MovementUrgency is Normal\n\n');

fprintf('Rule 3: IF GoalDistance is Near AND ObstacleDistance is VeryClose\n');
fprintf('        THEN MovementUrgency is Careful\n\n');

fprintf('Rule 4: IF GoalDistance is Medium AND ObstacleDistance is Safe\n');
fprintf('        THEN MovementUrgency is Normal\n\n');

fprintf('Rule 5: IF GoalDistance is Medium AND ObstacleDistance is Close\n');
fprintf('        THEN MovementUrgency is Careful\n\n');

fprintf('Rule 6: IF GoalDistance is Medium AND ObstacleDistance is VeryClose\n');
fprintf('        THEN MovementUrgency is Careful\n\n');

fprintf('Rule 7: IF GoalDistance is Far AND ObstacleDistance is Safe\n');
fprintf('        THEN MovementUrgency is Normal\n\n');

fprintf('Rule 8: IF GoalDistance is Far AND ObstacleDistance is Close\n');
fprintf('        THEN MovementUrgency is Careful\n\n');

fprintf('Rule 9: IF GoalDistance is Far AND ObstacleDistance is VeryClose\n');
fprintf('        THEN MovementUrgency is Careful\n\n');

%% Visualize Membership Functions
visualize_membership_functions();

%% Test with Sample Values
fprintf('\n==========================================\n');
fprintf('TESTING FUZZY SYSTEM\n');
fprintf('==========================================\n\n');

% Test Case 1: Goal far, obstacle safe
input1 = [25, 8];
output1 = evalfis(fis, input1);
fprintf('Test 1:\n');
fprintf('  Goal Distance = %.1f (FAR)\n', input1(1));
fprintf('  Obstacle Distance = %.1f (SAFE)\n', input1(2));
fprintf('  → Movement Urgency = %.2f (NORMAL)\n\n', output1);

% Test Case 2: Goal near, obstacle very close
input2 = [5, 1.5];
output2 = evalfis(fis, input2);
fprintf('Test 2:\n');
fprintf('  Goal Distance = %.1f (NEAR)\n', input2(1));
fprintf('  Obstacle Distance = %.1f (VERY CLOSE)\n', input2(2));
fprintf('  → Movement Urgency = %.2f (CAREFUL)\n\n', output2);

% Test Case 3: Goal near, obstacle safe
input3 = [8, 9];
output3 = evalfis(fis, input3);
fprintf('Test 3:\n');
fprintf('  Goal Distance = %.1f (NEAR)\n', input3(1));
fprintf('  Obstacle Distance = %.1f (SAFE)\n', input3(2));
fprintf('  → Movement Urgency = %.2f (AGGRESSIVE)\n\n', output3);

% Test Case 4: Goal medium, obstacle close
input4 = [15, 3];
output4 = evalfis(fis, input4);
fprintf('Test 4:\n');
fprintf('  Goal Distance = %.1f (MEDIUM)\n', input4(1));
fprintf('  Obstacle Distance = %.1f (CLOSE)\n', input4(2));
fprintf('  → Movement Urgency = %.2f (CAREFUL)\n\n', output4);

%% 3D Surface View
fprintf('Generating 3D Control Surface...\n');
generate_3d_surface(fis);

%% Show Rule Viewer (Interactive)
fprintf('\nOpening Interactive Rule Viewer...\n');
fprintf('You can drag the input sliders to see outputs change!\n\n');
ruleview(fis);

fprintf('==========================================\n');
fprintf('Visualization Complete!\n');
fprintf('==========================================\n');

%% ========== FUNCTIONS ==========

function fis = create_fuzzy_navigation_system()
    % Create Fuzzy Inference System with 2 inputs, 1 output
    fis = mamfis('Name', 'RobotNavigationFIS');
    
    % INPUT 1: Distance to Goal (0-30)
    fis = addInput(fis, [0 30], 'Name', 'GoalDistance');
    fis = addMF(fis, 'GoalDistance', 'trimf', [0 0 10], 'Name', 'Near');
    fis = addMF(fis, 'GoalDistance', 'trimf', [5 15 25], 'Name', 'Medium');
    fis = addMF(fis, 'GoalDistance', 'trimf', [20 30 30], 'Name', 'Far');
    
    % INPUT 2: Obstacle Distance (0-10)
    fis = addInput(fis, [0 10], 'Name', 'ObstacleDistance');
    fis = addMF(fis, 'ObstacleDistance', 'trimf', [0 0 2], 'Name', 'VeryClose');
    fis = addMF(fis, 'ObstacleDistance', 'trimf', [1 3 5], 'Name', 'Close');
    fis = addMF(fis, 'ObstacleDistance', 'trimf', [4 10 10], 'Name', 'Safe');
    
    % OUTPUT: Movement Urgency (0-100)
    fis = addOutput(fis, [0 100], 'Name', 'MovementUrgency');
    fis = addMF(fis, 'MovementUrgency', 'trimf', [0 0 30], 'Name', 'Careful');
    fis = addMF(fis, 'MovementUrgency', 'trimf', [20 50 80], 'Name', 'Normal');
    fis = addMF(fis, 'MovementUrgency', 'trimf', [70 100 100], 'Name', 'Aggressive');
    
    % FUZZY RULES (9 rules)
    rules = [
        1 3 3 1 1;
        1 2 2 1 1;
        1 1 1 1 1;
        2 3 2 1 1;
        2 2 1 1 1;
        2 1 1 1 1;
        3 3 2 1 1;
        3 2 1 1 1;
        3 1 1 1 1;
    ];
    
    fis = addRule(fis, rules);
end

function visualize_membership_functions()
    % Create figure for membership functions
    figure('Name', 'Fuzzy Membership Functions', 'Position', [100 100 1400 800]);
    
    % INPUT 1: Goal Distance
    subplot(2, 2, 1);
    x1 = 0:0.1:30;
    
    % Near: trimf [0 0 10]
    near = trimf(x1, [0 0 10]);
    plot(x1, near, 'r', 'LineWidth', 2.5);
    hold on;
    
    % Medium: trimf [5 15 25]
    medium = trimf(x1, [5 15 25]);
    plot(x1, medium, 'g', 'LineWidth', 2.5);
    
    % Far: trimf [20 30 30]
    far = trimf(x1, [20 30 30]);
    plot(x1, far, 'b', 'LineWidth', 2.5);
    
    title('INPUT 1: Goal Distance', 'FontSize', 14, 'FontWeight', 'bold');
    xlabel('Distance (units)', 'FontSize', 12);
    ylabel('Membership Degree', 'FontSize', 12);
    legend('Near', 'Medium', 'Far', 'Location', 'best', 'FontSize', 11);
    grid on;
    ylim([0 1.1]);
    xlim([0 30]);
    set(gca, 'FontSize', 10);
    
    % INPUT 2: Obstacle Distance
    subplot(2, 2, 2);
    x2 = 0:0.05:10;
    
    % VeryClose: trimf [0 0 2]
    veryclose = trimf(x2, [0 0 2]);
    plot(x2, veryclose, 'r', 'LineWidth', 2.5);
    hold on;
    
    % Close: trimf [1 3 5]
    close = trimf(x2, [1 3 5]);
    plot(x2, close, 'g', 'LineWidth', 2.5);
    
    % Safe: trimf [4 10 10]
    safe = trimf(x2, [4 10 10]);
    plot(x2, safe, 'b', 'LineWidth', 2.5);
    
    title('INPUT 2: Obstacle Distance', 'FontSize', 14, 'FontWeight', 'bold');
    xlabel('Distance (units)', 'FontSize', 12);
    ylabel('Membership Degree', 'FontSize', 12);
    legend('VeryClose', 'Close', 'Safe', 'Location', 'best', 'FontSize', 11);
    grid on;
    ylim([0 1.1]);
    xlim([0 10]);
    set(gca, 'FontSize', 10);
    
    % OUTPUT: Movement Urgency
    subplot(2, 2, [3, 4]);
    x_out = 0:0.5:100;
    
    % Careful: trimf [0 0 30]
    careful = trimf(x_out, [0 0 30]);
    plot(x_out, careful, 'r', 'LineWidth', 2.5);
    hold on;
    
    % Normal: trimf [20 50 80]
    normal = trimf(x_out, [20 50 80]);
    plot(x_out, normal, 'g', 'LineWidth', 2.5);
    
    % Aggressive: trimf [70 100 100]
    aggressive = trimf(x_out, [70 100 100]);
    plot(x_out, aggressive, 'b', 'LineWidth', 2.5);
    
    title('OUTPUT: Movement Urgency', 'FontSize', 14, 'FontWeight', 'bold');
    xlabel('Urgency Level (0-100)', 'FontSize', 12);
    ylabel('Membership Degree', 'FontSize', 12);
    legend('Careful', 'Normal', 'Aggressive', 'Location', 'best', 'FontSize', 11);
    grid on;
    ylim([0 1.1]);
    xlim([0 100]);
    set(gca, 'FontSize', 10);
end

function generate_3d_surface(fis)
    % Generate 3D control surface
    figure('Name', 'Fuzzy Control Surface', 'Position', [150 150 900 700]);
    
    % Create input grid
    goal_dist = linspace(0, 30, 40);
    obs_dist = linspace(0, 10, 40);
    [X, Y] = meshgrid(goal_dist, obs_dist);
    
    % Calculate output for each input combination
    Z = zeros(size(X));
    for i = 1:size(X, 1)
        for j = 1:size(X, 2)
            Z(i, j) = evalfis(fis, [X(i, j), Y(i, j)]);
        end
    end
    
    % Plot 3D surface
    surf(X, Y, Z, 'EdgeColor', 'none', 'FaceAlpha', 0.9);
    colormap(jet);
    h = colorbar;
    ylabel(h, 'Movement Urgency', 'FontSize', 11);
    
    hold on;
    % Add contour lines
    contour3(X, Y, Z, 12, 'k', 'LineWidth', 0.5);
    
    title('Fuzzy Control Surface: Movement Urgency', 'FontSize', 14, 'FontWeight', 'bold');
    xlabel('Goal Distance (units)', 'FontSize', 12);
    ylabel('Obstacle Distance (units)', 'FontSize', 12);
    zlabel('Movement Urgency', 'FontSize', 12);
    
    % Set view angle
    view(135, 30);
    grid on;
    set(gca, 'FontSize', 10);
    
    % Add annotations for key regions
    text(5, 8, 85, '← AGGRESSIVE', 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'w');
    text(25, 2, 25, '← CAREFUL', 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'k');
    text(15, 5, 50, '← NORMAL', 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'k');
end