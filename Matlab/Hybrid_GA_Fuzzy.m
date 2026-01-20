% Hybrid Fuzzy-GA Robot Pathfinding with Performance Evaluation
clear all; close all; clc;

%% Choose Map
fprintf('=================================\n');
fprintf('HYBRID FUZZY-GA PATHFINDING\n');
fprintf('=================================\n');
fprintf('Select Map:\n');
fprintf('1. Easy Map\n');
fprintf('2. Hard Map\n');
map_choice = input('Enter choice (1 or 2): ');

if map_choice == 1
    [map_grid, start_pos, goal_pos] = create_easy_map();
    map_name = 'Easy Map';
else
    [map_grid, start_pos, goal_pos] = create_hard_map();
    map_name = 'Hard Map';
end

%% GA Parameters
pop_size = 150;
max_generations = 300;
crossover_rate = 0.85;
mutation_rate = 0.15;
elite_count = 10;
path_length = 100;

%% Initialize
fprintf('\nInitializing Hybrid Fuzzy-GA...\n');
population = initialize_population(pop_size, path_length, start_pos);
fis = create_fuzzy_system();

best_fitness_history = zeros(max_generations, 1);
best_path_overall = [];
best_fitness_overall = -inf;

fprintf('Running Hybrid GA-Fuzzy...\n\n');

%% GA Evolution with Fuzzy Evaluation
for gen = 1:max_generations
    [fitness, paths] = evaluate_population(population, map_grid, start_pos, goal_pos, fis);
    
    [best_fitness_history(gen), best_idx] = max(fitness);
    
    if best_fitness_history(gen) > best_fitness_overall
        best_fitness_overall = best_fitness_history(gen);
        best_path_overall = paths{best_idx};
    end
    
    if mod(gen, 20) == 0 || gen == 1
        fprintf('Gen %3d: Best Fitness = %8.2f | Path Length = %d\n', ...
            gen, best_fitness_history(gen), size(paths{best_idx}, 1));
    end
    
    best_path = paths{best_idx};
    if check_goal_reached(best_path, goal_pos)
        fprintf('\n*** GOAL REACHED at generation %d! ***\n', gen);
        best_path_overall = best_path;
        break;
    end
    
    parents = selection(population, fitness, pop_size - elite_count);
    [~, sorted_idx] = sort(fitness, 'descend');
    elite = population(sorted_idx(1:elite_count), :);
    offspring = crossover(parents, crossover_rate);
    offspring = mutation(offspring, mutation_rate);
    population = [elite; offspring];
end

%% Calculate Performance Metrics
metrics_ga = calculate_performance_metrics(best_path_overall, map_grid, start_pos, goal_pos, gen);

%% Display Results
fprintf('\nGenerating visualization...\n');
display_ga_results(map_grid, best_path_overall, start_pos, goal_pos, map_name, best_fitness_history, gen, metrics_ga);

%% ========== FUNCTIONS ==========

function fis = create_fuzzy_system()
    % Enhanced Fuzzy System: 2 inputs, 1 output
    fis = mamfis('Name', 'ObstacleAvoidance');
    
    % INPUT 1: Distance to Goal
    fis = addInput(fis, [0 30], 'Name', 'GoalDistance');
    fis = addMF(fis, 'GoalDistance', 'trimf', [0 0 10], 'Name', 'close');
    fis = addMF(fis, 'GoalDistance', 'trimf', [5 15 25], 'Name', 'medium');
    fis = addMF(fis, 'GoalDistance', 'trimf', [20 30 30], 'Name', 'far');
    
    % INPUT 2: Obstacle Proximity
    fis = addInput(fis, [0 10], 'Name', 'ObstacleProximity');
    fis = addMF(fis, 'ObstacleProximity', 'trimf', [0 0 2], 'Name', 'veryClose');
    fis = addMF(fis, 'ObstacleProximity', 'trimf', [1 3 5], 'Name', 'close');
    fis = addMF(fis, 'ObstacleProximity', 'trimf', [4 10 10], 'Name', 'safe');
    
    % OUTPUT: Risk Level
    fis = addOutput(fis, [0 100], 'Name', 'RiskLevel');
    fis = addMF(fis, 'RiskLevel', 'trimf', [0 0 30], 'Name', 'low');
    fis = addMF(fis, 'RiskLevel', 'trimf', [25 50 75], 'Name', 'moderate');
    fis = addMF(fis, 'RiskLevel', 'trimf', [70 100 100], 'Name', 'high');
    
    % Fuzzy Rules
    rules = [
        1 3 1 1 1;  % close goal, safe obstacle -> low risk
        1 2 2 1 1;  % close goal, close obstacle -> moderate risk
        1 1 3 1 1;  % close goal, very close obstacle -> high risk
        2 3 1 1 1;  % medium goal, safe obstacle -> low risk
        2 2 2 1 1;  % medium goal, close obstacle -> moderate risk
        2 1 3 1 1;  % medium goal, very close obstacle -> high risk
        3 3 2 1 1;  % far goal, safe obstacle -> moderate risk
        3 2 2 1 1;  % far goal, close obstacle -> moderate risk
        3 1 3 1 1;  % far goal, very close obstacle -> high risk
    ];
    fis = addRule(fis, rules);
end

function [fitness, paths] = evaluate_population(population, map_grid, start_pos, goal_pos, fis)
    pop_size = size(population, 1);
    fitness = zeros(pop_size, 1);
    paths = cell(pop_size, 1);
    
    for i = 1:pop_size
        [fitness(i), paths{i}] = evaluate_individual(population(i, :), map_grid, start_pos, goal_pos, fis);
    end
end

function [fitness, path] = evaluate_individual(chromosome, map_grid, start_pos, goal_pos, fis)
    [rows, cols] = size(map_grid);
    current_pos = start_pos;
    path = current_pos;
    
    collision_count = 0;
    fuzzy_risk_total = 0;
    steps = 0;
    min_distance_to_goal = norm(start_pos - goal_pos);
    
    moves = [-1 0; 1 0; 0 -1; 0 1; -1 1; -1 -1; 1 1; 1 -1];
    
    for move_idx = chromosome
        steps = steps + 1;
        new_pos = current_pos + moves(move_idx, :);
        
        if new_pos(1) < 1 || new_pos(1) > rows || new_pos(2) < 1 || new_pos(2) > cols
            collision_count = collision_count + 1;
            continue;
        end
        
        if map_grid(new_pos(1), new_pos(2)) == 1
            collision_count = collision_count + 1;
            continue;
        end
        
        % Fuzzy evaluation with 2 inputs
        goal_dist = norm(new_pos - goal_pos);
        obs_dist = calculate_obstacle_distance(new_pos, map_grid);
        risk = evalfis(fis, [goal_dist, obs_dist]);
        fuzzy_risk_total = fuzzy_risk_total + risk * 0.5;
        
        current_pos = new_pos;
        path = [path; current_pos];
        
        if goal_dist < min_distance_to_goal
            min_distance_to_goal = goal_dist;
        end
        
        if goal_dist < 1.5
            break;
        end
    end
    
    distance_to_goal = norm(current_pos - goal_pos);
    goal_bonus = 0;
    if distance_to_goal < 1.5
        goal_bonus = 10000 - steps * 5;
    elseif distance_to_goal < 3
        goal_bonus = 3000;
    elseif distance_to_goal < 5
        goal_bonus = 1000;
    end
    
    progress_bonus = (norm(start_pos - goal_pos) - min_distance_to_goal) * 50;
    collision_penalty = collision_count * 150;
    path_length_penalty = steps * 0.5;
    
    fitness = goal_bonus + progress_bonus - distance_to_goal * 20 - collision_penalty - path_length_penalty - fuzzy_risk_total;
end

function distance = calculate_obstacle_distance(pos, map_grid)
    [rows, cols] = size(map_grid);
    min_dist = 10;
    
    for i = -5:5
        for j = -5:5
            check_pos = pos + [i, j];
            if check_pos(1) >= 1 && check_pos(1) <= rows && ...
               check_pos(2) >= 1 && check_pos(2) <= cols
                if map_grid(check_pos(1), check_pos(2)) == 1
                    dist = sqrt(i^2 + j^2);
                    min_dist = min(min_dist, dist);
                end
            end
        end
    end
    distance = min_dist;
end

function population = initialize_population(pop_size, path_length, start_pos)
    population = zeros(pop_size, path_length);
    useful_moves = [4, 5, 7];
    
    for i = 1:pop_size
        if i <= pop_size * 0.3
            population(i, :) = randi(8, 1, path_length);
        else
            for j = 1:path_length
                if rand() < 0.6
                    population(i, j) = useful_moves(randi(3));
                else
                    population(i, j) = randi(8);
                end
            end
        end
    end
end

function parents = selection(population, fitness, num_parents)
    pop_size = size(population, 1);
    parents = zeros(num_parents, size(population, 2));
    
    min_fitness = min(fitness);
    if min_fitness < 0
        fitness = fitness - min_fitness + 1;
    end
    
    for i = 1:num_parents
        contestants = randi(pop_size, 3, 1);
        [~, winner_idx] = max(fitness(contestants));
        parents(i, :) = population(contestants(winner_idx), :);
    end
end

function offspring = crossover(parents, crossover_rate)
    num_parents = size(parents, 1);
    offspring = parents;
    
    for i = 1:2:num_parents-1
        if rand() < crossover_rate
            points = sort(randi([1, size(parents, 2)], 1, 2));
            temp = offspring(i, points(1):points(2));
            offspring(i, points(1):points(2)) = offspring(i+1, points(1):points(2));
            offspring(i+1, points(1):points(2)) = temp;
        end
    end
end

function offspring = mutation(offspring, mutation_rate)
    [num_offspring, gene_length] = size(offspring);
    useful_moves = [4, 5, 7];
    
    for i = 1:num_offspring
        for j = 1:gene_length
            if rand() < mutation_rate
                if rand() < 0.5
                    offspring(i, j) = useful_moves(randi(3));
                else
                    offspring(i, j) = randi(8);
                end
            end
        end
    end
end

function reached = check_goal_reached(path, goal_pos)
    if isempty(path)
        reached = false;
        return;
    end
    distances = sqrt(sum((path - goal_pos).^2, 2));
    reached = any(distances < 1.5);
end

function metrics = calculate_performance_metrics(path, map_grid, start_pos, goal_pos, generations)
    metrics.goal_reached = check_goal_reached(path, goal_pos);
    metrics.path_length = size(path, 1);
    metrics.generations = generations;
    
    % Count collisions
    collisions = 0;
    if ~isempty(path)
        for i = 1:size(path, 1)
            pos = path(i, :);
            [rows, cols] = size(map_grid);
            if pos(1) >= 1 && pos(1) <= rows && pos(2) >= 1 && pos(2) <= cols
                if map_grid(pos(1), pos(2)) == 1
                    collisions = collisions + 1;
                end
            end
        end
    end
    metrics.collisions = collisions;
    
    % Path smoothness
    direction_changes = 0;
    if size(path, 1) > 2
        for i = 2:size(path, 1)-1
            dir1 = path(i, :) - path(i-1, :);
            dir2 = path(i+1, :) - path(i, :);
            if ~isequal(dir1, dir2)
                direction_changes = direction_changes + 1;
            end
        end
    end
    metrics.direction_changes = direction_changes;
    metrics.smoothness = 100 - (direction_changes / max(1, size(path, 1)) * 100);
    
    % Success rate
    metrics.success_rate = double(metrics.goal_reached) * 100;
    
    % Final distance
    if ~isempty(path)
        metrics.final_distance = norm(path(end, :) - goal_pos);
    else
        metrics.final_distance = inf;
    end
end

function [map_grid, start_pos, goal_pos] = create_easy_map()
    map_grid = zeros(20, 20);
    start_pos = [2, 2];
    goal_pos = [18, 18];
    obstacles = [6 6 3 3; 11 10 3 3];
    for i = 1:size(obstacles, 1)
        x = obstacles(i,1); y = obstacles(i,2);
        w = obstacles(i,3); h = obstacles(i,4);
        map_grid(y:y+h-1, x:x+w-1) = 1;
    end
    map_grid(start_pos(1), start_pos(2)) = 2;
    map_grid(goal_pos(1), goal_pos(2)) = 3;
end

function [map_grid, start_pos, goal_pos] = create_hard_map()
    map_grid = zeros(20, 20);
    start_pos = [10, 2];
    goal_pos = [10, 18];
    obstacles = [5 0 1 8; 5 12 1 8; 9 4 1 12; 12 0 1 8; 12 12 1 8; 15 4 1 12];
    for i = 1:size(obstacles, 1)
        x = obstacles(i,1); y = obstacles(i,2);
        w = obstacles(i,3); h = obstacles(i,4);
        row_start = max(1, y+1); row_end = min(20, y+h);
        col_start = max(1, x+1); col_end = min(20, x+w);
        if row_start <= 20 && col_start <= 20
            map_grid(row_start:row_end, col_start:col_end) = 1;
        end
    end
    map_grid(start_pos(1), start_pos(2)) = 2;
    map_grid(goal_pos(1), goal_pos(2)) = 3;
end

function display_ga_results(map_grid, best_path, start_pos, goal_pos, map_name, fitness_history, final_gen, metrics)
    figure('Name', 'Hybrid Fuzzy-GA Results', 'Position', [50 50 1400 600]);
    
    % Plot path
    subplot(1, 2, 1);
    cmap = [1 1 1; 0.2 0.2 0.2; 0 1 0; 1 0 0];
    colormap(cmap);
    imagesc(map_grid);
    hold on;
    axis equal tight;
    
    if ~isempty(best_path)
        plot(best_path(:,2), best_path(:,1), 'b-', 'LineWidth', 2.5);
        plot(best_path(:,2), best_path(:,1), 'bo', 'MarkerSize', 6, 'MarkerFaceColor', 'b');
        plot(best_path(1,2), best_path(1,1), 'go', 'MarkerSize', 20, 'MarkerFaceColor', 'g', 'LineWidth', 3);
        plot(best_path(end,2), best_path(end,1), 'mo', 'MarkerSize', 15, 'MarkerFaceColor', 'm', 'LineWidth', 2);
    end
    
    plot(goal_pos(2), goal_pos(1), 'r*', 'MarkerSize', 25, 'LineWidth', 3);
    title(sprintf('%s - Hybrid GA-Fuzzy Path', map_name), 'FontSize', 14, 'FontWeight', 'bold');
    xlabel('Column (X)'); ylabel('Row (Y)');
    legend('Path', 'Robot', 'Start', 'Current', 'Goal', 'Location', 'best');
    
    % Performance metrics
    subplot(1, 2, 2);
    axis off;
    text(0.1, 0.95, 'HYBRID GA-FUZZY PERFORMANCE', 'FontSize', 14, 'FontWeight', 'bold');
    text(0.1, 0.85, sprintf('Map: %s', map_name), 'FontSize', 11);
    text(0.1, 0.75, sprintf('Goal Reached: %s', mat2str(metrics.goal_reached)), 'FontSize', 11);
    text(0.1, 0.65, sprintf('Success Rate: %.1f%%', metrics.success_rate), 'FontSize', 11);
    text(0.1, 0.55, sprintf('Generations: %d', metrics.generations), 'FontSize', 11);
    text(0.1, 0.45, sprintf('Path Length: %d steps', metrics.path_length), 'FontSize', 11);
    text(0.1, 0.35, sprintf('Collisions: %d', metrics.collisions), 'FontSize', 11);
    text(0.1, 0.25, sprintf('Direction Changes: %d', metrics.direction_changes), 'FontSize', 11);
    text(0.1, 0.15, sprintf('Path Smoothness: %.1f%%', metrics.smoothness), 'FontSize', 11);
    text(0.1, 0.05, sprintf('Final Distance: %.2f', metrics.final_distance), 'FontSize', 11);
    
    fprintf('\n=== HYBRID GA-FUZZY RESULTS ===\n');
    fprintf('Goal Reached: %d\n', metrics.goal_reached);
    fprintf('Path Length: %d\n', metrics.path_length);
    fprintf('Collisions: %d\n', metrics.collisions);
    fprintf('Smoothness: %.2f%%\n', metrics.smoothness);
    fprintf('================================\n');
end