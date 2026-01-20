function PathfindingGUI()
    % Robot Pathfinding GUI with Performance Comparison
    
    % Create main figure
    fig = uifigure('Name', 'Robot Pathfinding Simulator', ...
                   'Position', [100 100 1200 700]);
    
    % Create UI components
    createUI(fig);
end

function createUI(fig)
    % Title
    uilabel(fig, 'Position', [400 650 400 40], ...
            'Text', 'Robot Pathfinding Simulator', ...
            'FontSize', 20, 'FontWeight', 'bold', ...
            'HorizontalAlignment', 'center');
    
    % Map Selection Panel
    mapPanel = uipanel(fig, 'Position', [20 500 250 130], ...
                       'Title', 'Map Selection');
    
    uilabel(mapPanel, 'Position', [10 70 200 20], ...
            'Text', 'Select Map:');
    
    mapDropdown = uidropdown(mapPanel, 'Position', [10 40 220 30], ...
                             'Items', {'Easy Map', 'Hard Map'}, ...
                             'Value', 'Easy Map');
    
    % Algorithm Selection Panel
    algoPanel = uipanel(fig, 'Position', [20 320 250 160], ...
                        'Title', 'Algorithm Selection');
    
    fuzzyCheck = uicheckbox(algoPanel, 'Position', [10 100 200 20], ...
                            'Text', 'Fuzzy Logic Only', 'Value', 1);
    
    hybridCheck = uicheckbox(algoPanel, 'Position', [10 70 200 20], ...
                             'Text', 'Hybrid GA-Fuzzy', 'Value', 1);
    
    compareCheck = uicheckbox(algoPanel, 'Position', [10 40 200 20], ...
                              'Text', 'Compare Both', 'Value', 0);
    
    % Parameters Panel
    paramPanel = uipanel(fig, 'Position', [20 100 250 200], ...
                         'Title', 'GA Parameters');
    
    uilabel(paramPanel, 'Position', [10 140 100 20], 'Text', 'Population:');
    popField = uieditfield(paramPanel, 'numeric', 'Position', [120 140 100 20], ...
                           'Value', 150);
    
    uilabel(paramPanel, 'Position', [10 110 100 20], 'Text', 'Generations:');
    genField = uieditfield(paramPanel, 'numeric', 'Position', [120 110 100 20], ...
                           'Value', 200);
    
    uilabel(paramPanel, 'Position', [10 80 100 20], 'Text', 'Crossover:');
    crossField = uieditfield(paramPanel, 'numeric', 'Position', [120 80 100 20], ...
                             'Value', 0.85);
    
    uilabel(paramPanel, 'Position', [10 50 100 20], 'Text', 'Mutation:');
    mutField = uieditfield(paramPanel, 'numeric', 'Position', [120 50 100 20], ...
                           'Value', 0.15);
    
    % Run Button
    runBtn = uibutton(fig, 'Position', [20 40 250 40], ...
                      'Text', 'RUN SIMULATION', ...
                      'FontSize', 14, 'FontWeight', 'bold', ...
                      'BackgroundColor', [0.2 0.8 0.2]);
    
    % Status Text
    statusLabel = uilabel(fig, 'Position', [300 40 800 30], ...
                          'Text', 'Ready to run simulation...', ...
                          'FontSize', 12);
    
    % Results Panel
    resultsAxes = uiaxes(fig, 'Position', [300 100 850 530]);
    title(resultsAxes, 'Simulation Results');
    
    % Store data in figure
    fig.UserData.mapDropdown = mapDropdown;
    fig.UserData.fuzzyCheck = fuzzyCheck;
    fig.UserData.hybridCheck = hybridCheck;
    fig.UserData.compareCheck = compareCheck;
    fig.UserData.popField = popField;
    fig.UserData.genField = genField;
    fig.UserData.crossField = crossField;
    fig.UserData.mutField = mutField;
    fig.UserData.statusLabel = statusLabel;
    fig.UserData.resultsAxes = resultsAxes;
    
    % Button callback
    runBtn.ButtonPushedFcn = @(btn,event) runSimulation(fig);
end

function runSimulation(fig)
    % Get UI data
    data = fig.UserData;
    
    % Get parameters
    mapChoice = data.mapDropdown.Value;
    runFuzzy = data.fuzzyCheck.Value;
    runHybrid = data.hybridCheck.Value;
    compareMode = data.compareCheck.Value;
    
    pop_size = data.popField.Value;
    max_gen = data.genField.Value;
    cross_rate = data.crossField.Value;
    mut_rate = data.mutField.Value;
    
    % Update status
    data.statusLabel.Text = 'Running simulation...';
    drawnow;
    
    % Get map
    if strcmp(mapChoice, 'Easy Map')
        [map_grid, start_pos, goal_pos] = create_easy_map();
    else
        [map_grid, start_pos, goal_pos] = create_hard_map();
    end
    
    results = struct();
    
    % Run Fuzzy Only
    if runFuzzy || compareMode
        data.statusLabel.Text = 'Running Fuzzy Logic...';
        drawnow;
        [path_fuzzy, metrics_fuzzy] = run_fuzzy_controller(map_grid, start_pos, goal_pos);
        results.fuzzy.path = path_fuzzy;
        results.fuzzy.metrics = metrics_fuzzy;
    end
    
    % Run Hybrid GA-Fuzzy
    if runHybrid || compareMode
        data.statusLabel.Text = 'Running Hybrid GA-Fuzzy...';
        drawnow;
        [path_hybrid, metrics_hybrid, fitness_hist] = run_hybrid_ga(map_grid, start_pos, goal_pos, ...
                                                                     pop_size, max_gen, cross_rate, mut_rate);
        results.hybrid.path = path_hybrid;
        results.hybrid.metrics = metrics_hybrid;
        results.hybrid.fitness = fitness_hist;
    end
    
    % Display results
    data.statusLabel.Text = 'Generating visualization...';
    drawnow;
    
    if compareMode
        display_comparison(data.resultsAxes, map_grid, start_pos, goal_pos, results, mapChoice);
    elseif runFuzzy && ~runHybrid
        display_single_result(data.resultsAxes, map_grid, start_pos, goal_pos, ...
                              results.fuzzy.path, results.fuzzy.metrics, 'Fuzzy Logic', mapChoice);
    elseif runHybrid && ~runFuzzy
        display_single_result(data.resultsAxes, map_grid, start_pos, goal_pos, ...
                              results.hybrid.path, results.hybrid.metrics, 'Hybrid GA-Fuzzy', mapChoice);
    end
    
    data.statusLabel.Text = sprintf('Simulation complete! Map: %s', mapChoice);
    
    % Display performance table
    if compareMode
        display_performance_table(results);
    end
end

function [path, metrics] = run_fuzzy_controller(map_grid, start_pos, goal_pos)
    fis = create_fuzzy_system_2input();
    max_steps = 500;
    current_pos = start_pos;
    path = current_pos;
    collisions = 0;
    
    for step = 1:max_steps
        dist_goal = norm(current_pos - goal_pos);
        obs_dist = calculate_obstacle_distance(current_pos, map_grid);
        
        if dist_goal < 1.5
            break;
        end
        
        urgency = evalfis(fis, [dist_goal, obs_dist]);
        
        dx = goal_pos(2) - current_pos(2);
        dy = goal_pos(1) - current_pos(1);
        angle = atan2d(dy, dx);
        if angle < 0
            angle = angle + 360;
        end
        
        move = select_move_fuzzy(angle, urgency, current_pos, map_grid, goal_pos);
        moves = [-1 0; 1 0; 0 -1; 0 1; -1 1; -1 -1; 1 1; 1 -1];
        new_pos = current_pos + moves(move, :);
        
        [rows, cols] = size(map_grid);
        if new_pos(1) < 1 || new_pos(1) > rows || new_pos(2) < 1 || new_pos(2) > cols
            collisions = collisions + 1;
            % Try alternative move
            for alt_move = 1:8
                alt_pos = current_pos + moves(alt_move, :);
                if alt_pos(1) >= 1 && alt_pos(1) <= rows && alt_pos(2) >= 1 && alt_pos(2) <= cols
                    if map_grid(alt_pos(1), alt_pos(2)) ~= 1
                        new_pos = alt_pos;
                        break;
                    end
                end
            end
            if new_pos(1) < 1 || new_pos(1) > rows || new_pos(2) < 1 || new_pos(2) > cols
                continue;
            end
        end
        
        if map_grid(new_pos(1), new_pos(2)) == 1
            collisions = collisions + 1;
            % Try to find alternative safe move
            found_safe = false;
            for alt_move = 1:8
                alt_pos = current_pos + moves(alt_move, :);
                if alt_pos(1) >= 1 && alt_pos(1) <= rows && alt_pos(2) >= 1 && alt_pos(2) <= cols
                    if map_grid(alt_pos(1), alt_pos(2)) ~= 1
                        new_pos = alt_pos;
                        found_safe = true;
                        break;
                    end
                end
            end
            if ~found_safe
                continue;
            end
        end
        
        current_pos = new_pos;
        path = [path; current_pos];
    end
    
    metrics = compute_metrics(path, goal_pos, collisions, 0);
end

function [path, metrics, fitness_hist] = run_hybrid_ga(map_grid, start_pos, goal_pos, ...
                                                        pop_size, max_gen, cross_rate, mut_rate)
    fis = create_fuzzy_system_2input();
    path_length = 100;
    population = initialize_pop(pop_size, path_length);
    
    best_path = [];
    best_fitness = -inf;
    fitness_hist = zeros(max_gen, 1);
    
    for gen = 1:max_gen
        [fitness, paths] = eval_pop_ga(population, map_grid, start_pos, goal_pos, fis);
        [fitness_hist(gen), idx] = max(fitness);
        
        if fitness_hist(gen) > best_fitness
            best_fitness = fitness_hist(gen);
            best_path = paths{idx};
        end
        
        if check_goal(best_path, goal_pos)
            fitness_hist = fitness_hist(1:gen);
            break;
        end
        
        parents = tournament_select(population, fitness, pop_size - 10);
        [~, sidx] = sort(fitness, 'descend');
        elite = population(sidx(1:10), :);
        offspring = do_crossover(parents, cross_rate);
        offspring = do_mutation(offspring, mut_rate);
        population = [elite; offspring];
    end
    
    path = best_path;
    collisions = count_collisions(path, map_grid);
    metrics = compute_metrics(path, goal_pos, collisions, gen);
end

function fis = create_fuzzy_system_2input()
    fis = mamfis('Name', 'NavController');
    fis = addInput(fis, [0 30], 'Name', 'GoalDist');
    fis = addMF(fis, 'GoalDist', 'trimf', [0 0 10], 'Name', 'near');
    fis = addMF(fis, 'GoalDist', 'trimf', [5 15 25], 'Name', 'med');
    fis = addMF(fis, 'GoalDist', 'trimf', [20 30 30], 'Name', 'far');
    
    fis = addInput(fis, [0 10], 'Name', 'ObsDist');
    fis = addMF(fis, 'ObsDist', 'trimf', [0 0 2], 'Name', 'close');
    fis = addMF(fis, 'ObsDist', 'trimf', [1 3 5], 'Name', 'med');
    fis = addMF(fis, 'ObsDist', 'trimf', [4 10 10], 'Name', 'safe');
    
    fis = addOutput(fis, [0 100], 'Name', 'Urgency');
    fis = addMF(fis, 'Urgency', 'trimf', [0 0 30], 'Name', 'low');
    fis = addMF(fis, 'Urgency', 'trimf', [25 50 75], 'Name', 'mod');
    fis = addMF(fis, 'Urgency', 'trimf', [70 100 100], 'Name', 'high');
    
    rules = [1 3 3 1 1; 1 2 2 1 1; 1 1 1 1 1; 2 3 2 1 1; 2 2 1 1 1; 
             2 1 1 1 1; 3 3 2 1 1; 3 2 1 1 1; 3 1 1 1 1];
    fis = addRule(fis, rules);
end

function dist = calculate_obstacle_distance(pos, map_grid)
    [rows, cols] = size(map_grid);
    min_dist = 10;
    for i = -5:5
        for j = -5:5
            cp = pos + [i, j];
            if cp(1) >= 1 && cp(1) <= rows && cp(2) >= 1 && cp(2) <= cols
                if map_grid(cp(1), cp(2)) == 1
                    min_dist = min(min_dist, sqrt(i^2 + j^2));
                end
            end
        end
    end
    dist = min_dist;
end

function move = select_move_fuzzy(angle, urgency, pos, map_grid, goal_pos)
    % Movements: 1=Up, 2=Down, 3=Left, 4=Right, 5=UpRight, 6=UpLeft, 7=DownRight, 8=DownLeft
    
    % Determine primary direction based on angle to goal
    if angle >= -22.5 && angle < 22.5
        primary_moves = [4, 5, 7]; % Right
    elseif angle >= 22.5 && angle < 67.5
        primary_moves = [7, 4, 2]; % DownRight
    elseif angle >= 67.5 && angle < 112.5
        primary_moves = [2, 7, 8]; % Down
    elseif angle >= 112.5 && angle < 157.5
        primary_moves = [8, 2, 3]; % DownLeft
    elseif angle >= 157.5 || angle < -157.5
        primary_moves = [3, 6, 8]; % Left
    elseif angle >= -157.5 && angle < -112.5
        primary_moves = [6, 3, 1]; % UpLeft
    elseif angle >= -112.5 && angle < -67.5
        primary_moves = [1, 6, 5]; % Up
    else % -67.5 to -22.5
        primary_moves = [5, 1, 4]; % UpRight
    end
    
    % If urgency is low (careful mode), check for obstacles
    [rows, cols] = size(map_grid);
    moves_def = [-1 0; 1 0; 0 -1; 0 1; -1 1; -1 -1; 1 1; 1 -1];
    
    if urgency < 40
        % Careful mode: try to find safe move from primary directions
        for m = primary_moves
            test_pos = pos + moves_def(m, :);
            if test_pos(1) >= 1 && test_pos(1) <= rows && ...
               test_pos(2) >= 1 && test_pos(2) <= cols
                if map_grid(test_pos(1), test_pos(2)) ~= 1
                    move = m;
                    return;
                end
            end
        end
        
        % If all primary moves blocked, try any safe move toward goal
        best_move = primary_moves(1);
        best_score = -inf;
        for m = 1:8
            test_pos = pos + moves_def(m, :);
            if test_pos(1) >= 1 && test_pos(1) <= rows && ...
               test_pos(2) >= 1 && test_pos(2) <= cols
                if map_grid(test_pos(1), test_pos(2)) ~= 1
                    % Score based on distance to goal
                    dist = norm(test_pos - goal_pos);
                    score = -dist;
                    if score > best_score
                        best_score = score;
                        best_move = m;
                    end
                end
            end
        end
        move = best_move;
    else
        % Aggressive/Normal mode: use primary direction
        move = primary_moves(1);
    end
end

function pop = initialize_pop(pop_size, path_len)
    pop = randi(8, pop_size, path_len);
end

function [fit, paths] = eval_pop_ga(pop, map_grid, start, goal, fis)
    n = size(pop, 1);
    fit = zeros(n, 1);
    paths = cell(n, 1);
    for i = 1:n
        [fit(i), paths{i}] = eval_ind(pop(i,:), map_grid, start, goal, fis);
    end
end

function [fitness, path] = eval_ind(chromo, map_grid, start, goal, fis)
    [rows, cols] = size(map_grid);
    pos = start;
    path = pos;
    coll = 0;
    steps = 0;
    min_dist = norm(start - goal);
    moves = [-1 0; 1 0; 0 -1; 0 1; -1 1; -1 -1; 1 1; 1 -1];
    
    for m = chromo
        steps = steps + 1;
        new_p = pos + moves(m, :);
        if new_p(1) < 1 || new_p(1) > rows || new_p(2) < 1 || new_p(2) > cols
            coll = coll + 1;
            continue;
        end
        if map_grid(new_p(1), new_p(2)) == 1
            coll = coll + 1;
            continue;
        end
        
        pos = new_p;
        path = [path; pos];
        d = norm(pos - goal);
        min_dist = min(min_dist, d);
        if d < 1.5, break; end
    end
    
    d = norm(pos - goal);
    bonus = 0;
    if d < 1.5, bonus = 10000 - steps * 5;
    elseif d < 3, bonus = 3000;
    end
    
    fitness = bonus + (norm(start-goal) - min_dist)*50 - d*20 - coll*150 - steps*0.5;
end

function parents = tournament_select(pop, fit, num)
    n = size(pop, 1);
    parents = zeros(num, size(pop, 2));
    fit = fit - min(fit) + 1;
    for i = 1:num
        c = randi(n, 3, 1);
        [~, w] = max(fit(c));
        parents(i, :) = pop(c(w), :);
    end
end

function off = do_crossover(par, rate)
    n = size(par, 1);
    off = par;
    for i = 1:2:n-1
        if rand() < rate
            pts = sort(randi(size(par,2), 1, 2));
            temp = off(i, pts(1):pts(2));
            off(i, pts(1):pts(2)) = off(i+1, pts(1):pts(2));
            off(i+1, pts(1):pts(2)) = temp;
        end
    end
end

function off = do_mutation(off, rate)
    [n, m] = size(off);
    for i = 1:n
        for j = 1:m
            if rand() < rate
                off(i, j) = randi(8);
            end
        end
    end
end

function reached = check_goal(path, goal)
    if isempty(path), reached = false; return; end
    reached = any(sqrt(sum((path - goal).^2, 2)) < 1.5);
end

function coll = count_collisions(path, map_grid)
    coll = 0;
    [rows, cols] = size(map_grid);
    for i = 1:size(path, 1)
        p = path(i, :);
        if p(1) >= 1 && p(1) <= rows && p(2) >= 1 && p(2) <= cols
            if map_grid(p(1), p(2)) == 1
                coll = coll + 1;
            end
        end
    end
end

function metrics = compute_metrics(path, goal, coll, gen)
    metrics.goal_reached = check_goal(path, goal);
    metrics.path_length = size(path, 1);
    metrics.collisions = coll;
    metrics.generations = gen;
    
    dc = 0;
    if size(path, 1) > 2
        for i = 2:size(path, 1)-1
            if ~isequal(path(i,:)-path(i-1,:), path(i+1,:)-path(i,:))
                dc = dc + 1;
            end
        end
    end
    metrics.smoothness = 100 - (dc / max(1, size(path, 1)) * 100);
    metrics.success_rate = double(metrics.goal_reached) * 100;
    metrics.final_distance = norm(path(end, :) - goal);
end

function [m, s, g] = create_easy_map()
    m = zeros(20, 20);
    s = [2, 2];
    g = [18, 18];
    obs = [6 6 3 3; 11 10 3 3];
    for i = 1:size(obs, 1)
        m(obs(i,2):obs(i,2)+obs(i,4)-1, obs(i,1):obs(i,1)+obs(i,3)-1) = 1;
    end
    m(s(1), s(2)) = 2;
    m(g(1), g(2)) = 3;
end

function [m, s, g] = create_hard_map()
    m = zeros(20, 20);
    s = [10, 2];
    g = [10, 18];
    obs = [5 0 1 8; 5 12 1 8; 9 4 1 12; 12 0 1 8; 12 12 1 8; 15 4 1 12];
    for i = 1:size(obs, 1)
        rs = max(1, obs(i,2)+1);
        re = min(20, obs(i,2)+obs(i,4));
        cs = max(1, obs(i,1)+1);
        ce = min(20, obs(i,1)+obs(i,3));
        if rs <= 20 && cs <= 20
            m(rs:re, cs:ce) = 1;
        end
    end
    m(s(1), s(2)) = 2;
    m(g(1), g(2)) = 3;
end

function display_comparison(ax, map_grid, start, goal, results, mapName)
    cla(ax);
    cmap = [1 1 1; 0.2 0.2 0.2; 0 1 0; 1 0 0];
    colormap(ax, cmap);
    imagesc(ax, map_grid);
    hold(ax, 'on');
    axis(ax, 'equal', 'tight');
    
    if isfield(results, 'fuzzy')
        plot(ax, results.fuzzy.path(:,2), results.fuzzy.path(:,1), ...
             'r-', 'LineWidth', 2, 'DisplayName', 'Fuzzy');
    end
    
    if isfield(results, 'hybrid')
        plot(ax, results.hybrid.path(:,2), results.hybrid.path(:,1), ...
             'b-', 'LineWidth', 2, 'DisplayName', 'Hybrid GA-Fuzzy');
    end
    
    plot(ax, goal(2), goal(1), 'k*', 'MarkerSize', 20, 'LineWidth', 2, ...
         'DisplayName', 'Goal');
    
    title(ax, sprintf('%s - Algorithm Comparison', mapName));
    legend(ax, 'Location', 'best');
end

function display_single_result(ax, map_grid, start, goal, path, metrics, algoName, mapName)
    cla(ax);
    cmap = [1 1 1; 0.2 0.2 0.2; 0 1 0; 1 0 0];
    colormap(ax, cmap);
    imagesc(ax, map_grid);
    hold(ax, 'on');
    axis(ax, 'equal', 'tight');
    
    if ~isempty(path)
        plot(ax, path(:,2), path(:,1), 'b-', 'LineWidth', 2.5);
        plot(ax, path(:,2), path(:,1), 'bo', 'MarkerSize', 6, 'MarkerFaceColor', 'b');
    end
    
    plot(ax, goal(2), goal(1), 'r*', 'MarkerSize', 25, 'LineWidth', 3);
    title(ax, sprintf('%s - %s\nGoal: %s | Length: %d | Smooth: %.1f%%', ...
                      mapName, algoName, mat2str(metrics.goal_reached), ...
                      metrics.path_length, metrics.smoothness));
end

function display_performance_table(results)
    fig = uifigure('Name', 'Performance Comparison', 'Position', [200 200 700 300]);
    
    data = {};
    rowNames = {};
    
    if isfield(results, 'fuzzy')
        m = results.fuzzy.metrics;
        data{end+1} = {'Fuzzy Logic', m.goal_reached, m.path_length, m.collisions, ...
                       sprintf('%.1f%%', m.smoothness), sprintf('%.2f', m.final_distance)};
        rowNames{end+1} = 'Fuzzy';
    end
    
    if isfield(results, 'hybrid')
        m = results.hybrid.metrics;
        data{end+1} = {'Hybrid GA-Fuzzy', m.goal_reached, m.path_length, m.collisions, ...
                       sprintf('%.1f%%', m.smoothness), sprintf('%.2f', m.final_distance)};
        rowNames{end+1} = 'Hybrid';
    end
    
    uit = uitable(fig, 'Data', data, ...
                  'ColumnName', {'Algorithm', 'Goal Reached', 'Path Length', ...
                                 'Collisions', 'Smoothness', 'Final Distance'}, ...
                  'Position', [20 20 660 250]);
end