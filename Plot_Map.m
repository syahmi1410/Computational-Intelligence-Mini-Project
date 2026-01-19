function Plot_Map(map)

figure('Name',map.name,'NumberTitle','off');
hold on; axis equal; grid on;

xlim(map.xlim);
ylim(map.ylim);

% Plot obstacles (no legend for rectangles)
for i = 1:size(map.obstacles,1)
    rectangle('Position', map.obstacles(i,:), ...
              'FaceColor', [0 0 0]);
end

% Plot start and goal (with legend)
hStart = plot(map.start(1), map.start(2), ...
              'go', 'MarkerSize',10, 'LineWidth',2);
hGoal  = plot(map.goal(1), map.goal(2), ...
              'ro', 'MarkerSize',10, 'LineWidth',2);

title(map.name)
xlabel('X')
ylabel('Y')

legend([hStart, hGoal], {'Start','Goal'}, 'Location','best');

end
