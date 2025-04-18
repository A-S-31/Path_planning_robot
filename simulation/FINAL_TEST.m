clc; clear; close all;

% Grid size
gridSize = [10, 10];

% Start and Goal
startPos = [1, 1];
goalPos = [10, 10];

% Initialize map (0 = free, 1 = obstacle)
map = zeros(gridSize);

% Visualization Setup
figure('Name', 'Robot Path Planning Simulator', 'NumberTitle', 'off');
axis equal;
xlim([0 gridSize(2)]);
ylim([0 gridSize(1)]);
set(gca, 'YDir', 'reverse');
grid on;
hold on;

% Draw grid
for x = 0:gridSize(2)
    plot([x x], [0 gridSize(1)], 'k');
end
for y = 0:gridSize(1)
    plot([0 gridSize(2)], [y y], 'k');
end

% Draw Start and Goal
plot(startPos(2)-0.5, startPos(1)-0.5, 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot(goalPos(2)-0.5, goalPos(1)-0.5, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

% Allow user to place initial obstacles
disp('Click to place obstacles. Press Enter when done.');
obstacleHandles = [];
while true
    [x, y] = ginput(1);
    if isempty(x)
        break;
    end
    col = ceil(x); row = ceil(y);
    if row <= gridSize(1) && col <= gridSize(2)
        if map(row, col) == 0
            map(row, col) = 1;
            h = fill([col-1 col col col-1], [row-1 row-1 row row], 'k');
            obstacleHandles = [obstacleHandles; h];
        end
    end
end

% Run A* pathfinding
[path, pathFound] = a_star(startPos, goalPos, map);
if ~pathFound
    error('No path found!');
end

% Plot initial path
pathLine = plot(path(:,2)-0.5, path(:,1)-0.5, 'b-', 'LineWidth', 2);
robot = plot(startPos(2)-0.5, startPos(1)-0.5, 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');

% Robot movement and real-time obstacle placement
i = 2;
while i <= size(path, 1)
    current = path(i, :);

    % Allow user to click to place obstacle in real time
    if waitforbuttonpress
        cp = get(gca, 'CurrentPoint');
        col = ceil(cp(1,1)); row = ceil(cp(1,2));
        if row >= 1 && row <= gridSize(1) && col >= 1 && col <= gridSize(2)
            if map(row, col) == 0
                map(row, col) = 1;
                h = fill([col-1 col col col-1], [row-1 row-1 row row], 'k');
                obstacleHandles = [obstacleHandles; h];
            end
        end
    end

    % Check if next cell is blocked
    if map(current(1), current(2)) == 1
        disp('⚠️ Obstacle encountered! Recalculating...');
        [path, pathFound] = a_star(path(i-1,:), goalPos, map);
        if ~pathFound
            error('No alternative path found!');
        end
        delete(pathLine);
        pathLine = plot(path(:,2)-0.5, path(:,1)-0.5, 'b--', 'LineWidth', 2);
        i = 2; % restart movement from new position
        continue;
    end

    % Move robot
    robot.XData = current(2)-0.5;
    robot.YData = current(1)-0.5;
    pause(0.3); % Control speed
    i = i + 1;
end

disp('🎯 Goal reached!');

%% --- A* Pathfinding ---
function [path, found] = a_star(start, goal, map)
    directions = [0 1; 1 0; 0 -1; -1 0];
    [rows, cols] = size(map);

    cameFrom = containers.Map;
    gScore = inf(rows, cols);
    fScore = inf(rows, cols);

    gScore(start(1), start(2)) = 0;
    fScore(start(1), start(2)) = heuristic(start, goal);

    openSet = [start];

    while ~isempty(openSet)
        current = openSet(1, :);
        minF = fScore(current(1), current(2));
        for i = 2:size(openSet,1)
            if fScore(openSet(i,1), openSet(i,2)) < minF
                current = openSet(i,:);
                minF = fScore(current(1), current(2));
            end
        end

        if all(current == goal)
            path = reconstruct_path(cameFrom, current);
            found = true;
            return;
        end

        openSet = setdiff(openSet, current, 'rows');

        for d = 1:4
            neighbor = current + directions(d,:);
            if any(neighbor < 1) || neighbor(1) > rows || neighbor(2) > cols
                continue;
            end
            if map(neighbor(1), neighbor(2)) == 1
                continue;
            end

            tentative_gScore = gScore(current(1), current(2)) + 1;

            if tentative_gScore < gScore(neighbor(1), neighbor(2))
                key = sprintf('%d,%d', neighbor(1), neighbor(2));
                cameFrom(key) = current;
                gScore(neighbor(1), neighbor(2)) = tentative_gScore;
                fScore(neighbor(1), neighbor(2)) = tentative_gScore + heuristic(neighbor, goal);
                if ~ismember(neighbor, openSet, 'rows')
                    openSet = [openSet; neighbor];
                end
            end
        end
    end

    path = [];
    found = false;
end

function h = heuristic(a, b)
    h = abs(a(1) - b(1)) + abs(a(2) - b(2));
end

function path = reconstruct_path(cameFrom, current)
    path = current;
    while true
        key = sprintf('%d,%d', current(1), current(2));
        if ~isKey(cameFrom, key)
            break;
        end
        current = cameFrom(key);
        path = [current; path];
    end
end
