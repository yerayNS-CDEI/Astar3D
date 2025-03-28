%% RESULTS VISUALIZATION OF THE 3D A* ALGORITHM

result_path = [[2, 2, 1]; [2, 3, 1]; [3, 3, 1]; [3, 4, 1]; [4, 4, 1]; [4, 4, 2]; [4, 5, 2]; ...
    [5, 5, 2]; [5, 5, 3]; [5, 6, 3]; [6, 6, 3]; [6, 6, 4]; [6, 7, 4]; [7, 7, 4]; [7, 7, 5]; ...
    [7, 8, 5]; [8, 8, 5]; [8, 8, 6]; [8, 9, 6]; [9, 9, 6]; [9, 9, 7]; [9, 10, 7]; [10, 10, 7]; ...
    [10, 10, 8]; [10, 11, 8]; [11, 11, 8]; [11, 11, 9]; [11, 12, 9]; [12, 12, 9]; [12, 12, 10]; ...
    [12, 13, 10]; [13, 13, 10]; [13, 13, 11]; [13, 14, 11]; [14, 14, 11]; [14, 14, 12]; [14, 15, 12]; ...
    [15, 15, 12]; [15, 15, 13]; [15, 16, 13]; [16, 16, 13]; [16, 16, 14]; [16, 17, 14]; [17, 17, 14]; ...
    [17, 17, 15]; [17, 18, 15]; [18, 18, 15]];

grid = zeros(20, 20, 20);  % 20x20 grid, all free space initially
% Add some obstacles
grid(5:15, 10, 1:6) = 1;  % Vertical wall
grid(5:15, 10, 11:18) = 1;  % Vertical wall
grid(5, 6:15, 6:15) = 1;   % Horizontal wall

% Find the coordinates of the obstacles
[rows, cols, heights] = ind2sub(size(grid), find(grid == 1));

% Plot obstacles using scatter3 in black
scatter3(cols, rows, heights, 250, 'k', 'filled','o'); hold on;
% Plot the path with red color
% scatter3(result_path(:,2), result_path(:,1), result_path(:,3), 50, 'r', 'filled');
% Plot the path with red color and connect the points with a line
plot3(result_path(:,2), result_path(:,1), result_path(:,3), 'r-', 'LineWidth', 2);

xlabel('X');
ylabel('Y');
zlabel('Z');
% axis equal;
xlim([0, 20])
ylim([0, 20])
zlim([0, 20])
title('3D Plot of Obstacles and Path');

% % Plot the points in 3D with color according to the 4th column
% figure;
% h = scatter3(x, y, z, 50, color, 'filled');  % 50 is the marker size
% colormap('turbo(20)');  % You can replace 'jet' with any other colormap like 'parula', 'cool', 'hot', etc.
% colorbar;  % Optional: to show a color scale
% axis equal
% alpha = 0.5;
% % set(h, 'MarkerEdgeAlpha', alpha, 'MarkerFaceAlpha', alpha)    # activate just when checking, it takes a lot of resources







