function snakePlanarPlot(theta,d,TargetPose,figNum,obstacle)
figure(figNum); hold on; grid on; axis equal;

% Fwd Kine
[~,positions] = snakePlanarFwdKine(d, theta);

% Plot links
plot(positions(1, :), positions(2, :), '-o', 'LineWidth', 2, 'MarkerSize', 8);

% Highlight origin
plot(positions(1,1), positions(2,1), 'k.', 'MarkerSize', 30);

% Desired pose
plot(TargetPose(1,3), TargetPose(2,3), 'r.', 'MarkerSize', 30);

if exist('obstacle', 'var')
    for i = 1:length(obstacle)
        center = obstacle(i).position;
        radius = obstacle(i).radius;
        position = [center(1) - radius, center(2) - radius, radius * 2, radius * 2];
        rectangle('Position', position, 'Curvature', [1,1]);
    end
end

xlabel('X'); ylabel('Y');
title('Planar Manipulator');
hold off

end