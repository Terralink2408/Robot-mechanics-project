function snakePlanarPlot(th1,th2,th3,d,TargetPose,figNum)
figure(figNum); hold on; grid on; axis equal;

% Fwd Kine
[~,positions] = snakePlanarFwdKine(d, [th1; th2; th3]);

% Plot links
plot(positions(1, :), positions(2, :), '-o', 'LineWidth', 2, 'MarkerSize', 8);

% Highlight origin
plot(positions(1,1), positions(2,1), 'k.', 'MarkerSize', 30);

% Desired pose
plot(TargetPose(1,3), TargetPose(2,3), 'r.', 'MarkerSize', 30);

xlabel('X'); ylabel('Y');
title('Planar Manipulator');
hold off

end