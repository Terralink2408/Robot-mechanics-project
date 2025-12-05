function spatialManipulatorPlot(linkList, paramList, TargetPose, figNum, obstacle)
figure(figNum); hold on; grid on; axis equal;

% Forward Kinematics
[~, positions] = dhFwdKine(linkList, paramList);
% positions is 3×(n+1)

% Plot links
plot3(positions(1,:), positions(2,:), positions(3,:), ...
      '-o', 'LineWidth', 2, 'MarkerSize', 8);

% Highlight base
plot3(positions(1,1), positions(2,1), positions(3,1), ...
      'k.', 'MarkerSize', 30);

% Desired end pose
if exist('TargetPose','var') && ~isempty(TargetPose)
    plot3(TargetPose(1,4), TargetPose(2,4), TargetPose(3,4), ...
          'r.', 'MarkerSize', 30);
end

% Obstacle (sphere)
if exist('obstacle','var') && ~isempty(obstacle)
    for o = 1:length(obstacle)
        cx = obstacle(o).position(1); cy = obstacle(o).position(2); cz = obstacle(o).position(3);
        r  = obstacle(o).radius;

        [Xs, Ys, Zs] = sphere(20);
        surf(r*Xs + cx, r*Ys + cy, r*Zs + cz, ...
            'FaceAlpha', 0.2, 'EdgeColor','none');
    end
end

xlabel('X'); ylabel('Y'); zlabel('Z');
title('Spatial Manipulator');
view(3); % good 3D angle
hold off;

end
