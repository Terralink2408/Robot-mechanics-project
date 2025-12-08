
%gen3 = importrobot('kinova_gen3.urdf', 'DataFormat','row');
%robot = gen3;


%test trajectory
nSteps = 400;

t = linspace(0, 2*pi, nSteps)';  % base time parameter

% Smooth joint motions using sinusoids
qTraj = zeros(nSteps,7);         % preallocate

qTraj(:,1) = 0.6*sin(t);                 % base rotates left ↔ right
qTraj(:,2) = 0.4*sin(2*t + pi/4);        % shoulder lift
qTraj(:,3) = 0.5*sin(1.5*t + pi/2);      % elbow forward/back
qTraj(:,4) = 0.6*sin(2*t);               % wrist roll
qTraj(:,5) = 0.5*sin(1.2*t + 1);         % wrist pitch
qTraj(:,6) = 0.4*sin(2.5*t);             % wrist yaw
qTraj(:,7) = 0.8*sin(0.8*t + 0.6*pi);    % last joint swings nicely

traj = [];
nJoints = 7;

N = size(traj, 1);


figure('Name','Kinova Animation');
ax = axes;
show(robot, qTraj(1,:), 'Parent', ax, 'Visuals','on');
axis equal
view(135, 20);          % nice 3D view
xlim([-1 1]); ylim([-1 1]); zlim([0 1.5]);  % adjust to your arm size
hold on

for k = 1:nSteps
    show(robot, qTraj(k,:), ...
         'Parent', ax, ...
         'Visuals', 'on', ...
         'PreservePlot', false);  % overwrite previous pose
    drawnow;                      % update the figure
    pause(0.03);                  % adjust speed (seconds per frame)
end