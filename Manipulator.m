%% simple6DOF_demo.m
% Demo of a simple 7-DOF manipulator using rigidBodyTree

clear; clc; close all;

% Build the robot
robot = buildSimple7DOF();
robot.DataFormat = 'row';   % joint config as row vectors

% Home configuration (all zeros)
q_home = zeros(1, 7);

% Visualize at home pose
figure;
show(robot, q_home, 'Frames', 'off', 'PreservePlot', false);
axis equal;
view(135, 20);
title('Simple 6-DOF Manipulator (Home Configuration)');

% Forward kinematics: get transform from base to end-effector
T_0_6 = getTransform(robot, q_home, 'link6', robot.BaseName);
disp('End-effector pose at home configuration (T_0_6):');
disp(T_0_6);

% Geometric Jacobian at end-effector
J_0_6 = geometricJacobian(robot, q_home, 'link6');
disp('Geometric Jacobian at home configuration (base frame):');
disp(J_0_6);

% Show a random configuration
q_rand = randomConfiguration(robot);   % Already a numeric row vector

figure;
show(robot, q_home, 'Frames', 'on', 'PreservePlot', false);
axis equal;
view(135, 20);
title('Simple 6-DOF Manipulator (Home Configuration)');

% Random config
q_rand = randomConfiguration(robot);
figure;
show(robot, q_rand, 'Frames', 'on', 'PreservePlot', false);
axis equal;
view(135, 20);
title('Simple 6-DOF Manipulator (Random Configuration)');


%% Function: buildSimple7DOF
function robot = buildSimple7DOF()

    % Create empty rigidBodyTree
    robot = rigidBodyTree('DataFormat', 'row', 'MaxNumBodies', 7);

    % Standard DH parameters: [a, alpha, d, theta]
    % You can modify these to match whatever robot you like.
    % Here we just make something kinematically reasonable.
    DH =  [ ...
    0      pi      0         -1;
    0      pi/2   -0.2848     0;
    0      pi/2   -0.0118     0;
    0      pi/2   -0.4208     0;
    0      pi/2   -0.0128     0;
    0      pi/2   -0.3143     0;
    0      pi/2    0          0;
    0      pi     -0.1674     0;
];



    % Build each link/joint and add to tree
    prevBodyName = robot.BaseName;
    
    N = 7;
    for i = 1:N
        body = rigidBody(sprintf('link%d', i));
        joint = rigidBodyJoint(sprintf('joint%d', i), 'revolute');

        % Set DH transform 
        setFixedTransform(joint, DH(i, :), 'dh');

        % joint limits
        joint.PositionLimits = [-pi, pi];

        body.Joint = joint;

        % Add body to robot
        addBody(robot, body, prevBodyName);
        prevBodyName = body.Name;
    end

end
