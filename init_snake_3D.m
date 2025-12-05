%% 3D Snake test (2 DOF per link: yaw + pitch)

numModules = 8;
linkLength = 0.15;

snake3D = buildSnakeRobot3D(numModules, linkLength);
q = zeros(1, snake3D.NumBodies);   % 2 DOF per module -> 16 joints

% Neutral pose
figure;
show(snake3D, q);
axis equal;
title('3D snake (neutral)');

% Wiggle pose in 3D
yawIdx   = 1:2:snake3D.NumBodies;   % joints 1,3,5,... (yaw)
pitchIdx = 2:2:snake3D.NumBodies;   % joints 2,4,6,... (pitch)

q = zeros(1, snake3D.NumBodies);
q(yawIdx)   = 0.4 * sin((1:numModules) * 0.5);
q(pitchIdx) = 0.3 * cos((1:numModules) * 0.5);

figure;
show(snake3D, q);
axis equal;
title('3D snake (wiggle pose)');


%% Rigidbodytree stuff
function snake = buildSnakeRobot3D(numModules, linkLength)
    % 3D snake: each module has two revolute joints:
    %   - yaw (about z)
    %   - pitch (about y) + linkLength translation along x

    snake = rigidBodyTree('DataFormat','row', ...
                          'MaxNumBodies', 2*numModules);

    parent = snake.BaseName;  

    % simple link properties
    radius = 0.03;                 % 3 cm radius
    mass   = 0.2;                  % 200 g per link (rough guess)

    for i = 1:numModules
        % ----- Yaw joint body -----
        yawBodyName  = sprintf('yawBody_%d', i);
        yawJointName = sprintf('yawJoint_%d', i);

        yawBody  = rigidBody(yawBodyName);
        yawJoint = rigidBodyJoint(yawJointName, 'revolute');
        yawJoint.JointAxis = [0 0 1];      % yaw about z
        setFixedTransform(yawJoint, eye(4));
        yawBody.Joint = yawJoint;

        % (optional) very small mass so it's not completely massless
        yawBody.Mass = 0.01;
        yawBody.CenterOfMass = [0 0 0];
        yawBody.Inertia = [1e-4 1e-4 1e-4 0 0 0];

        addBody(snake, yawBody, parent);

        % ----- Pitch joint body (with link length) -----
        pitchBodyName  = sprintf('pitchBody_%d', i);
        pitchJointName = sprintf('pitchJoint_%d', i);

        pitchBody  = rigidBody(pitchBodyName);
        pitchJoint = rigidBodyJoint(pitchJointName, 'revolute');
        pitchJoint.JointAxis = [0 1 0];    % pitch about y

        % Transform from yaw body to pitch body:
        % translate along x by linkLength
        T = [1 0 0 linkLength;
             0 1 0 0;
             0 0 1 0;
             0 0 0 1];
        setFixedTransform(pitchJoint, T); %This thing is straght from Chat
        pitchBody.Joint = pitchJoint;

        % ---- Add simple cylinder visual along x axis ----
        % cylinder of length linkLength, centered halfway along x
        Tvis = trvec2tform([linkLength/2 0 0]);
        addVisual(pitchBody, 'Sphere', radius, Tvis);

        % ---- Add simple inertial properties ----
        pitchBody.Mass = mass;
        pitchBody.CenterOfMass = [linkLength/2 0 0];
        % rough diagonal inertia for a slender rod (not critical)
        I = 1e-3;
        pitchBody.Inertia = [I I I 0 0 0];

        addBody(snake, pitchBody, yawBodyName);

        parent = pitchBodyName;
    end
end

%exportrobot(snake3D, 'Snake_model', 'snake3D.urdf');

exportrobot(snake3D,OutputFileName="snake3D.urdf",ExportMesh=true);