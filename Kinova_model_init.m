

%
dhparams = [ ...
    0    pi/2    0.1284   pi;   % Joint 1
    0    pi/2    0.2104   pi;   % Joint 2
    0    pi/2    0.2104   pi;   % Joint 3
    0    pi/2    0.2084   pi;   % Joint 4
    0    pi/2    0.1059   pi;   % Joint 5
    0    pi/2    0        pi;   % Joint 6
    0    pi      0.0615   pi];  % Joint 7


%robot = rigidBodyTree;
robot = rigidBodyTree('DataFormat','row','MaxNumBodies',7); %for animation


bodies = cell(6,1);
joints = cell(6,1);

jointRadius = 0.03;   % size of the "ball" at each joint
linkRadius  = 0.015;  % thickness of link cylinders
for i = 1:7
    bodies{i} = rigidBody(['body' num2str(i)]);
    joints{i} = rigidBodyJoint(['jnt' num2str(i)],"revolute");
    setFixedTransform(joints{i},dhparams(i,:),"dh");
    bodies{i}.Joint = joints{i};

    linkRadius = 0.03;
    linkLength = max(0.05, abs(dhparams(i,1)) + abs(dhparams(i,3))); 
    
    
    % ==== Visuals ====

    % joint sphere
    addVisual(bodies{i}, "Sphere", jointRadius);

    % Cylinder along link
    a_i = dhparams(i,1);
    d_i = dhparams(i,3);

    linkLength = abs(a_i) + abs(d_i);
    if linkLength < 0.03
        linkLength = 0.03;   % minimum so it's visible
    end

    % Place cylinder along +z of the joint frame, centered between joints
    % (default cylinder axis is z)
    T_link = trvec2tform([0 0 linkLength/2]);

    addVisual(bodies{i}, "Cylinder", [linkRadius, linkLength], T_link);

    % ==================

    if i == 1 % Add first body to base
        addBody(robot,bodies{i},"base")
    else % Add current body to previous body by name
        addBody(robot,bodies{i},bodies{i-1}.Name)
    end
end

showdetails(robot)
figure(Name="Kinova Robot Model")
show(robot);