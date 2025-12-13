
%% Set Up of parameters
% DH parameters:  [theta_i   d_i (m)           a_i (m)  alpha_i (rad)]
DH = [0, 0.0, 0.0, pi;      % i = 0 (from base)
    0, -(0.1564 + 0.1284), 0.0, pi/2;    % i = 1
    0, -(0.0054 + 0.0064), 0.0, pi/2;    % i = 2
    0, -(0.2104 + 0.2104), 0.0, pi/2;    % i = 3
    0, -(0.0064 + 0.0064), 0.0, pi/2;    % i = 4
    0, -(0.2084 + 0.1059), 0.0, pi/2;    % i = 5
    0, 0.0, 0.0, pi/2;    % i = 6
    0, -(0.1059 + 0.0615), 0.0, pi];     % i = 7 (to interface)

massList = [1.697,1.377,1.1636,1.1636,0.930,0.678,0.678,0.364];
offsetList = [0,0,-pi,-pi,-pi,-pi,-pi,-pi];

comList = [[-0.000648, -0.000166, 0.084487];
            [-0.000023, -0.010364, -0.073360];
            [-0.000044, -0.099580, -0.013278];
            [-0.000044, -0.006641, -0.117892];
            [-0.000018, -0.075478, -0.015006];
            [0.000001, -0.009432, -0.063883];
            [0.000001, -0.045483, -0.009650];
            [-0.000093, 0.000132, -0.022905]];

inertiaList = [
    [0.004622 0.000009 0.000060;
    0.000009 0.004495 0.000009;
    0.000060 0.000009 0.002079];

    [ 0.004570   0.000001   0.000002;
      0.000001   0.004831   0.000448;
      0.000002   0.000448   0.001409 ];

    [ 0.011088   0.000005    0.000000;
      0.000005   0.001072   -0.000691;
      0.000000  -0.000691    0.011255 ];

    [ 0.010932   0.000000  -0.000007;
      0.000000   0.011127   0.000606;
     -0.000007   0.000606   0.001043 ];

    [ 0.008147   -0.000001  0.000000;
      -0.000001   0.000631   -0.000500;
     0.000000   -0.000500   0.008316 ];

    [ 0.001596   0.000000  0.000000;
      0.000000   0.001607   0.000256;
     0.000000   0.000256   0.000399 ];

    [ 0.001641   0.000000  0.000000;
      0.000000   0.000410   -0.000278;
     0.000000   -0.000278   0.001641 ];

    [ 0.000214   0.000000  0.000001;
      0.000000   0.000223   -0.000002;
     0.000001   -0.000002   0.000240 ]];


numLinks = size(DH, 1);   % should be 9 rows in your doc, 8 in your DH

clear linkList
for i = 1:numLinks
    a     = DH(i, 3);
    d     = DH(i, 2);
    alpha = DH(i, 4);

    % -------- Determine if this is a joint or static link ----------
    % MATLAB row numbers:
    % i = 1 → DH row 0 (static)
    % i = 2..8 → DH rows 1..7 (rotary joints)
    % i = 9 → interface (static)

    if i == 1
        % STATIC LINK (base or interface)
        theta = 0   % document θ0 = 0, θ8 = π
        isR   = -1;         % static
    else
        % ROTARY JOINT
        theta = [];         % empty → treated as rotary
        isR   = 1;
    end

    % ------------- Build the link --------------------
    linkList(i) = createLink(a, d, alpha, theta, offsetList(i), comList(i),massList(i), inertiaList(i));

    % enforce isRotary flag
    linkList(i).isRotary = isR;
end



%% Set up Environment
kinova = loadrobot("kinovaGen3",DataFormat="row");
geomStructType = struct("Type",exampleHelperCollisionEnum.Box, ...
                  "X",0, ...
                  "Y",0, ...
                  "Z",0, ...
                  "Vertices",coder.typeof(zeros(3),[inf 3],[1 0]), ...
                  "Radius",0, ...
                  "Height",0, ...
                  "Pose",eye(4));
geomStruct = geomStructType;
geomStruct.Vertices=zeros(3);

%% Floor
%{
boxGeom = geomStruct;
boxGeom.Type = exampleHelperCollisionEnum.Box;
boxGeom.X = 1;
boxGeom.Y = 1;
boxGeom.Z = 0.1;
boxGeom.Pose = trvec2tform([0 0 -0.051]);
%}

%% Object Set up
sphereGeom = geomStruct;
sphereGeom.Type = exampleHelperCollisionEnum.Sphere;
sphereGeom.Radius = 0.125;
sphereGeom.Pose = trvec2tform([0.2 0.0 0.20]);

env1 = [sphereGeom];


%% Desired transform
%[desTransform,~] = dhFwdKine(linkList, paramList); % Forward kinematics for desired transform

%% Path Planning and Visualization
desTransform = [1 0 0 0.4;
                0 1 0 0;
                0 0 1 0.25
                0 0 0 1];

paramList = [0, pi/4, 0, -pi/2, 0, 0, pi/2]; % Random paramList

[thetaObs,errorObs] = dhInvKineObs(linkList, desTransform, paramList, sphereGeom);
computeDes = dhFwdKine(linkList, thetaObs)

planInEnv1 = exampleHelperVariableHeterogeneousPlanner(paramList,thetaObs,env1)

[final_pos, ~] = dhFwdKine(linkList, planInEnv1(end,:));

figure(Name="Planned path in env1",Visible="on",Units="normalized",OuterPosition=[0,0,1,1])
HelperVisualizeEnvironment(env1,kinova,planInEnv1,final_pos);

%[q, qd, qdd] = trapveltraj(planInEnv1', size(planInEnv1,1))
