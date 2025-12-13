function interpolatedPlan=exampleHelperVariableHeterogeneousPlanner(start,goal,geomStructs)
%This function is for internal use only and may be removed in the future.
%exampleHelperVariableHeterogeneousPlanner Plan in a perceived environment of a Kinova Gen 3 robot
%   INTEPROLATEDPLAN=exampleHelperVariableHeterogeneousPlanner(START,GOAL,GEOMSTRUCTS)
%   Outputs a collision free geometric plan, INTEPROLATEDPLAN, of a Kinova
%   Gen3 robot in an environment defined by GEOMSTRUCTS which is a variable
%   sized array of struct elements that capture the information of a
%   collision geometry in the environment. Each struct element in
%   GEOMSTRUCTS is of the form:
%       geomStruct=struct("Type", exampleHelperCollisionEnum.Box, ...
%                 "X", 0, ...
%                 "Y", 0, ...
%                 "Z", 0, ...
%                 "Vertices", zeros(3),...
%                 "Radius", 0, ...
%                 "Height", 0, ...
%                 "Pose", eye(4));
%   START and GOAL are the start and goal joint configurations of the
%   robot, respectively, and are specified as a row vector.

%Copyright 2021-2023 The MathWorks, Inc.
    
    % Create a placeholder variable for environment which is a cell-array
    % of collision meshes with vertices that are variably sized. The size
    % of the environment is that of the input array of collision geometry
    % struct elements.
    coder.varsize('vertices',[inf,3],[1,0]);
    vertices=zeros(3);
    env=repmat({collisionMesh(vertices)},1,length(geomStructs));

    % Load the rigid body tree for which the planner will be defined and
    % make it persistent. This improves the speed of the mexed function,
    % since the robot need only be instantiated once.
    persistent rbt  
    if isempty(rbt)
        rbt=loadrobot("kinovaGen3",DataFormat="row");
    end

    % Set up the environment. The maximum number of collision objects that
    % the environment can hold is 100.
    
    for i=coder.unroll(1:100)
        if(i<=length(geomStructs))

            % For every struct element, create the corresponding collision
            % object (collisonBox, collisionCylinder, collisionSphere, or
            % collisionMesh) and convert that to its corresponding mesh
            % equivalent thereby homogenizing the environment.
            s=geomStructs(i);
            if(s.Type==exampleHelperCollisionEnum.Box)
                env{i}=convertToCollisionMesh(...
                    collisionBox(s.X,s.Y,s.Z));
            elseif(s.Type==exampleHelperCollisionEnum.Sphere)
                env{i}=convertToCollisionMesh(...
                    collisionSphere(s.Radius));
            elseif(s.Type==exampleHelperCollisionEnum.Cylinder)
                env{i}=convertToCollisionMesh(...
                    collisionCylinder(s.Radius,s.Height));
            elseif(s.Type==exampleHelperCollisionEnum.Mesh)
                env{i}=collisionMesh(s.Vertices);
            end

            % Assign the pose of the element.
            env{i}.Pose=s.Pose;
        end
    end

    % Create and set up the planner from the rigid body tree and
    % environment.
    planner=manipulatorRRT(rbt,env);
    planner.SkippedSelfCollisions="parent";
    planner.MaxConnectionDistance=0.4;
    planner.ValidationDistance=0.05;

    % For repeatable results, seed the random number generator and store
    % the current seed value.
    prevseed=rng(0);

    % Plan and interpolate.
    planOut=planner.plan(start,goal);
    interpolatedPlan=planner.interpolate(planOut);

    % Restore the random number generator to the previously stored seed.
    rng(prevseed);
end
