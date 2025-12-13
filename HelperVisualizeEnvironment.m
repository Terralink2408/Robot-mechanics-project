function HelperVisualizeEnvironment(env,robot,plannedpath,final_pos)
%
% 
% 
% Function modiefied from:
%exampleHelperVisualizeVarSizeEnvironment Visualize the planned path of the robot in the environment

%Copyright 2021-2023 The MathWorks, Inc.
fig = figure('Color','w');
fig.Position = [100 100 1920 1080];

hold on;
for i=1:length(env)
    geom=env(i);
    if(geom.Type==exampleHelperCollisionEnum.Box)
        c=collisionBox(geom.X,geom.Y,geom.Z);
    elseif(geom.Type==exampleHelperCollisionEnum.Sphere)
        c=collisionSphere(geom.Radius);
    elseif(geom.Type==exampleHelperCollisionEnum.Cylinder)
        c=collisionCylinder(geom.Radius,geom.Height);
    end
    c.Pose=geom.Pose;
    show(c);
end
light;
ax = gca;
axis('equal')
ax.XLim=[-0.5,0.5];
ax.YLim=[-0.5,0.5];
ax.ZLim=[-0.5,1.0];
view([0, 90])

%set(ax, 'Units','normalized','Position',[0 0 1 1]);


duration = 5;
numConfigs = size(plannedpath, 1);
maxFrameRate = 60;
rawFrameRate = numConfigs/duration;

if rawFrameRate <= maxFrameRate
    frameRate = rawFrameRate;
    step = 1;
else
    frameRate = maxFrameRate;
    step = ceil(numConfigs / (desiredDuration*frameRate));
end


v = VideoWriter('robotPath.mp4','MPEG-4');
v.FrameRate = frameRate;
v.Quality = 95;
open(v);

for k=1:step:numConfigs
    show(robot,plannedpath(k,:),...
        "PreservePlot",false,...
        "FastUpdate",true);
    drawnow;

    % --- Mark goal position ---
    endPos = final_pos(1:3,4);

    plot3(endPos(1),endPos(2),endPos(3),'mo', ...
        'MarkerSize',12,'MarkerFaceColor','m');

    frame = getframe(fig);
    writeVideo(v, frame);
end

close(v);
hold off;
disp('Saved video: robotPath.mp4');
end
