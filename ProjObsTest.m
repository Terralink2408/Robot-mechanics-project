clear all; clc
%% Desired transform
numLinks = 3;
linkLength = 5/numLinks; % Manipulator total length of 5 m 
d = linkLength*ones(numLinks,1); % Equal length links
paramList = [pi/6, -pi/12, pi/12]; % Random paramList
for i = 1:numLinks
    linkList(i) = createLink(linkLength, 0, 0, [], 0, 0, 0, 0);
end
[desTransform,~] = dhFwdKine(linkList, paramList); % Forward kinematics for desired transform
desTransform_33 = [desTransform(1,1:2), desTransform(1,4); desTransform(2,1:2), desTransform(2,4); 0 0 1]; % Resize to 3x3 for planar plot

%% Inverse Kine: No obstacle
theta = dhInvKineObs(linkList, desTransform, paramList); % Inv kine
snakePlanarPlot(theta, d, desTransform_33, 1) % Planar plot

%% Inverse Kine: Obstacle
clear obstacle
obstacle(1).position = [1,1,0]; % Obstacle 1
obstacle(1).radius = 0.35;
obstacle(2).position = [3,1,0]; % Obstacle 2
obstacle(2).radius = 0.35;
tic; % Function timing for testing
[thetaObs,errorObs] = dhInvKineObs(linkList, desTransform, paramList, obstacle);
elapsedTime = toc;
obstacle(3).position = obstacle(1).position; % "Safe zone" around obstacle
obstacle(3).radius = obstacle(1).radius + 0.05;
obstacle(4).position = obstacle(2).position;
obstacle(4).radius = obstacle(2).radius + 0.05;
snakePlanarPlot(thetaObs, d, desTransform_33, 1, obstacle); % Planar plot with obstacles

%% Spatial Example
spatialManipulatorPlot(linkList, theta, desTransform, 2)
spatialManipulatorPlot(linkList, thetaObs, desTransform, 2, obstacle)

%% EVERYTHING BELOW FOR TESTING/GAIN TUNING
%{
%% K and points test
k = linspace(0,1,11);
pts = linspace(0,50,11);
for i = 1:length(k)
    for j = 1:length(pts)
        tic;
        [thetaObs,errorObs(i,j)] = dhInvKineObs(linkList, desTransform, paramList, obstacle, k(i), pts(j));
        lapsedTime(i,j) = toc;
        [~,pos] = dhFwdKine(linkList, thetaObs);
        [isOverlap1, penetration1] = lineCircleOverlap(pos(1:2,1), pos(1:2,2), obstacle_buf(1));
        [isOverlap2, penetration2] = lineCircleOverlap(pos(1:2,2), pos(1:2,3), obstacle_buf(2));
        overlap(i,j) = isOverlap1 + isOverlap2;
        penetration(i,j) = penetration1 + penetration2;
    end
end

%% Change numLinks

numLinks = 4;
linkLength = 5/numLinks;
d = linkLength*ones(numLinks,1);
paramList = zeros(numLinks,1);
paramList(1) = atan2(desTransform(2,4),desTransform(1,4));
for i = 1:numLinks
    linkList(i) = createLink(linkLength, 0, 0, [], 0, 0, 0, 0);
end
[thetaNew,errorNew] = dhInvKineObs(linkList, desTransform, paramList);
snakePlanarPlot(thetaNew, d, desTransform_33, 2);

clear obstacle
obstacle(1).position = [1,1,0];
obstacle(1).radius = 0.35;
obstacle(2).position = [3,1,0];
obstacle(2).radius = 0.35;
[thetaNewObs,errorNewObs] = dhInvKineObs(linkList, desTransform, paramList, obstacle)
obstacle(3).position = obstacle(1).position;
obstacle(3).radius = obstacle(1).radius + 0.05;
obstacle(4).position = obstacle(2).position;
obstacle(4).radius = obstacle(2).radius + 0.05;
snakePlanarPlot(thetaNewObs, d, desTransform_33, 2, obstacle);


%%
function [isOverlap, penetration] = lineCircleOverlap(A, B, obstacle)

    C = obstacle.position(1:2)';
    R = obstacle.radius;

    % Vector from A to B
    AB = B - A;
    % Vector from A to center
    AC = C - A;

    % Project AC onto AB to get closest point parameter t (0–1 for segment)
    t = dot(AC, AB) / dot(AB, AB);

    % Clamp t to stay on segment
    t = max(0, min(1, t));

    % Closest point on the segment to circle center
    closestPoint = A + t * AB;

    % Distance from obstacle center to closest point on segment
    dist = norm(closestPoint - C);

    % Check overlap
    isOverlap = dist < R;

    % Penetration depth (0 if not overlapping)
    penetration = max(0, R - dist);
end
%}