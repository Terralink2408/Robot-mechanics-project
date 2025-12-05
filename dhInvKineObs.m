% dhInvKineObs: Inverse kinematics with obstacle avoidance for a DH-parameterized manipulator.
%
% [paramList, error] = dhInvKineObs(linkList, desTransform, paramListGuess, obstacle)
% This function computes the joint parameter vector that best achieves a
% desired end-effector homogeneous transform while simultaneously avoiding
% spherical obstacles in the workspace. The solution is obtained using
% constrained nonlinear optimization (fmincon), combining an attractive
% cost driving the robot toward the target pose and a repulsive cost that
% penalizes proximity to obstacles. Joint limits are enforced explicitly,
% and the resulting IK solution is returned along with a scalar error
% measuring the residual pose mismatch.
%
% paramList = N×1 optimized joint parameters (θ or d), wrapped to [−π, π]
% error     = scalar norm of the 6×1 pose error (position + angle-axis)
%
% linkList        = list of link structures created by createLink
% desTransform    = 4×4 desired homogeneous transform of the end-effector
% paramListGuess  = N×1 initial guess for the joint parameters
% obstacle        = array of obstacle structs, each with fields:
%                     • position (3×1)
%                     • radius   (scalar)
%
% Noah Kelly
% 10884361
% MEGN544
% 12/5/2025
function [paramList,error] = dhInvKineObs(linkList, desTransform, paramListGuess, obstacle)

n = length(paramListGuess);
thMin = -pi*ones(n,1);
thMax = pi*ones(n,1);

costFun = @(th) totalCost(th);

% optimization
options = optimoptions('fmincon');

paramList = fmincon(costFun, paramListGuess, [], [], [], [], thMin, thMax, [], options);

[curTransform,~] = dhFwdKine(linkList, paramList);
error = transformError(desTransform, curTransform);
error = norm(error);

    function C = totalCost(th)
        % Attractive cost
        [curTransform,~] = dhFwdKine(linkList, th);
        transError = transformError(desTransform, curTransform);
        C_target = norm(transError)^2; % Cost function as the squared norm of the error

        % Repulsive cost
        C_obs = obstacleCost(th);

        C = C_target + C_obs; % Combine attractive and repulsive costs
    end

    function error = transformError(desTransform, curTransform)
        transError = desTransform(1:3,4) - curTransform(1:3,4);
        rotError = rot2AngleAxis(desTransform(1:3,1:3)*(curTransform(1:3,1:3)'));
        error = [transError; rotError];
    end

    function C_obs = obstacleCost(th)
        C_obs = 0;
        k = 0.3; % Gain: to be tuned
        pts = sampleManipulator(th);
        
        if exist('obstacle', 'var')
            for o = 1:length(obstacle)
                d0 = obstacle(o).radius + 0.05; % Safe distance around obstacle: to be tuned

                for i = 1:size(pts,1)
                    d = norm(pts(i, :) - obstacle(o).position);
                    if d < d0
                        C_obs = C_obs + k*(1/d - 1/d0)^2; % Increase cost as the distance decreases
                    end
                end
            end
        end
    end

    function pts = sampleManipulator(th)
        k = 10; % number of pts per link
        [~,positions] = dhFwdKine(linkList, th);
        pts = [positions(1,1), positions(2,1), positions(3,1)];
        for i = 1:length(linkList)
            pt = [linspace(positions(1,i), positions(1,i+1), k)', ...
                  linspace(positions(2,i), positions(2,i+1), k)', ...
                  linspace(positions(3,i), positions(3,i+1), k)'];
            pts = [pts; pt(2:end,:)]; % Append the sampled points to the list
        end
    end
end