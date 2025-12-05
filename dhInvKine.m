% dhInvKine: Computes inverse kinematics for a manipulator described by DH parameters.
%
% [paramList, error] = dhInvKine(linkList, desTransform, paramListGuess)
% This function computes the parameter list (joint variables) required for
% the manipulator to achieve a desired end-effector homogeneous transform.
% It uses the provided initial guess to iterate toward a solution and
% returns both the resulting joint parameters and the residual pose error.
%
% paramList  = N×1 vector of joint parameters (θ or d) that achieve the
%              target transform, wrapped to [-pi, pi] for rotary joints
% error      = residual error between the achieved transform and the
%              desired transform, expressed as a scalar
%
% linkList        = list of joint/link structures created by createLink
% desTransform    = 4×4 desired homogeneous transform of the end-effector
% paramListGuess  = N×1 initial guess for the joint parameters, based on
%                   the robot's encoder conventions
%
% Noah Kelly
% 10884361
% MEGN544
% 11/24/2025
function [paramList, error] = dhInvKine(linkList, desTransform, paramListGuess)
tol = 1e-10;
iter = 1;
paramList = paramListGuess;

[curTransform,~] = dhFwdKine(linkList, paramList);
error = transformError(desTransform, curTransform);

while norm(error) > tol && iter < 100
    [J, ~] = velocityJacobian(linkList,paramList);
    J_inv = jacobianSVDInv(J);
    JinvE = J_inv*error;
    if abs(JinvE) < tol % Check J_inv * error
        return
    else
        paramList = paramList + JinvE; % Update paramList
    end
    
    curTransform = dhFwdKine(linkList, paramList); % Update current transform
    error = transformError(desTransform, curTransform); % Update error
    iter = iter + 1;
end

for i = 1:length(paramList)
    if linkList(i).isRotary == 1 % Rotary joint only
        paramList(i) = wrapToPi(paramList(i)); % Wrap joint parameters to [-pi, pi]
    end
end

error = norm(error);

    function error = transformError(desTransform, curTransform)
    transError = desTransform(1:3,4) - curTransform(1:3,4);
    rotError = rot2AngleAxis(desTransform(1:3,1:3)*(curTransform(1:3,1:3)'));
    error = [transError; rotError];
    end

    function J_inv = jacobianSVDInv(J)
    [U, S, V] = svd(J);
    D = diag(S);
    S_pinv = zeros(size(S'));
    for j = 1:length(D)
        if max(D)/D(j) <= 500
            S_pinv(j,j) = 1 / D(j);
        else
            S_pinv(j,j) = 0;
        end
    end
    J_inv = V*S_pinv*U';
    end

end