% dhFwdKine: Computes the forward kinematics of a manipulator using a
% provided set of DH parameters.
%
% H = dhFwdKine(linkList, paramList)
% This function calculates the homogeneous transformation matrix describing
% the pose of the manipulator's end effector relative to the base frame,
% using the DH parameters and current joint variable states.
%
% H = 4x4 homogeneous transformation matrix representing the end effector
%     position and orientation relative to the base frame.
%
% linkList = Array of link structures, each created using createLink().
% paramList = Array containing the current state of the joint variables
%              (radians for rotary joints or meters for prismatic joints),
%              as reported by the robot's encoders.
%
% Noah Kelly
% 10884361
% MEGN544
% 11/10/2025
function [H, positions] = dhFwdKine(linkList, paramList)
n = length(linkList);

% Preallocate positions (base at column 1)
positions = zeros(3, n+1);
positions(:,1) = [0;0;0];

% Initialize the transformation matrix as the identity matrix
H = eye(4);

% Loop through each link to compute the transformation matrix
for i = 1:n
    switch linkList(i).isRotary
        case 0 % prismatic
            T = dhTransform(linkList(i).a, paramList(i) - linkList(i).offset, linkList(i).alpha, linkList(i).theta);
        case 1 % rotary
            T = dhTransform(linkList(i).a, linkList(i).d, linkList(i).alpha, paramList(i) - linkList(i).offset);
        case -1 % static
            T = dhTransform(linkList(i).a, linkList(i).d, linkList(i).alpha, linkList(i).theta);
    end
    H = H*T;
    positions(:, i+1) = H(1:3,4);
end
end