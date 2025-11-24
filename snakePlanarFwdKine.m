function [T, positions] = snakePlanarFwdKine(d, theta)
n = length(theta);

T = eye(3);

% Store joint positions (including base at [0,0])
positions = zeros(2, n+1);
positions(:,1) = [0; 0];

for i = 1:n
    % Compute local transform for each link
    T_this = [cos(theta(i)), -sin(theta(i)), d(i)*cos(theta(i));
              sin(theta(i)),  cos(theta(i)), d(i)*sin(theta(i));
              0,              0,             1];

    T = T * T_this;  % accumulate transformation

    % Extract x,y position of this joint
    positions(:, i+1) = T(1:2, 3);
end
end