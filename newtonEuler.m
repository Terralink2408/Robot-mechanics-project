% newtonEuler: Computes the inverse dynamics joint torques of a serial-link manipulator.
%
% jointTorques = newtonEuler(linkList, paramList, paramListDot, paramListDDot, boundry_conditions)
% This function implements the Newton–Euler recursive algorithm to compute
% the joint torques (or forces, for prismatic joints) required to achieve
% the specified joint motion. The algorithm performs an outward scan to
% propagate velocities and accelerations and an inward scan to propagate
% forces and moments, incorporating both motion-induced and external loads.
%
% jointTorques = N×1 vector of required joint torques/forces for each DOF
%
% linkList            = list of joint/link structures created by createLink
% paramList           = N×1 vector of current joint angles or displacements
% paramListDot        = N×1 vector of current joint velocities
% paramListDDot       = N×1 vector of current joint accelerations
% boundry_conditions  = structure containing:
%       base_angular_velocity      – 3×1 angular velocity at the base
%       base_angular_acceleration  – 3×1 angular acceleration at the base
%       base_linear_acceleration   – 3×1 linear acc. at base (include gravity)
%       distal_force               – 3×1 external force at the tool frame
%       distal_torque              – 3×1 external torque at the tool frame
%
% Noah Kelly
% 10884361
% MEGN544
% 12/7/2025
function [jointTorques] = newtonEuler( linkList, paramList, paramListDot, paramListDDot, boundry_conditions )
% Number of Joints
numJoints = length(linkList);

% Pre allocate a list of values that need to be stored in memory between
% loops
list = repmat(struct( 'Zlast', zeros(3,1),...   % Z i-1, rotation axis for link i in base frame
    'Woi', zeros(3,1),...     % Angular velocity of origin i in base frame
    'doi', zeros(3,1),...     % Position of Origin i relative to base in base frame
    'Fi', zeros(3,1),...      % Inertial Force on link i in base frame
    'Ni', zeros(3,1),...      % Inertial Torque on link i in base frame
    'rii', zeros(3,1),...     % Displacement from i-1 to com
    'ri1_i',zeros(3,1) ),...  % Displacemenbt from i to com
    numJoints,1);

% Initialize link variables that get propagated forward
Toi = eye(4); % Transform from 0 to joint i
W = boundry_conditions.base_angular_velocity; % Angluar Velocity in joint frame
Wdot = boundry_conditions.base_angular_acceleration; % Angular Acceleration in joint frame
V = [0;0;0];
Vdot = boundry_conditions.base_linear_acceleration; % Linear acceleration in joint frame

W_BC = boundry_conditions.base_angular_velocity; %Necessary for Jv and Jvdot

num_static = 0; % number of static links encountered

for i=1:numJoints % begin forward iteration from base to tool
    
    % Calculate link transform from i-1 to i
    if linkList(i).isRotary == 1
        % hint i-num_static is the param index to be on
        Ti = dhTransform(linkList(i).a, ...
                         linkList(i).d, ...
                         linkList(i).alpha, ...
                         paramList(i-num_static) - linkList(i).offset);
    elseif linkList(i).isRotary == 0
        Ti = dhTransform(linkList(i).a, ...
                         paramList(i-num_static) - linkList(i).offset, ...
                         linkList(i).alpha, ...
                         linkList(i).theta);
    else
        Ti = dhTransform(linkList(i).a, ...
                         linkList(i).d, ...
                         linkList(i).alpha, ...
                         linkList(i).theta);
        num_static = num_static+1;
    end
    
    % extract distance from joint i-1 to joint i (i-1 frame)
    di = Ti(1:3,4);
    
    % Roi
    Roi = Toi(1:3,1:3);
    
    % extract rotation from joint i to joint i-1 (transpose of i-1 to i
    % rotation)
    Rt = Ti(1:3,1:3)';
    
    % Update joint frame acceleartion, angular Acceleration, and
    % angular velocity. In the i-i frame (so z is [0;0;1])
    Z = [0;0;1];
    if linkList(i).isRotary == 1
        Wprev = W;
        W = Wprev + paramListDot(i-num_static)*Z; % update ang vel
        Wdot = Wdot + paramListDDot(i-num_static)*Z + ...
            paramListDot(i-num_static)*cross(Wprev, Z); % update ang accel in joint frame
        
        Vdotprev = Vdot;
        Vdot = Vdot + cross(Wdot,di) + ...
               cross(W,cross(W,di)); % update accel in joint frame
    elseif linkList(i).isRotary == 0 % W/Wdot same, Vdot different
        Wprev = W; % updage ang vel
        Wdot = Wdot; % update ang accel in joint frame
        
        Vdotprev = Vdot;
        Vdot = Vdot + cross(Wdot,di) + cross(W,cross(W,di)) + ...
               2*paramListDot(i-num_static)*cross(Wprev,Z) + ...
               paramListDDot(i-num_static)*Z; % update accel in joint frame
    else
        Wprev = W; % updage ang vel
        Wdot = Wdot; % update ang accel in joint frame
        
        Vdotprev = Vdot;
        Vdot = Vdot + cross(Wdot,di) + ...
               cross(W,cross(W,di)); % update accel in joint frame
    end
    
    % rotate from i-1 frame to i frame
    Z = Rt*Z;
    Wdot = Rt*Wdot;
    Wprev = Rt*Wprev;
    W = Rt*W;
    V = Rt*V;
    Vdotprev = Rt*Vdotprev;
    Vdot = Rt*Vdot;
    
    % Calculate the displacement from the i-1 frame to the i'th com in
    % the i'th frame
    ri1_i =  Rt*di+linkList(i).com;
    
    % Calculate the Acceleration of the Center of Mass of the link in
    % the ith frame
    Vcdot = Vdotprev + cross(Wdot, ri1_i) + cross(W,cross(W,ri1_i));
    if linkList(i).isRotary == 0
        Vcdot = Vcdot + 2*paramListDot(i-num_static)*cross(Wprev,Z) + ...
                paramListDDot(i-num_static)*Z;
    end
    
    % Calculate and Save Inertial Force and Torque in the i'th frame
    F = linkList(i).mass * Vcdot; % Newton's Equation %Vdot?
    N = linkList(i).inertia*Wdot + ...
        cross(W,linkList(i).inertia*W); % Euler's Equation
    
    % Save values specific to calculating Jv and JvDot that we already know
    list(i).Zlast  = Toi(1:3,3); % Save Zi-1 for Jv and JvDot
    Toi = Toi*Ti; % Update base to link transform
    list(i).doi = Toi(1:3,4); % save distance from base to joint i for Jv and JvDot
    list(i).Woi = Toi(1:3,1:3)*W; % Save Wi i base frame for JvDot
    list(i).Fi = Toi(1:3,1:3)*F; % Inertial Force in base frame
    list(i).Ni = Toi(1:3,1:3)*N; % Inertial Torque in base frame
    list(i).ri1_i = Toi(1:3,1:3)*ri1_i; % Displacement from i-1 to com
    list(i).rii   = Toi(1:3,1:3)*linkList(i).com; % Displacemenbt from i to com
end % End Forward Iterations

% Initialize variables for calculating Jv and JvDot
doN = list(end).doi; % Extract Distance from Base to Tool in base frame

% Initialize variables for force/torque propagation
f = boundry_conditions.distal_force; % Initialize force to external force on the tool in the tool frame
n = boundry_conditions.distal_torque; % Initialize torque to external torque on the tool in the tool frame

% Rotate f & n to base frame
f = Toi(1:3,1:3)*f;
n = Toi(1:3,1:3)*n;

% preallocate joint torque vector
jointTorques = zeros(numJoints,1); % preallocate for speed

for i = numJoints:-1:1 % From Last joint to base
    
    % displacement from origin i-1 to i in base. Hint: use list(i).doi to help...
    if( i> 1)
        d = list(i).doi - list(i-1).doi;
    else
        d = list(i).doi;
    end
    
    %First Half or torque update based on applied force
    n = n - cross(list(i).rii, f);

    % Update Force on joint i in base frame with inertial force from before
    f = f + list(i).Fi;
    
    % Final Update Torque on joint i in base frame with inertial torque and
    % constraint force
    n = n + list(i).Ni + cross(list(i).ri1_i, f);
    
    if linkList(i).isRotary == 1 % Rotational Joint
        % joint i torque is the Z component
        jointTorques(i-num_static) = dot(list(i).Zlast, n);
        
    elseif linkList(i).isRotary == 0 % Prysmatic
        % joint i force is the Z component
        jointTorques(i-num_static) =  dot(list(i).Zlast, f);
        
    else
        num_static = num_static-1;
    end
    
end % End Backword Iterations
end % end newtonEular Function definition