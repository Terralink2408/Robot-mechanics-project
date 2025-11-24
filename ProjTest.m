t0 = 0; 
tf = 2;
n = 100;
t_list = linspace(t0, tf, n); % time vector

d = 1 + (2 - 1) * rand(3,1); % link lengths

th_given_0 = (-pi) + (pi - (-pi)) * rand(3,1); % given initial theta values for fwd kine
th_given_f = (-pi) + (pi - (-pi)) * rand(3,1); % given final theta values for fwd kine

[T_0,~] = snakePlanarFwdKine(d, th_given_0); % Fwd kine: initial homo trans
[T_f,~] = snakePlanarFwdKine(d, th_given_f); % Fwd kine: final homo trans

[th1_0, th2_0, th3_0, ~] = snakePlanarInvKine(T_0, d, th_given_0); % Inv kine: initial theta values
[th1_f, th2_f, th3_f, ~] = snakePlanarInvKine(T_f, d, th_given_f); % Inv kine: final theta values

q0 = [th1_0 th2_0 th3_0]; % theta vector for cubic interp
qf = [th1_f th2_f th3_f];

% Cubic interpolatation
a = zeros(3,4);
for i = 1:3
    a(i,:) = cubicInterpCoeff(q0(i), qf(i), t0, tf);
end
Q = cubicInterpolate(a, t0, t_list);

% Plot movie from initial to final transform
for j = 1:n
    figure(1)
    clf(1)
    snakePlanarPlot(Q(1,j),Q(2,j),Q(3,j),d,T_f,1)
    pause(0.1)
    %exportgraphics(gcf,'testAnimated.gif','Append',true)
end