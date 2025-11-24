function [th1, th2, th3, reachable] = snakePlanarInvKine(T_des, d, th_last)

x4 = T_des(1,3);
y4 = T_des(2,3);
phi = atan2(T_des(2,1), T_des(1,1));  % end-effector orientation

x3 = x4 - d(3)*cos(phi);
y3 = y4 - d(3)*sin(phi);

r = sqrt(x3^2 + y3^2);

if r > (d(1) + d(2)) || r < abs(d(1) - d(2))
    th1 = []; th2 = []; th3 = [];
    reachable = 0;
    return
end

reachable = 1;

cos_th2 = (r^2 - d(1)^2 - d(2)^2) / (2*d(1)*d(2));
sin_th2 = sqrt(1 - cos_th2^2);

th2 = [atan2( sin_th2, cos_th2 );
       atan2(-sin_th2, cos_th2 )];

th1 = zeros(2,1);

for i = 1:2
    k1 = d(1) + d(2)*cos(th2(i));
    k2 = d(2)*sin(th2(i));
    th1(i) = atan2(y3, x3) - atan2(k2, k1);
end

% Compute th3 from phi = th1 + th2 + th3
th3 = phi - th1 - th2;

th1 = wrapToPi(th1);
th2 = wrapToPi(th2);
th3 = wrapToPi(th3);

if exist('th_last', 'var')
    best_diff = 100;
    for k = 1:2
        diff = abs(th1(k)-th_last(1)) + abs(th2(k)-th_last(2)) + abs(th3(k)-th_last(3));
        if diff < best_diff
            best_index = k;
            best_diff = diff;
        end
    end
    th1 = th1(best_index);
    th2 = th2(best_index);
    th3 = th3(best_index);
end
end