%state space function
function dx = samplesys(t,x1,T)
[B, C, G] = dynamics_matrices([x1(1);x1(3)],[x1(2);x1(4)]);
% [B, C, G] = dynamics_matrices([x1(1);x1(2)],[x1(3);x1(4)]);

qddot = B\(T-G-C*[x1(2);x1(4)]);

% dx = zeros(4,1);
% dx(1) = x1(2);
% dx(2) = qddot(1);
% dx(3) = x1(4);
% dx(4) = qddot(2);
% dx = [x1(3);x1(4);qddot];
dx = [x1(2);qddot(1);x1(4);qddot(2)];
end
