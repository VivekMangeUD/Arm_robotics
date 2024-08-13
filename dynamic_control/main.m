clear all
close all

tau = importdata('torques.txt');

% initial condition given pose and velocity as 0 --> {q1, q2, q1dot, q2dot}
x1 = [0;0;0;0];
x2 = [0;0;0;0];
joint_ang1k = zeros(length(tau),2);
joint_ang200 = zeros(length(tau),2);

options = odeset('RelTol',1e-4,'AbsTol',[1e-5 1e-5 1e-5 1e-5]);

for i = 1:length(tau)

    [tsim,y1] = ode45(@samplesys,[0 1/1000],x1,options,tau(i,:)');
    x1 = y1(end,:);
    % format of Output - {q1dot, q1ddot, q2dot, q2dot}
    joint_ang1k(i+1,:) = [y1(end,1), y1(end,3)];
    

    [tsim,y2] = ode45(@samplesys,[0 1/200],x2,options,tau(i,:)');
    x2 = y2(end,:);
    % format of Output - {q1dot, q1ddot, q2dot, q2dot}
    joint_ang200(i+1,:) = [y2(end,1), y2(end,3)];
end
% Graph plot
figure(1)
pos1 = fwd_kin(joint_ang1k(:,1),joint_ang1k(:,2));
plot(pos1(:,1),pos1(:,2));
title('1 kHz');
xlabel('X');
ylabel('Y');
 
figure(2)
pos2 = fwd_kin(joint_ang200(:,1),joint_ang200(:,2));
plot(pos2(:,1),pos2(:,2));
title('200 Hz');
xlabel('X');
ylabel('Y');

