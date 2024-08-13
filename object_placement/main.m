close all
clear

% DH param declaration
d = [0.340,0.0,0.400,0.0,0.400,0.0,0.126];
a = zeros(1,7);
alpha = [-pi/2,pi/2,pi/2,-pi/2,-pi/2,pi/2,0];

% Initial Angle
deg_q1 = [58.2686, 75.3224, 11.7968, 45.9029 ,-22.1081, -31.2831, -42.3712];
q1 = deg2rad(deg_q1);
% Q at config qc
deg_qc = [-77.26, -38.76, 26.22, 93.29, -56.69, -59.94, 118];
qc = deg2rad(deg_qc);

% limits declaration
% q_pose_limit = [165,115,165,115,165,115,170];
% q_velocity_limit = [98,98,100,130,140,180,180];
q_pose_limit = rad2deg([165,115,165,115,165,115,170]);
q_velocity_limit = rad2deg([98,98,100,130,140,180,180]);

% Forward Kinemtics to calculate base to end effector at q = q1 & qc
[x1_fwd,T01qc, T02qc, T03qc, T04qc, T05qc, T06qc, T0Eqc] = fwd_kin(d,qc,a,alpha);
[T01q1, T02q1, T03q1, T04q1, T05q1, T06q1, T0Eq1] = finmat(d,q1,a,alpha);

% Calc transformation from End eff to Camera
% rotation in z by -90 degree
yec = -0.0662000;
xec = 0.0;
zec = 0.025+0.0181;
rot_ec = myrotmat(-pi/2,'z');
TEC(1:3,1:3) = rot_ec;
TEC(4,:)= [0 0 0];
TEC(:,4) = [xec yec zec 1];

% calc transform from end eff to top - middle point of flange
% considering the orientation of top plate at target when placed as desired
yef = 0.033+(0.025/2);
zef = 0.0600;
xef = 0.00;
ef_rot_z = myrotmat(-pi/2,'z');
ef_rot_x = myrotmat(pi,'x');
ef_new_rot = ef_rot_z*ef_rot_x;
TEF(1:3,1:3) = ef_new_rot;
TEF(4,:) = [0 0 0];
TEF(:,4) = [xef yef zef 1];

% Calc of Aruko Marker from Camera
xac = -0.14195360128424114;
yac = -0.06062556383004704;
zac = 0.3528046636209403;

roll_xac =  -172.95718336855933;
pitch_yac = -27.847557028831005;
yaw_zac = 68.70697922863141;

% angles = [roll_xac pitch_yac yaw_zac];
angles = [yaw_zac pitch_yac roll_xac];
rad_angles = deg2rad(angles);
rot_ac = eul2rotm(rad_angles, 'ZYX');

TCA(1:3,1:3) = rot_ac;
TCA(4,:) = [0 0 0];
TCA(:,4) = [xac yac zac 1];

% calc of Target pose from Aruko marker
TAT(1:3,1:3) = eye(3);
TAT(4,:) = [0 0 0];
TAT(:,4) = [0.053975+0.050 -(0.053975+0.050) 0 1];

% Final Transformation of base to Target
T0T = T0Eqc*TEC*TCA*TAT;

% Method
% Calc T0E for final pose at end position T0T = T0F where T0F = T0E*TEF
% T0F = T0T;
final_T0E = T0T*inv(TEF);

% Final pose of the end_effector
PD = final_T0E(1:3,4);

% When theta = (0,pi)
phi = atan2(final_T0E(2,3),final_T0E(1,3));
theta = atan2(sqrt((final_T0E(1,3))^2+(final_T0E(2,3))^2),final_T0E(3,3));
zi = atan2(final_T0E(3,2),-final_T0E(3,1));

PHID = [phi; theta; zi];

k = 100000;
inv_q = zeros(7, k);
e = zeros(6, k);
inv_q(:,1) = q1;
K = 1*eye(6);

Ta(1:3,1:6) = [eye(3,3) zeros(3,3)];
Ta(4:6,1:6) = [zeros(3,3) transpose([0 0 1; -sin(phi) cos(phi) 0;
cos(phi)*sin(theta) sin(phi)*sin(theta) cos(theta)])];

% [Ja_q1,~] = ana_jacob_calc(d, inv_q(:,1), a, alpha,Ta);

for i=1:k
    [xe,~] = fwd_kin(d, inv_q(:,i), a, alpha);
    e(:,i) = [PD; PHID]-xe;
    [Ja,~] = ana_jacob_calc(d, inv_q(:,i), a, alpha,Ta);
    qdot = pinv(Ja)*K*e(:,i);
    % limit check:
    % If inside limit update the value of q, else, Take lower limit
    if  inv_q(1,i)>= deg2rad(-165) && inv_q(1,i) <= deg2rad(165) 
        inv_q(1,i+1) = inv_q(1,i) + qdot(1,1)*0.01; 
    else
        inv_q(1,i+1) = deg2rad(-165);
    end
    if  inv_q(2,i)>= deg2rad(-115) && inv_q(2,i) <= deg2rad(115)
        inv_q(2,i+1) = inv_q(2,i) + qdot(2,1)*0.01; 
    else
        inv_q(2,i+1) = deg2rad(-115);
    end
    if  inv_q(3,i)>= deg2rad(-165) && inv_q(3,i) <= deg2rad(165)
        inv_q(3,i+1) = inv_q(3,i) + qdot(3,1)*0.01; 
    else
        inv_q(3,i+1) = deg2rad(-165);
    end
    if  inv_q(4,i)>= deg2rad(-115) && inv_q(4,i) <= deg2rad(115)
        inv_q(4,i+1) = inv_q(4,i) + qdot(4,1)*0.01; 
    else
        inv_q(4,i+1) = deg2rad(-115);
    end
    if  inv_q(5,i)>= deg2rad(-165) && inv_q(5,i) <= deg2rad(165)
        inv_q(5,i+1) = inv_q(5,i) + qdot(5,1)*0.01; 
    else
        inv_q(5,i+1) = deg2rad(-165);
    end
    if  inv_q(6,i)>= deg2rad(-115) && inv_q(6,i) <= deg2rad(115)
        inv_q(6,i+1) = inv_q(6,i) + qdot(6,1)*0.01; 
    else
        inv_q(6,i+1) = deg2rad(-115);
    end
    if  inv_q(7,i)>= deg2rad(-170) && inv_q(7,i) <= deg2rad(170)
        inv_q(7,i+1) = inv_q(7,i) + qdot(7,1)*0.01; 
    else
        inv_q(7,i+1) = deg2rad(-170);
    end
    if (max(abs(e(:,i)))<0.0000000001) 
        final_config=inv_q(:,i); 
        break;
    end
end

% % % Jacobian check
% % deg_q2 = deg_q1 + 1;
% % q2 = deg2rad(deg_q2);
% % [x1,T01q1, T02q1, T03q1, T04q1, T05q1, T06q1, T0Eq1] = fwd_kin(d,q1,a,alpha);
% % [x2,T01q2, T02q2, T03q2, T04q2, T05q2, T06q2, T0Eq2] = fwd_kin(d,q2,a,alpha);
% % % % compute j for q1 and q2
% % [ana_q1, geo_q1] = ana_jacob_yash_logic(d,q1,a,alpha);
% % [ana_q2, geo_q2] = ana_jacob_yash_logic(d,q2,a,alpha);
% % delta_x = geo_q1*(q2'-q1');
% % xyz_angles = rotm2eul(T0Eq1(1:3,1:3),'xyz');
% % xyz2_angles = rotm2eul(T0Eq2(1:3,1:3),'xyz');
% % x1(4:6) = xyz_angles;
% % x2(4:6) = xyz2_angles;
% % delta_x_fwd = x2-x1;
% % f = delta_x-delta_x_fwd

% Plotting error to check Convergence
% subplot(2,3,1)
% plot(e(1,:))
% subplot(2,3,2)
% plot(e(2,:))
% subplot(2,3,3)
% plot(e(3,:))
% subplot(2,3,4)
% plot(e(4,:))
% subplot(2,3,5)
% plot(e(5,:))
% subplot(2,3,6)
% plot(e(6,:))

% Verify final config transformation
[~,~, ~, ~, ~, ~, ~, T0Efinal] = fwd_kin(d,final_config,a,alpha);
T0Efinal = T0Efinal*TEF;

% Tragectory planning
ti = 0;
tf = 10;
t = transpose(linspace(ti,tf,2000));

% Final config
% qf_degree = rad2deg(final_config);
qf_degree = (final_config);

% Storing all q values 
q_f = zeros(length(t), length(qf_degree));
qdot_f = zeros(length(t), length(qf_degree));
% constant declaration
a2 = zeros(1,7);
a3 = zeros(1,7);
a0 = q1';
% a3 = -0.0667*a2;

for i = 1:length(t)
    for j = 1:7
        a2(j) = (qf_degree(j) - q1(j))/33.30;
        a3(j) = (-1/15)*a2(j);
        q_f(i,j) = (a3(j)*t(i)^3 +a2(j)*t(i)^2 + a0(j));
        qdot_f(i,j) = 3*a3(j)*t(i)^2 +2*a2(j)*t(i);
    end
end

% % Plot Q and Qdot
% figure
% plot(t, (q_f))
% figure
% plot(t, deg2rad(qdot_f))

% % % Test round of q and final test matrix
% % -2.1922 -1.5973 -2.1692 -1.1599 -1.3796 1.0249 -0.5145
qf_check_trag = [-2.1922 -1.5973 -2.1692 -1.1599 -1.3796 1.0249 -0.5145]' - final_config;
[~,~, ~, ~, ~, ~, ~, roundT0Efinal] = fwd_kin(d,q_f(end,:),a,alpha);
roundT0Efinal = roundT0Efinal*TEF;

% Check pose and velocity limits in tragetory
% Velocity limit check
if rad2deg(qdot_f(:,1)) > 98 | rad2deg(qdot_f(:,1)) < -98
    disp("Joint 1 velocity limit hit")
elseif rad2deg(qdot_f(:,2)) > 98 | rad2deg(qdot_f(:,2)) < -98
    disp("Joint 2 velocity limit hit")
elseif rad2deg(qdot_f(:,3)) > 100 | rad2deg(qdot_f(:,3)) < -100
    disp("Joint 3 velocity limit hit")
elseif rad2deg(qdot_f(:,4)) > 130 | rad2deg(qdot_f(:,4)) < -130
    disp("Joint 4 velocity limit hit")
elseif rad2deg(qdot_f(:,5)) > 140 | rad2deg(qdot_f(:,5)) < -140
    disp("Joint 5 velocity limit hit")
elseif rad2deg(qdot_f(:,6)) > 180 | rad2deg(qdot_f(:,6)) < -180
    disp("Joint 6 velocity limit hit")
elseif rad2deg(qdot_f(:,7)) > 180 | rad2deg(qdot_f(:,7)) < -180
    disp("Joint 7 velocity limit hit")
end

% Pose limit check
if rad2deg(q_f(:,1)) > 170 | rad2deg(q_f(:,1)) < -170
    disp("Joint 1 pose limit hit")
elseif rad2deg(q_f(:,2)) > 120 | rad2deg(q_f(:,2)) < -120
    disp("Joint 2 pose limit hit")
elseif rad2deg(q_f(:,3)) > 170 | rad2deg(q_f(:,3)) < -170
    disp("Joint 3 pose limit hit")
elseif rad2deg(q_f(:,4)) > 120 | rad2deg(q_f(:,4)) < -120
    disp("Joint 4 pose limit hit")
elseif rad2deg(q_f(:,5)) > 170 | rad2deg(q_f(:,5)) < -170
    disp("Joint 5 pose limit hit")
elseif rad2deg(q_f(:,6)) > 120 | rad2deg(q_f(:,6)) < -120
    disp("Joint 6 pose limit hit")
elseif rad2deg(q_f(:,7)) > 175 | rad2deg(q_f(:,7)) < -175
    disp("Joint 7 pose limit hit")
end
writematrix(round((q_f)',4)','Mange_Vivek_Dharmesh.txt','Delimiter','space')
writematrix(round((rad2deg(q_f))',4)','Mange_Vivek_Dharmesh_Degree_file.txt','Delimiter','space')
writematrix(round((rad2deg(qdot_f))',4)','Mange_Vivek_Dharmesh_velocity_Degree_file.txt','Delimiter','space')
