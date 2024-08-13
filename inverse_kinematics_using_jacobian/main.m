close all
clear all
PD = [0.7458; -0.2494; 0.4810];
phi = atan2(-0.1357,0.8984);
theta = atan2(sqrt(0.8984^2+0.1357^2),0.4177);
zi = atan2(-0.8114,0.4089);
PHID = [phi; theta; zi];
k = 1000;
q = zeros(7, k);
e = zeros(6, k);
J = zeros(6,7);
Tr(1:3,1:6) = [eye(3,3) zeros(3,3)];
Tr(4:6,1:6) = [zeros(3,3) transpose([0 0 1; -sin(phi) cos(phi) 0;
cos(phi)*cos(theta) sin(phi)*sin(theta) cos(theta)])
];
q(:,1) = [pi/7; pi/6; pi/5; pi/4; pi/3; pi/2; pi];
K = 100*eye(6);
for i=1:k
    xe = fwd_kin(q(:,i));
    Ja = ana_jacob(q(:,i),PD,J,Tr);
    e(:,i) = [PD; PHID]-xe;
    qdot = pinv(Ja)*K*e(:,i);
    q(:,i+1) = q(:,i)+qdot*0.001;
    if (max(abs(e(:,i)))<0.001)
        final_config=q(:,i); 
        break;
    end
end
fwd_kin(final_config)
T07 = [0.0525 0.4360 0.8984 0.7458; -0.9111 0.3893 -0.1357 -0.2494;
       -0.4089 -0.8114 0.4177 0.4810; 0         0       0       1]
final_config
finmat(final_config)