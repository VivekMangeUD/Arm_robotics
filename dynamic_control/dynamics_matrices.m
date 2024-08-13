function [B, C, G] = dynamics_matrices(q, qdot)

g = 9.807;
Il1 = 0.01;
Il2 = 0.015;

ml1 = 0.1;
ml2 = 0.15;

kr1 = 20;
kr2 = 20;

l1 = 0.25;
l2 = 0.25;

a1 = 0.5;
a2 = 0.6;

Im1 = 0.001;
Im2 = 0.001;

mm1 = 0.01;
mm2 = 0.01;

B(1,1) = Il1 + ml1*l1^2 + (kr1^2)*Im1 + Il2 + ml2*(a1^2 + l2^2 + 2*a1*l2*cos(q(2))) + Im2 + mm2*a1^2;
B(1,2) = Il2 + ml2*(l2^2 + a1*l2*cos(q(2))) + kr2*Im2;
B(2,1) = B(1,2);
B(2,2) = Il2 + ml2*l2^2 + kr2^2*Im2;

h = -ml2*a1*l2*sin(q(2));

C = [ h*qdot(2), h*(qdot(1)+qdot(2)); -h*qdot(1), 0];

G(1,1) = (ml1*l1 + mm2*a1 + ml2*a1)*g*cos(q(1)) + ml2*l2*g*cos(q(1)+q(2));
G(2,1) = ml2*l2*g*cos(q(1)+q(2));