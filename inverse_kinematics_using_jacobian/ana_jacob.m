function y = jac(q, pe, J, Ta)
    q1 = q(1); q2 = q(2); q3 = q(3); q4 = q(4); q5 = q(5); q6 = q(6);
    A01 = [cos(q1) 0 sin(q1) 0; sin(q1) 0 -cos(q1) 0; 0 1 0 0.3105; 0 0 0 1];
    A12 = [cos(q2) 0 -sin(q2) 0; sin(q2) 0 cos(q2) 0; 0 -1 0 0; 0 0 0 1];
    A02 = A01*A12;
    A23 = [cos(q3) 0 -sin(q3) 0; sin(q3) 0 cos(q3) 0; 0 -1 0 0.4; 0 0 0 1];
    A03 = A02*A23;
    A34 = [cos(q4) 0 sin(q4) 0; sin(q4) 0 -cos(q4) 0; 0 1 0 0; 0 0 0 1];
    A04 = A03*A34;
    A45 = [cos(q5) 0 sin(q5) 0; sin(q5) 0 -cos(q5) 0; 0 1 0 0.39; 0 0 0 1];
    A05 = A04*A45;
    A56 = [cos(q6) 0 -sin(q6) 0; sin(q6) 0 cos(q6) 0; 0 -1 0 0; 0 0 0 1];
    A06 = A05*A56; 
    z0 = [0 0 1]'; p0 = [0 0 0]';
    z1 = A01(1:3,3); p1 = A01(1:3,4);
    z2 = A02(1:3,3); p2 = A02(1:3,4);
    z3 = A03(1:3,3); p3 = A03(1:3,4);
    z4 = A04(1:3,3); p4 = A04(1:3,4);
    z5 = A05(1:3,3); p5 = A05(1:3,4);
    z6 = A06(1:3,3); p6 = A06(1:3,4); 
    Jg = [cross(z0,pe-p0) cross(z1,pe-p1) cross(z2,pe-p2) cross(z3,pe-p3) cross(z4,pe-p4) cross(z5,pe-p5) cross(z6,pe-p6);
           z0 z1 z2 z3 z4 z5 z6];
    y = inv(Ta)*Jg;
end