function b = finmat(q)
    q1 = q(1);
    q2 = q(2);
    q3 = q(3);
    q4 = q(4);
    q5 = q(5);
    q6 = q(6);
    q7 = q(7);
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
    A67 = [cos(q7) -sin(q7) 0 0; sin(q7) cos(q7) 0 0; 0 0 1 0.083; 0 0 0 1];
    T07 = A06*A67;
    b = T07;
end