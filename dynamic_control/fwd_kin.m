function pos = fwd_kin(q1,q2)

a1 = 0.5;
a2 = 0.6;

%forward kinematics
pos = [a1*cos(q1)+a2*cos(q2+q1), a1*sin(q1)+a2*sin(q2+q1)];
end

