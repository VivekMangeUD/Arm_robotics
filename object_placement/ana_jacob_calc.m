function [y,Jg] = ana_jacob_calc(d,q,a,alpha,Ta)

    [xe,T01, T02, T03, T04, T05, T06, T0E] = fwd_kin(d,q,a,alpha);
    z0 = [0 0 1]'; p0 = [0 0 0]';
    z1 = T01(1:3,3); p1 = T01(1:3,4);
    z2 = T02(1:3,3); p2 = T02(1:3,4);
    z3 = T03(1:3,3); p3 = T03(1:3,4);
    z4 = T04(1:3,3); p4 = T04(1:3,4);
    z5 = T05(1:3,3); p5 = T05(1:3,4);
    z6 = T06(1:3,3); p6 = T06(1:3,4);
    z7 = T0E(1:3,3); pe = T0E(1:3,4);
    Jg = [cross(z0,pe-p0) cross(z1,pe-p1) cross(z2,pe-p2) cross(z3,pe-p3) cross(z4,pe-p4) cross(z5,pe-p5) cross(z6,pe-p6);
           z0 z1 z2 z3 z4 z5 z6];
       
    y = inv(Ta)*Jg;
end
