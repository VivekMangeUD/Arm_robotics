function [b,T01, T02, T03, T04, T05, T06, T0E] = fwd_kin(d,q,a,alpha)
    
    [T01, T02, T03, T04, T05, T06, T0E] = finmat(d,q,a,alpha);
    x = T0E(1,4); y = T0E(2,4); z = T0E(3,4);
    r23 = T0E(2,3); r13 = T0E(1,3); r33 = T0E(3,3); r32 = T0E(3,2); r31 = T0E(3,1);
    phi = atan2(r23,r13);
    thet = atan2(sqrt(r13^2+r23^2),r33);
    zi = atan2(r32,-r31);

    b = [x; y; z; phi; thet; zi];
end
