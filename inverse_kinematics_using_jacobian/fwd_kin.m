function a = fwkin(q)
    
    T07 = finmat(q);
    x = T07(1,4); y = T07(2,4); z = T07(3,4);
    r23 = T07(2,3); r13 = T07(1,3); r33 = T07(3,3); r32 = T07(3,2); r31 = T07(3,1);
    phi = atan2(r23,r13);
    thet = atan2(sqrt(r13^2+r23^2),r33);
    zi = atan2(r32,-r31);

    a = [x; y; z; phi; thet; zi];
end
