function [T01, T02, T03, T04, T05, T06, T0E] = finmat(d,q,a,alpha)
A = zeros(4,4,7);
for i=1:length(q)
    A(:,:,i) = [cos(q(i)), -sin(q(i))*cos(alpha(i)), sin(q(i))*sin(alpha(i)), a(i)*cos(q(i));
    sin(q(i)), cos(q(i))*cos(alpha(i)), -cos(q(i))*sin(alpha(i)), a(i)*sin(q(i));
    0, sin(alpha(i)), cos(alpha(i)), d(i);
    0, 0, 0, 1;];    

end
T01 = A(:,:,1);
T02 = A(:,:,1)*A(:,:,2);
T03 = A(:,:,1)*A(:,:,2)*A(:,:,3);
T04 = A(:,:,1)*A(:,:,2)*A(:,:,3)*A(:,:,4);
T05 = A(:,:,1)*A(:,:,2)*A(:,:,3)*A(:,:,4)*A(:,:,5);
T06 = A(:,:,1)*A(:,:,2)*A(:,:,3)*A(:,:,4)*A(:,:,5)*A(:,:,6);
T0E = A(:,:,1)*A(:,:,2)*A(:,:,3)*A(:,:,4)*A(:,:,5)*A(:,:,6)*A(:,:,7);

end
