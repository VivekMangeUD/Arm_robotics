% Verify the window GH in FOV of Camera at point F
clear all
close all

% Read values of rotation from encoder
fid = fopen('encoder.txt','rt');
D = textscan(fid,'%f %f %f %f %f %f'); % Generate a (1x6) Matrix to store all the Encoder data

% Original Pose of all the joints
o = [0 0 0];
a=[-1 4 0];
b=[0 8 0];
c=[-2 10 0];
d=[8 14 0];
e=[10 20 0];
f=[11 20 0];
g = [40 21 0];
h = [40 19 0];
mid_gh = [((g(1) + h(1))/2) ((g(2)+h(2))/2)]; % Midpoint Calculation
output=[]; % Output variable declaration to store either 0 or 1
% Distance between joints - Link Lengths
oa = norm(a-o);
ab = norm(b-a);
bc = norm(c-b);
cd = norm(d-c);
de = norm(e-d);
ef = norm(f-e);

% Initail angle declaration based on the zero position
qo = 1.8158;
qa = 0.4895;
qb = 1.030;
qc = 1.9756;
qd = 0.8674;
qe = 1.2490;

figure
for j = 1:length(D{1,1})  % For loop till the length of encoder data
    po = [0 0 0 1];  % base Position of joint O
    
    % Logic
    % Calculate pose of all the joints.
    Rzo = myrotmat(qo+D{1,1}(j), 'z');  % Rotation wrt to Z axis with offest qo angle(0 pose of the joint) + Encoder data
    % calculate Homogeneous Matrix 
    %   [Rz(q+encoder) 0]   *   [I(3) Length]
    %   [      0       1]       [ 0     1   ]  
    Tooe = [Rzo; 0 0 0];
    Tooe = [Tooe, [0; 0; 0; 1]];

    Toea = [eye(3,3); 0 0 0 ];
    Toea = [Toea,[oa,0,0,1]'];

    pa = Tooe*Toea*[0;0;0;1];  % Homogenous Matrix * Pose of A wrt frame A


    Rza = myrotmat(-qa+D{1,2}(j), 'z');  % -qa WRT to right Right hand thumb rule
    Taae = [Rza; 0 0 0];
    Taae = [Taae, [0; 0; 0; 1]];

    Taeb = [eye(3,3); 0 0 0 ];
    Taeb = [Taeb,[ab,0,0,1]'];

    pb = Tooe*Toea*Taae*Taeb*[0;0;0;1];


    Rzb = myrotmat(qb+D{1,3}(j), 'z');
    Tbbe = [Rzb; 0 0 0];
    Tbbe = [Tbbe, [0; 0; 0; 1]];

    Tbec = [eye(3,3); 0 0 0 ];
    Tbec = [Tbec,[bc,0,0,1]'];

    pc = Tooe*Toea*Taae*Taeb*Tbbe*Tbec*[0;0;0;1];


    Rzc = myrotmat(-qc+D{1,4}(j), 'z');
    Tcce = [Rzc; 0 0 0];
    Tcce = [Tcce, [0; 0; 0; 1]];

    Tced = [eye(3,3); 0 0 0 ];
    Tced = [Tced,[cd,0,0,1]'];

    pd = Tooe*Toea*Taae*Taeb*Tbbe*Tbec*Tcce*Tced*[0;0;0;1];


    Rzd = myrotmat(qd+D{1,5}(j), 'z');
    Tdde = [Rzd; 0 0 0];
    Tdde = [Tdde, [0; 0; 0; 1]];

    Tdee = [eye(3,3); 0 0 0 ];
    Tdee = [Tdee,[de,0,0,1]'];

    pe = Tooe*Toea*Taae*Taeb*Tbbe*Tbec*Tcce*Tced*Tdde*Tdee*[0;0;0;1];


    Rze = myrotmat(-qe+D{1,6}(j), 'z');
    Teee = [Rze; 0 0 0];
    Teee = [Teee, [0; 0; 0; 1]];

    Teef = [eye(3,3); 0 0 0 ];
    Teef = [Teef,[ef,0,0,1]'];

    pf = Tooe*Toea*Taae*Taeb*Tbbe*Tbec*Tcce*Tced*Tdde*Tdee*Teee*Teef*[0;0;0;1];

    % Generate the FOV  
    slope_ef = (pe(2)-pf(2)/pe(1)-pf(1)); % calculate slope of EF
    angle_of_intersection = atan2(pe(2) - pf(2), pe(1) - pf(1));  % Tanget of the slope
    % add and subtract 30 degree to generate the FOV of 60 degree
    plus_30_angle = angle_of_intersection + deg2rad(30);
    minus_30_angle = angle_of_intersection - deg2rad(30);
    % Calculate slope and new line equations of the 2 angles
    slope_plus_30 = tan(plus_30_angle);
    slope_minus_30 = tan(minus_30_angle);
    % Line + 30 degree
    const1 = pf(2) - slope_plus_30*pf(1);
    Y1 = slope_plus_30*(h(1)) + const1;

    % Line -30 degree
    const2 = pf(2) - slope_minus_30*pf(1);
    Y2 = slope_minus_30*(g(1)) + const2;
    if Y1<h(2) || Y2>g(2)
        output(j) = 0;
    else
        output(j) = 1;
    end

    % Calculate Minimum Distance of F and Window GH
    % Consider 3 points of window GH - Point G, Point H and Midpoint of GH
    % calculate distance of point F and point G, Point H and Midpoint GH
    min_distance{1,1}(j) = sqrt(((pf(1)-g(1))^2)+((pf(2)-g(2))^2));
    min_distance{1,2}(j) = sqrt(((pf(1)-mid_gh(1))^2)+((pf(2)-mid_gh(2))^2));
    min_distance{1,3}(j) = sqrt(((pf(1)-h(1))^2)+((pf(2)-h(2))^2));
end
% Grapgh Plot for Time vs output
plot((1:28800)/3600, output(1:28800))
axis([-0.5 8.5 -0.5 1.5])
xlabel('Time(h)')
ylabel('Binay Signal')
title('The dinosaur mechanism.')
% calculate of Minimum value in the array
G_min = min(min_distance{1,1},[],'all');
GH_mid_min = min(min_distance{1,2},[],'all');
H_min = min(min_distance{1,3},[],'all');
% Final minimum value of point F WRT to window GH
m= min([G_min,H_min,GH_mid_min]);
fprintf('Minimum Distance of Point F from Window GH is : %f \n', m);

