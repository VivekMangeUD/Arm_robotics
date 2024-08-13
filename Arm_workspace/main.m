% Define the link lengths
a1 = 0.5;
a2 = 0.3;
a3 = 0.2;
count = 0;
% Define the joint limits
q1_min = -pi/3;
q1_max = pi/3;
q2_min = -2*pi/3;
q2_max = 2*pi/3;
q3_min = -pi/2;
q3_max = pi/2;

c = [];
e = [];
k = [];
l = [];
g = [];
h = [];
splits = [];
%% Step 1
% Create matrix with either of the joints as max or min - total
% combinations = 2^n
% m = zeros(26,3);
m(1,:) = [q1_max q2_max q3_max];
m(2,:) = [q1_max q2_max q3_min];
m(3,:) = [q1_max q2_min q3_max];
m(4,:) = [q1_max q2_min q3_min];
m(5,:) = [q1_min q2_max q3_max];
m(6,:) = [q1_min q2_max q3_min];
m(7,:) = [q1_min q2_min q3_max];
m(8,:) = [q1_min q2_min q3_min];

%% Step 2
% Keep 1 joint at limits and rest all to 0
% keep q1 on limits
m(9,:) = [q1_max 0 0];
m(10,:) = [q1_min 0 0];

% keep q2 on limits
m(11,:) = [0 q2_max 0];
m(12,:) = [0 q2_min 0];

% keep q3 on limits
m(13,:) = [0 0 q3_max];
m(14,:) = [0 0 q3_min];

%% Step 3
% Keep 2 joint at limits and rest all to 0
% keep q2 and q3 on limits

m(15,:) = [0 q2_max q3_max];
m(16,:) = [0 q2_max q3_min];
m(17,:) = [0 q2_min q3_max];
m(18,:) = [0 q2_min q3_min];

% keep q1 and q3 on limits
m(19,:) = [q1_max 0 q3_max];
m(20,:) = [q1_max 0 q3_min];
m(21,:) = [q1_min 0 q3_max];
m(22,:) = [q1_min 0 q3_min];

% keep q1 and q2 on limits
m(23,:) = [q1_max q2_max 0];
m(24,:) = [q1_max q2_min 0];
m(25,:) = [q1_min q2_max 0];
m(26,:) = [q1_min q2_min 0];

figure
subplot(1,2,1)
for i = 1:length(m) % Running the loop to compare all 26 matrix
    for j = 1:length(m) % loop to compare the matrix with each other
         c(i,:) = (m(i,:) == m(j,:)); % compare the matrix and generate an array of 1 and 0 where 1 is same element and 0 is differemt
         if nnz(c(i,:)) == 2 % Check the no of non zero values i.e no of ones in the array, we use this to check if there is only one different after comparing
             if j>i % This conditions will help neglect the repeating conditions
                e(end+1,:) = [i j]; % store the successful combinations 
                k(end+1,:) = (m(e(end,1),:) == m(e(end,2),:)); % compare the combination matrix and get 0 and 1
                l(end+1,:) = find(k(end,:)==0); % find the pos of 0 
                splits(end+1,:) = linspace(m(e(end,1),l(end)), m(e(end,2),l(end)), 50); % split the respected varible in 50 steps
                if l(end,1) ==1 % if the split angle in 1st angle, use the below equation
                    x = a1*cos(splits(end,:))+a2*cos(splits(end,:)+m(i,2))+a3*cos(splits(end,:)+m(i,2)+m(i,3));
                    y = a1*sin(splits(end,:))+a2*sin(splits(end,:)+m(i,2))+a3*sin(splits(end,:)+m(i,2)+m(i,3));
                    plot(x,y,'b')
                    hold on
                elseif l(end,1) == 2 % if the split angle in 1st angle, use the below equation
                    x = a1*cos(m(i,1))+a2*cos(m(i,1)+splits(end,:))+a3*cos(m(i,1)+splits(end,:)+m(i,3));
                    y = a1*sin(m(i,1))+a2*sin(m(i,1)+splits(end,:))+a3*sin(m(i,1)+splits(end,:)+m(i,3));     
                    plot(x,y,'b')
                    hold on
                elseif l(end, 1) == 3 % if the split angle in 1st angle, use the below equation
                    x = a1*cos(m(i,1))+a2*cos(m(i,1)+m(i,2))+a3*cos(m(i,1)+m(i,2)+splits(end,:));
                    y = a1*sin(m(i,1))+a2*sin(m(i,1)+m(i,2))+a3*sin(m(i,1)+m(i,2)+splits(end,:));  
                    plot(x,y,'b')
                    hold on
                end
             end
         end
    end
end
axis equal
title('Robot Workspace')

% part B
% Split the joints in 50 steps
q1 = linspace(q1_min,q1_max,50)';
q2 = linspace(q2_min,q2_max,50)';
q3 = linspace(q3_min,q3_max,50)';

for i = 1:length(q1)
    for j = 1:length(q2)
        for k = 1:length(q3)
            % calculate x and y pose of for all the values of q1,q2 and q3
            g(end+1,:) = a1*cos(q1(i))+a2*cos(q1(i)+q2(j))+a3*cos(q1(i)+q2(j)+q3(k));
            h(end+1,:) = a1*sin(q1(i))+a2*sin(q1(i)+q2(j))+a3*sin(q1(i)+q2(j)+q3(k));
        end
    end
end

subplot(1,2,2)
plot(g,h,'r','marker','.')
hold on
axis equal
title('Validation Plot')
