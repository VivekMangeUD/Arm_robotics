function plotrefsys(R,p,c);

hold on
axis_start = p;

for i = 1:3    
    axis_end(:,i) = axis_start + R(:,i);
end

plot3(p(1), p(2), p(3), 'o','Color',c);

axis_c = ['r', 'g', 'b'];
for i = 1:3
plot3([axis_start(1) axis_end(1,i)],...
    [axis_start(2) axis_end(2,i)],...
    [axis_start(3) axis_end(3,i)], 'Color', axis_c(i));
end

axis equal


    