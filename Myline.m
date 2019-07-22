clc;
clear;
hold on

x = 1;
y = 2;

start_point = [0;0];
end_point = [10;20];

line_start = [start_point(x) end_point(x)];
line_end = [start_point(y) end_point(y)];
plot(line_start,line_end);

delta_x = end_point(x) - start_point(x);
delta_y = end_point(y) - start_point(y);
distance = sqrt(delta_x^2 + delta_y^2);
k = delta_y / delta_x;

unit_length = 0.05;
line_segment = floor(distance / unit_length);
last_line_length = distance - line_segment * unit_length;

A_point = start_point;
Insert_Line = start_point';

for i = 1 : 1 : line_segment
    B_point = [A_point(x) + unit_length;A_point(y)];
    C_point = [A_point(x) + unit_length;A_point(y) + unit_length];

    F_real_coord(x) = unit_length * i;
    F_real_coord(y) = F_real_coord(x) * k;
    F_delta_X = A_point(y) - F_real_coord(y);
    F_delta_Y = C_point(y) - F_real_coord(y);
    res = F_delta_X + F_delta_Y;

    if(res > 0)
        if(A_point(x) > end_point(x) || A_point(y) > end_point(y)) break
        end
        plot(B_point(x),B_point(y),'-o');
        A_point(x) = A_point(x) + unit_length;
    else
        if(C_point(x) > end_point(x) || C_point(y) > end_point(y)) break
        end
        plot(C_point(x),C_point(y),'-*');
        A_point(x) = A_point(x) + unit_length;
        A_point(y) = A_point(y) + unit_length;
    end

    Insert_Line = [Insert_Line;A_point'];
end
plot(Insert_Line(:,1),Insert_Line(:,2),'r');
