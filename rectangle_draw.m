function [rectangle_car] = rectangle_draw(x, y, phi, rtg_length, rtg_width, choice)

%% center
if choice == 1
cross = ((rtg_length/2)^2 + (rtg_width/2)^2)^0.5; 
between_angle = atan2(rtg_width/2,rtg_length/2);
%point 1
rectangle(1,1) = x  +  cross * cos(phi + between_angle);
rectangle(1,2) = y  +  cross * sin(phi + between_angle);
%point 2
rectangle(2,1) = x  +  cross * cos(phi - between_angle);
rectangle(2,2) = y  +  cross * sin(phi - between_angle);
%point 3
rectangle(3,1) = x  -  cross * cos(phi + between_angle);
rectangle(3,2) = y  -  cross * sin(phi + between_angle);
%point 4
rectangle(4,1) = x  -  cross * cos(phi - between_angle);
rectangle(4,2) = y  -  cross * sin(phi - between_angle);
rectangle_car = rectangle;
end


%% backward
if choice == 2
cross = ((rtg_length)^2 + (rtg_width/2)^2)^0.5;
between_angle = atan2(rtg_width/2,rtg_length);
%point 1
rectangle(1,1) = x  +  cross * cos(phi + between_angle);
rectangle(1,2) = y  +  cross * sin(phi + between_angle);
%point 2
rectangle(2,1) = x  +  cross * cos(phi - between_angle);
rectangle(2,2) = y  +  cross * sin(phi - between_angle);
%point 3
rectangle(3,1) = x  +  rtg_width/2 * cos(pi/2 - phi);
rectangle(3,2) = y  -  rtg_width/2 * sin(pi/2 - phi);
%point 4
rectangle(4,1) = x  -  rtg_width/2 * cos(pi/2 - phi);
rectangle(4,2) = y  +  rtg_width/2 * sin(pi/2 - phi);
rectangle_car = rectangle;
end