clear all;

%% parameter
V = -2; 
V_dot = 0; 
L = 6;
dt = 0.05;


%% desired position
xy_desire = [16 4; 12 4; 10 4; 8 2; 6 2; 4 0; 2 0; 0 0;];
[x_desire, y_desire] = BezierCurve(1000,xy_desire);
xy_desire_line = [x_desire y_desire];
phi_desire = 0;

x_desire_dot = 0;
y_desire_dot = 0;
phi_desire_dot = 0;
y_desire_dot2 = 0;

%% desire graph
plot(x_desire,y_desire,'.');
hold on;
for c = 1:8
plot(xy_desire(c,1),xy_desire(c,2),'ro');
end

%% initial position
x = 16;
y = 4;
phi = 0;

x_dot = 0;
y_dot = 0;
phi_dot = 0;

%%% ego drawing
ego = [x,y];
ego_car = rectangle_draw(ego(1,1), ego(1,2), phi, 6, 3,2);
ego_car_x = ego_car(:,1);
ego_car_y = ego_car(:,2);
%ego_car_x = [ego_car_x; ego_car_x(1)];
%ego_car_y = [ego_car_y; ego_car_y(1)];
patch_temp=patch(ego_car_x,ego_car_y,'green');

%% enviroment drawing
obstacle = [11,0];
obstacle_car = rectangle_draw(obstacle(1,1), obstacle(1,2), 0, 6, 3,2);
obstacle_car_x = obstacle_car(:,1);
obstacle_car_y = obstacle_car(:,2);
patch(obstacle_car_x,obstacle_car_y,'red');
hold off;

%% simulation loop start
C1 = 1;
C2 = 2;

waypoint_check = 1;
away_distance = 0.1;
while (waypoint_check < length(y_desire))
    %away_distance = y_desire(waypoint_check,1)-y;
    while away_distance <= 0.5      
        waypoint_check = waypoint_check + 1;
        if waypoint_check >= length(y_desire)
           break
        end
        x_d = x_desire(waypoint_check,1);
        y_d = y_desire(waypoint_check,1);
        away_distance = sqrt((x_d-x)^2 + (y_d-y)^2);
    end
    while x_d > x 
        waypoint_check = waypoint_check + 1;
        if waypoint_check >= length(y_desire)
           break
        end
        x_d = x_desire(waypoint_check,1);
        y_d = y_desire(waypoint_check,1);
        away_distance = sqrt((x_d-x)^2 + (y_d-y)^2);
    end
         
    
    %%% error
    x_tilda = x - x_desire(waypoint_check,1);
    y_tilda = y - y_desire(waypoint_check,1);
    y_tilda_dot = y_dot - y_desire_dot;
    y_tilda_dot2 = y_tilda_dot/dt;
  
    phi_desire = atan2(y_tilda,x_tilda); %%어려운 부분, 추후 수정
    x_desire_dot = V*cos(phi_desire);
    y_desire_dot = V*sin(phi_desire);
    
    %%% controller
    %u = y_desire_dot2(waypoint_check,1) - C1*y_tilda_dot - C2*y_tilda; 
    u = y_desire_dot2 - C1*y_tilda_dot - C2*y_tilda; 
    delta = atan((-L/((V^2)*cos(phi_desire)))*u); % delta : steering angle
    phi_dot = (-V/L)*tan(delta);
    %phi_dot = (1/(V*cos(phi)))*u;
    
    %%% update    
    phi = phi + phi_dot*dt;
    x_dot = V*cos(phi);
    y_dot = V*sin(phi);
    x = x + x_dot*dt;
    y = y + y_dot*dt;
    away_distance = sqrt((x_d-x)^2 + (y_d-y)^2);    
    
    
    %%% ego drawing
    ego = [x,y];
    ego_car = rectangle_draw(ego(1,1), ego(1,2), phi, 6, 3,2);
    ego_car_x = ego_car(:,1);
    ego_car_y = ego_car(:,2);
    patch_temp.Vertices=([ego_car_x ego_car_y]);
    hold on
    plot(x,y,'g.');
    drawnow;
    axis([-5 30 -5 10])
    
end


 



    