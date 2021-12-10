clear all;

%% parameter
V = 5; 
V_dot = 0; 
L = 1;
dt = 0.01;


%% desired position
xy_desire = [4 4; 12 4; 12 12; 4 12];
x_desire = xy_desire(:,1);
y_desire = xy_desire(:,2);
phi_desire = 0;

%% desire graph
plot(x_desire,y_desire,'.');
hold on;
for c = 1:length(xy_desire)
plot(xy_desire(c,1),xy_desire(c,2),'ro');
end

%% initial position
x = 2;
y = 2;
phi = 0;

x_dot = 0;
y_dot = 0;
phi_dot = 0;

%%% ego drawing
ego = [x,y];
ego_car = rectangle_draw(ego(1,1), ego(1,2), phi, 1, 0.8,1);
ego_car_x = ego_car(:,1);
ego_car_y = ego_car(:,2);
patch_temp=patch(ego_car_x,ego_car_y,'green');

%% simulation loop start
Kp = 10;

waypoint_check = 1;
away_distance = sqrt(8);
loop_count = 0;
while (loop_count < 2)  %% 몇번 돌지 정하기.
    %%% waypoint & loop count check    
    while away_distance <= 0.1
        waypoint_check = waypoint_check + 1;
        if waypoint_check > length(xy_desire)
            loop_count = loop_count + 1
            waypoint_check = 1;
        end
        x_d = x_desire(waypoint_check,1);
        y_d = y_desire(waypoint_check,1);
        away_distance = sqrt((x_d-x)^2 + (y_d-y)^2);
    end
    if waypoint_check > length(y_desire)
        break
    end
    
    %%% error set with desire point
    x_d = x_desire(waypoint_check,1);
    y_d = y_desire(waypoint_check,1);        
    x_tilda = x_d - x;
    y_tilda = y_d - y;
    phi_desire = atan2(y_tilda,x_tilda); %%어려운 부분, 추후 수정
    if (phi_desire > -pi && phi_desire < -pi/2) && (phi > pi/2 && phi < pi) 
        e = 1*((pi + phi_desire)+(pi - phi));
    else
        e = phi_desire - phi;
    end
    phi_desire_dot = phi_desire / dt;
    phi_desire_dot2 = phi_desire_dot / dt;
    
    %%% controller
    delta = atan(L*Kp*e/V);
    phi_dot = (V/L)*tan(delta);
        
    %%% update    
    phi = phi + phi_dot*dt;
    if phi > pi 
        phi = -1*(2*pi - phi);
    end
    x_dot = V*cos(phi);
    y_dot = V*sin(phi);
    x = x + x_dot*dt;
    y = y + y_dot*dt;
    away_distance = sqrt((x_desire(waypoint_check,1)-x)^2 + (y_desire(waypoint_check,1)-y)^2);    
    
    %%% ego drawing
    ego = [x,y];
    ego_car = rectangle_draw(ego(1,1), ego(1,2), phi, 1, 0.8,1);
    ego_car_x = ego_car(:,1);
    ego_car_y = ego_car(:,2);
    patch_temp.Vertices=([ego_car_x ego_car_y]);
    hold on
    plot(x,y,'g.');
    drawnow;
    axis([0 14 0 14])
    
end


 



    