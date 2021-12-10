clear all;

%% parameter
m = 10; % kg
I = 5; % kg*m^2
R = 0.5; % m
r = 0; % m
d = 2*r;
v_r = 0.5; % m/s

dt = 0.01;


K = [10 5 4]';
k1 = K(1);
k2 = K(2);
k3 = K(3);
K4 = [0 40; 40 0];
N_hidden_neurons = 10;
F = [0 10; 10 0];
G = [0 10; 10 0];
constant_k = 0.1;




% desired line
% y_desire = t;       %line case ii x=y;
% x_desire = t;
% 
% xy_dot_desire = [1; 1]; %2x1 matrix         line x=y case ii;
% 
% xy_2_dot_desire = [0; 0]; %2x1 matrix
% 
% xy_desire =[x_desire y_desire]';

%% desired position
xy_desire = [4 4; 8 8; 12 12; 16 16];
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
x = 0;
y = 0;
phi = 0;
V = [0 0]';

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
while (waypoint_check < 4)  %% 몇번 돌지 정하기.
    %%% waypoint & loop count check    
    while away_distance <= 0.1
        waypoint_check = waypoint_check + 1;
%         if waypoint_check > length(xy_desire)
%             loop_count = loop_count + 1
%             waypoint_check = 1;
%         end
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
    phi_d = atan2(y_d - y,x_d - x); %%어려운 부분, 추후 수정
    if (phi_d > -pi && phi_d < -pi/2) && (phi > pi/2 && phi < pi) 
        phi_tilda = 1*((pi + phi_d)+(pi - phi));
    else
        phi_tilda = phi_d - phi;
    end
    
    E_p = [cos(phi)  sin(phi) 0; -sin(phi) cos(phi) 0; 0 0 1] * [x_d-x; y_d-y; phi_tilda];
    error_1 = E_p(1);
    error_2 = E_p(2);
    error_3 = E_p(3);
    
    w_r = phi_d / dt;
    phi_dot = phi/dt;
    
    E_p_dot = [V(2)*error_2 - V(1) + v_r*cos(error_3);
               -V(2)*error_1 + v_r*sin(error_3);
               w_r - phi_dot];
           
    
    x_d_dot = v_r*cos(phi_desire);
    y_d_dot = v_r*sin(phi_desire);
    phi_d_dot = w_r;
    
    %% Lagrangian Ecuation
    S = [cos(phi) -d*sin(phi); sin(phi) d*cos(phi); 0 1];
%     Mq = [m 0 m*d*sin(phi); 0 m -m*d*cos(phi); m*d*sin(phi) -m*d*cos(phi) I];
%     Vm = [0 0 m*d*phi_dot*cos(phi); 0 0 m*d*phi_dot*sin(phi); 0 0 0];
%     Gq = 0;
%     Bq = 1/r * [cos(phi) cos(phi); sin(phi) sin(phi); R -R];
%     
%     At = [-sin(phi) cos(phi) -d];
%     gamma = -m*(x_c_dot*cos(phi) + y_c_dot*sin(phi))*phi_dot;
    
    
    V_c = [v_r*cos(error_3) + k1*error_1;
           w_r + k2*v_r*error_2 + k3*v_r*sin(error_3)];
    
    V_c_dot = [k1 0 -v_r*sin(error_3); 0  k2*v_r k3*v_r*cos(error_3)]*E_p_dot;
           
       
    E_c = V_c - V;
    
    
    
    %% update
    q_dot = S*V_c;
        %%% update    
    phi_dot = q_dot(3);
    
    q = [x y phi];
    
    q = q + q_dot*dt;
    x = q(1);
    y = q(2);    
    phi = q(3);
    if phi > pi 
        phi = -1*(2*pi - phi);
    end
    
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
    axis([-4 14 -4 14])
    
end


 



    