clc; clear; close all;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PARAMETERS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

sim = true; % toggle to start/stop playback animations of trajectory

plot_ik = true; % turn on/off inverse kinematics plots
plot_fk = true; % turn on/off forward kinematics plots
pva_plots = true; % turn on/off pos, vel, acc plots

paint_vel = 1;                     % [m/s] desired constant velocity when painting
ramp_dist = 0.6;                   % [m]   distance from stationary to start point
ramp_time = 1.2;                   % [s]   time from ramp_dist to start point
turn_time = 1;                     % [s]   time to turn around to pass wall one level above
wall_width = 2.000;                % [m]   width of wall
wall_height = 2.000;               % [m]   heigth of wall
wall_vStep = 0.500;                % [m]   vertical height step (vertical distance between horizontal lines
x_offset = 0.200;                  % [m]   offset from (0,0) in x direction
z_offset = 0.200;                  % [m]   offset from (0,0) in z direction

%%%%%%%%%%%%%%%%%%%%% CHECK FOR ERRORS IN VALUES %%%%%%%%%%%%%%%%%%%%%%%%%%
% Define error messages to appear for the user here (if impossible numbers
% are entered into the parameters above).

% Example:
% if(ramp_time > 0.20)
% error('Ramp time to large!')
% end

%%%%%%%%%%%%%%%%%%% POINTS TO GENERATE PATH BETWEEN %%%%%%%%%%%%%%%%%%%%%%%

% Coordinate system: xz (x = horizontal movement, z = vertical movement)
% pt = [x, z; x, z; ...]

% Initialize floor index and total height of the TP
floor = 0;
totalHeight = 0;

% starting path
pt = [
      x_offset + 0, z_offset + 0;
      x_offset + ramp_dist, z_offset + 0;
      x_offset + wall_width, z_offset + 0;
];

% all paths in-between start and ending path
while(totalHeight+wall_vStep <= wall_height)
    floor = floor+1;
    totalHeight = totalHeight + wall_vStep;
    if(totalHeight+2*wall_vStep > wall_height)
        break
    end
pt = [pt;      
      x_offset + wall_width, z_offset + floor*wall_vStep;
      x_offset + ramp_dist + 0, z_offset + floor*wall_vStep;
     ]; 
    floor = floor+1;
    totalHeight = totalHeight + wall_vStep;
    if(totalHeight+2*wall_vStep > wall_height)
        break
    end
pt = [pt;  
      x_offset + ramp_dist, z_offset + floor * wall_vStep;
      x_offset + wall_width, z_offset + floor * wall_vStep;
     ];
end

% ending path
pt = [pt;
      x_offset + wall_width, z_offset + floor * wall_vStep;
      x_offset + ramp_dist, z_offset + floor * wall_vStep;  
      x_offset + 0, z_offset + floor * wall_vStep;
];

% Example paths:
% pt = [
%         -ramp_dist, 0;
%         0, 0;
%         wall_width, 0;
%         wall_width, wall_vStep;
%         0, wall_vStep;
%         0, 2*wall_vStep;
%         wall_width, 2*wall_vStep;
%         wall_width, 3*wall_vStep;
%         0, 3*wall_vStep;
%         -ramp_dist, 3*wall_vStep
% ];

% pt = [
%       % level 1
%       x_offset + 0, z_offset + 0;
%       x_offset + ramp_dist, z_offset + 0;
%       x_offset + wall_width, z_offset + 0;
% 
%       % level 1 -> 2
%       x_offset + wall_width, z_offset + wall_vStep;
%       x_offset + ramp_dist + 0, z_offset + wall_vStep;
% 
%       % level 2 -> 3
%       x_offset + ramp_dist, z_offset + 2 * wall_vStep;
%       x_offset + wall_width, z_offset + 2 * wall_vStep;
% 
%       % level 3 -> 4
%       x_offset + wall_width, z_offset + 3 * wall_vStep;
%       x_offset + ramp_dist, z_offset + 3 * wall_vStep;  
%       x_offset + 0, z_offset + 3 * wall_vStep;
% ];

%%%%%%%%%%%%%%%%%%%%%%%% TRAJECTORY GENERATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%

dt = 1e-3;
N = length(pt)-1;   % number of paths to generate and put together

t0 = 0;             % calculates every path with initial time 0
v0_z = 0;           
v1_z = 0;           % initial and final velocities along z-axis is desired to be 0 for each path
a0_x = 0;
a1_x = 0;           % initial and final acceleration along both axis is desired to be 0
a0_z = 0;
a1_z = 0;

% loop through each set of points to generate trajectory
for i=1:N
    
    % set initial and final positions from the list of points
    x0 = pt(i,1);
    x1 = pt(i+1,1);
    z0 = pt(i,2);
    z1 = pt(i+1,2);
    
    if i == 1               % first trajectory (from stationary) will need zero initial velocity
        v0_x = 0;
        v1_x = paint_vel;
        t1 = ramp_time;        
    elseif i >= N           % last trajectory (to stationary) will need zero final velocity
        v0_x = -paint_vel;  % also assumes that robot stops on same side as it started (negative initial velocity)
        v1_x = 0;
        t1 = ramp_time;        
    else
        if pt(i,1) == pt(i+1,1) % if both x-points for the path is equal, it is a turn curve
            if pt(i,1) == x_offset+ramp_dist  % right-hand turn
                v0_x = -paint_vel;
                v1_x = paint_vel;
                t1 = turn_time;
            else
                v0_x = paint_vel;              % left-hand turn
                v1_x = -paint_vel;
                t1 = turn_time;
            end
        elseif pt(i,1) < pt(i+1, 1) % last kind of trajectories is the straight lines, checking for direction
            v0_x = paint_vel;       % right
            v1_x = paint_vel;
            t1 = (wall_width-ramp_dist)/paint_vel;      % calculates time based on constant velocity and distance (m//m/s = s)          
        else
            v0_x = -paint_vel;      % left
            v1_x = -paint_vel;
            t1 = (wall_width-ramp_dist)/paint_vel;      % calculates time based on constant velocity and distance (m//m/s = s)   
        end        
    end
    
    % generate the fifth order (quintic) polynomials (function in separate file)
    % x = position
    % x_t = velocity
    % x_tt = acceleration
    [x, x_t, x_tt] = Quintic(t0, t1, dt, x0, x1, v0_x, v1_x, a0_x, a1_x);
    [z, z_t, z_tt] = Quintic(t0, t1, dt, z0, z1, v0_z, v1_z, a0_z, a1_z);
    
    if i == 1   % initialize arrays with values from polynomial at first loop
        x_path_pos = x;
        x_path_vel = x_t;
        x_path_acc = x_tt;
        
        z_path_pos = z;
        z_path_vel = z_t;
        z_path_acc = z_tt;
        
        t_vect = (t0:dt:t1)';   % create time-vecor with matching sampletimes
        
    else        % append the other parts of both the path and time-vector
        x_path_pos = [x_path_pos; x];
        x_path_vel = [x_path_vel; x_t];
        x_path_acc = [x_path_acc; x_tt];
        
        z_path_pos = [z_path_pos; z];
        z_path_vel = [z_path_vel; z_t];
        z_path_acc = [z_path_acc; z_tt];
        
        t_vect = [t_vect; t_vect(end)+(0:dt:t1)'];
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%% INVERSE KINEMATICS %%%%%%%%%%%%%%%%%%%%%%%%%%%%

L_wire = 8.800;   % [m] max length of wire on spool
d = wall_width;   % [m] width between the two pulleys
R = 0.075;        % [m] radius of the spool
gear_ratio = 10;  % ratio of the gearbox between motor and pulley
cpr = 8192;       % encoder count per revolution

xA = 0;          % x [m] offset from origo to pulley A
zA = 2.000;      % z [m] offset from origo to pulley A
xB = wall_width; % x [m] offset from origo to pulley B
zB = 2.000;      % z [m] offset from origo to pulley B

d1_x = x_path_pos - xA; % x [m] component of distance vector from origo to pulley A
d1_z = z_path_pos - zA; % z [m] component of distance vector from origo to pulley A

d2_x = x_path_pos - xB; % x [m] component of distance vector from origo to pulley B
d2_z = z_path_pos - zB; % z [m] component of distance vector from origo to pulley B

L_1 = sqrt(d1_x.^2 + d1_z.^2); % [m] actual length of wire L1 (from pulley A -> TP)
theta_1 = atan2(d1_z,d1_x);    % [rad] angle between horizontal line between pulleys and L1
q_1 = (L_wire-L_1)./R;         % [rad] angular position of motor M1

% angular velocity of motor M0 [rad/s]
q1_t = -(z_path_vel + (x_path_vel.*cos(theta_1)) ./ (sin(theta_1))) ./ (sin(theta_1) + ((cos(theta_1).^2)) ./ (sin(theta_1))*R);

L_2 = sqrt(d2_x.^2 + d2_z.^2);   % [m] actual length of wire L2 (from pulley B -> TP)
theta_2 = pi + atan2(d2_z,d2_x); % [rad] angle between horizontal line between pulleys and L2
q_2 = (L_wire-L_2)./R;           % [rad] angular position of motor M1

% angular velocity of motor M1 [rad/s]
q2_t = -(z_path_vel + (x_path_vel.*cos(theta_2)) ./ (sin(theta_2))) ./ (sin(theta_2) + ((cos(theta_2).^2)) ./ (sin(theta_2))*R);

%%%%%%%%%%%%%%%%%%%%%%%%% INVERSE KINEMATICS PLOT %%%%%%%%%%%%%%%%%%%%%%%%%
if plot_ik
figure(1)

ik_rows = 3;
ik_columns = 2;

set(1,'Units','Centimeter')
set(1,'Position',[30 1 25 25]);
sgtitle('IK plot')
% cartesian velocity
subplot(ik_rows, ik_columns, 1)
plot(t_vect, x_path_vel)
grid on
title('x vel')
xlabel('t [s]')
ylabel("x' [m/s]")
ylim([min(x_path_vel)-0.05 max(x_path_vel)+0.05])

subplot(ik_rows, ik_columns, 2)
plot(t_vect, z_path_vel)
grid on
title('z vel')
xlabel('t [s]')
ylabel("z' [m/s]")
ylim([min(z_path_vel)-0.05 max(z_path_vel)+0.05])

% angular position
subplot(ik_rows, ik_columns, 3)
plot(t_vect, q_1)
grid on
title('q_1 pos')
xlabel('t [s]')
ylabel("q_1' [rad]")
ylim([min(q_1)-0.05 max(q_1)+0.05])

subplot(ik_rows, ik_columns, 4)
plot(t_vect, q_2)
grid on
title('q_2 pos')
xlabel('t [s]')
ylabel("q_2' [rad]")
ylim([min(q_2)-0.05 max(q_2)+0.05])

% angular velocity
subplot(ik_rows, ik_columns, 5)
plot(t_vect, q1_t)
grid on
title('q_1 vel')
xlabel('t [s]')
ylabel("q_1' [rad/s]")
ylim([min(q1_t)-0.05 max(q1_t)+0.05])

subplot(ik_rows, ik_columns, 6)
plot(t_vect, q2_t)
grid on
title('q_2 vel')
xlabel('t [s]')
ylabel("q_2' [rad/s]")
ylim([min(q2_t)-0.05 max(q2_t)+0.05])
end

%%%%%%%%%%%%%%%%%%%%%%%%%%% FORWARD KINEMATICS %%%%%%%%%%%%%%%%%%%%%%%%%%%%
theta_1_fk = acos((L_1.^2+d.^2-L_2.^2)./(2.*L_1.*d)); % [rad]
theta_2_fk = acos((L_2.^2+d.^2-L_1.^2)./(2.*L_2.*d)) + pi; % [rad]

x_fk = cos(theta_1_fk).*L_1; % [m]
z_fk = sin(theta_2_fk).*L_2 + zB; % [m]

q_1_fk = (L_wire-L_1)/R; % [rad]
q_2_fk = (L_wire-L_2)/R; % [rad]

q_1_fk_rev = q_1_fk/(2*pi); % [rev]
q_2_fk_rev = q_2_fk/(2*pi); % [rev]

%%%%%%%%%%%%%%%%%%%%%%%%% FORWARD KINEMATICS PLOT %%%%%%%%%%%%%%%%%%%%%%%%%
if plot_fk
figure(2)
set(2,'Units','Centimeter')
set(2,'Position',[30 1 25 25]);
fk_rows = 4;
fk_columns = 2;
sgtitle('FK plot')

subplot(fk_rows,fk_columns,1)
plot(t_vect, x_path_pos)
hold on
plot(t_vect,x_fk)
legend('x_{traj}', 'x_{fk}')
title('x trajectory and FK')

subplot(fk_rows,fk_columns,2)
plot(t_vect, z_path_pos)
hold on
plot(t_vect,z_fk)
legend('z_{traj}', 'z_{fk}')
title('z trajectory and FK')

subplot(fk_rows,fk_columns,3)
plot(t_vect,theta_1_fk)
legend('theta_1')
title('theta_1 FK')

subplot(fk_rows,fk_columns,4)
plot(t_vect,theta_2_fk)
legend('theta_2')
title('theta_2 FK')

subplot(fk_rows,fk_columns,5)
plot(t_vect,q_1_fk)
hold on
plot(t_vect,q_1)
legend('q_1')
title('q_1 FK')

subplot(fk_rows,fk_columns,6)
plot(t_vect,q_2_fk)
hold on
plot(t_vect,q_2)
legend('q_2')
title('q_2 FK')

subplot(fk_rows,fk_columns,7)
plot(t_vect,q_1_fk_rev)
legend('q_1 rev')
title('q_1 FK rev')

subplot(fk_rows,fk_columns,8)
plot(t_vect,q_2_fk_rev)
legend('q_2 rev')
title('q_2 FK rev')
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PVA PLOTS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if pva_plots
figure(3)
set(3,'Units','Centimeter')
set(3,'Position',[30 1 25 25]);
sgtitle('PVA plot')
rows = 4;
cols = 2;

% position
subplot(rows, cols, 1)
plot(t_vect, x_path_pos)
grid on;
title('x pos')
xlabel('t [s]')
ylabel('x [m]')
ylim([min(x_path_pos)-0.5 max(x_path_pos)+0.5])

subplot(rows, cols, 2)
plot(t_vect, z_path_pos)
grid on
title('z pos')
xlabel('t [s]')
ylabel('z [m]')
ylim([min(z_path_pos)-0.5 max(z_path_pos)+0.5])

% velocity
subplot(rows, cols, 3)
plot(t_vect, x_path_vel)
grid on
title('x vel')
xlabel('t [s]')
ylabel("x' [m/s]")
ylim([min(x_path_vel)-0.05 max(x_path_vel)+0.05])

subplot(rows, cols, 4)
plot(t_vect, z_path_vel)
grid on
title('z vel')
xlabel('t [s]')
ylabel("z' [m/s]")
ylim([min(z_path_vel)-0.05 max(z_path_vel)+0.05])

% acceleration
subplot(rows, cols, 5)
plot(t_vect, x_path_acc)
grid on
title('x acc')
xlabel('t [s]')
ylabel('x" [m/s^2]')
ylim([min(x_path_acc)-0.05 max(x_path_acc)+0.05])

subplot(rows, cols, 6)
plot(t_vect, z_path_acc)
grid on
title('z acc')
xlabel('t [s]')
ylabel('z" [m/s^2]')
ylim([min(z_path_acc)-0.05 max(z_path_acc)+0.05])

% p, v, a on x-axis
subplot(rows, cols, 7)
hold on
plot(t_vect, x_path_pos)
plot(t_vect, x_path_vel)
plot(t_vect, x_path_acc)
grid on
legend('Pos', 'Vel', 'Acc')
title('x, p v a')
xlabel('t [s]')
ylabel('x')
ylim([min(x_path_acc)-0.05 max(x_path_pos)+0.05])
hold off

% p, v, a on z-axis
subplot(rows, cols, 8)
hold on
plot(t_vect, z_path_pos)
plot(t_vect, z_path_vel)
plot(t_vect, z_path_acc)
legend('Pos', 'Vel', 'Acc')
grid on
title('z, p v a')
xlabel('t [s]')
ylabel('z')
ylim([min(z_path_acc)-0.05 max(z_path_pos)+0.05])
hold off

end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ANIMATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% NOT REAL TIME PLAYBACK
% Finish trajectory playback completely, or start in and stop it in IDE

if sim
    
    figure(4)
    set(4,'Units','Centimeter')
    set(4,'Position',[100 1 25 25]);
    xlabel('x [m]')
    ylabel('z [m]')
    
    for i=1:10:length(t_vect)
        cla

        hold on
            plot(x_path_pos, z_path_pos, '-.b', 'LineWidth', 2)
            plot(x_path_pos(1:i), z_path_pos(1:i), '-g', 'LineWidth', 3)
            plot(x_path_pos(i), z_path_pos(i), 'o', 'MarkerFaceColor', 'red', 'MarkerSize', 10)

            %%%%% Red frame for visual feedback on area to be painted %%%%%
            % LV
            plot([0+x_offset+ramp_dist,0+x_offset+ramp_dist], [0,wall_height], 'r', 'LineWidth', 2)
            % RV
            plot([x_offset+wall_width,x_offset+wall_width], [0,wall_height], 'r', 'LineWidth', 2)

            % BH
            plot([x_offset+ramp_dist, x_offset+wall_width], [0,0], 'r', 'LineWidth', 2)
            % TH
            plot([x_offset+ramp_dist, x_offset+wall_width], [wall_height,wall_height], 'r', 'LineWidth', 2)
        hold off

        text(0,2250,['p_x = ',num2str(round(x_path_pos(i),1))],'Color','red','FontSize',14)
        text(0,2000,['v_x = ',num2str(round(x_path_vel(i),2))],'Color','red','FontSize',14)
        text(0,1750,['a_x = ',num2str(round(x_path_acc(i),2))],'Color','red','FontSize',14)

        text(2000,2250,['p_z = ',num2str(round(z_path_pos(i),1))],'Color','red','FontSize',14)
        text(2000,2000,['v_z = ',num2str(round(z_path_vel(i),2))],'Color','red','FontSize',14)
        text(2000,1750,['a_z = ',num2str(round(z_path_acc(i),2))],'Color','red','FontSize',14)

        xlim([min(x_path_pos)-1, max(x_path_pos)+1])
        axis equal
        box on
        grid on
        %movegui(2,[2000 100]);

        title(['t = ',num2str(round(t_vect(i),1))])

% Comment out if statement below if MATLAB is lagging/crashing...
        if (i==1)
            title('Press any key','FontSize',20)
            pause
        end
        pause(0.01)
    end
end





