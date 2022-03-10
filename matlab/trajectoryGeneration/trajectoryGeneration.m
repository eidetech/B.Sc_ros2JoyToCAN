clc; clear; close all;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PARAMETERS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

sim = true; % toggle to start/stop playback animations of trajectory
plotting = true; % toggle all plotting except animation

paint_vel = 1000;                  % [mm/s] desired constant velocity when painting
ramp_vel = 500;                    % [mm/s] desired ramp velocity
ramp_dist = 100;                   % [mm]   distance from stationary to start point
ramp_time = ramp_dist/ramp_vel;    % [s]    time from ramp_dist to start point
turn_time = 2.5;                   % [s]    time to turn around to pass wall one level above
wall_width = 1500;                 % [mm]   width of wall
wall_height = 2000;                % [mm]   heigth of wall
wall_vStep = 500;                  % [mm]   vertical height step (vertical distance between horizontal lines
x_offset = 800;                    % [mm]   offset from (0,0) in x direction
z_offset = 200;                    % [mm]   offset from (0,0) in z direction

%%%%%%%%%%%%%%%%%%%%% CHECK FOR ERRORS IN VALUES %%%%%%%%%%%%%%%%%%%%%%%%%%
if(ramp_time > 0.20)
error('Ramp time to large!')
end

%%%%%%%%%%%%%%%%%%% POINTS TO GENERATE PATH BETWEEN %%%%%%%%%%%%%%%%%%%%%%%

% Coordinate system: xz (x = horizontal movement, z = vertical movement)
% pt = [x, z; x, z; ...]

pt = [
      % level 1
      x_offset + 0, z_offset + 0;
      x_offset + ramp_dist, z_offset + 0;
      x_offset + ramp_dist + wall_width, z_offset + 0;

      % level 1 -> 2
      x_offset + ramp_dist + wall_width, z_offset + wall_vStep;
      x_offset + ramp_dist + 0, z_offset + wall_vStep;

      % level 2 -> 3
      x_offset + ramp_dist + 0, z_offset + 2 * wall_vStep;
      x_offset + ramp_dist + wall_width, z_offset + 2 * wall_vStep;

      % level 3 -> 4
      x_offset + ramp_dist + wall_width, z_offset + 3 * wall_vStep;
      x_offset + ramp_dist + 0, z_offset + 3 * wall_vStep;  
      x_offset + 0, z_offset + 3 * wall_vStep;
];


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

%%%%%%%%%%%%%%%%%%%%%%%% TRAJECTORY GENERATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%

dt = 1e-2;
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
            if pt(i,1) == x_offset + ramp_dist  % right-hand turn
                v0_x = -paint_vel;
                v1_x = paint_vel;
                t1 = turn_time;
            else
                v0_x = paint_vel;               % left-hand turn
                v1_x = -paint_vel;
                t1 = turn_time;
            end
        elseif pt(i,1) < pt(i+1, 1) % last kind of trajectories is the straight lines, checking for direction
            v0_x = paint_vel;       % right
            v1_x = paint_vel;
            t1 = (wall_width)/paint_vel;      % calculates time based on constant velocity and distance (m//m/s = s)          
        else
            v0_x = -paint_vel;      % left
            v1_x = -paint_vel;
            t1 = (wall_width)/paint_vel;      % calculates time based on constant velocity and distance (m//m/s = s)   
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

L_wire = 8800;   % max length of wire on spool
d = wall_width;  % width between the two pulleys
R = 75;          % radius of the spool
gear_ratio = 10; % ratio of the gearbox between motor and pulley
cpr = 8192;      % encoder count per revolution

xA = 0;          % x offset from origo to pulley A
zA = 2000;       % z offset from origo to pulley A
xB = wall_width; % x offset from origo to pulley B
zB = 2000;       % z offset from origo to pulley B

d1_x = x_path_pos - xA; % x component of distance vector from origo to pulley A
d1_z = z_path_pos - zA; % z component of distance vector from origo to pulley A

d2_x = x_path_pos - xB; % x component of distance vector from origo to pulley B
d2_z = z_path_pos - zB; % z component of distance vector from origo to pulley B

L_1 = sqrt(d1_x.^2 + d1_z.^2); % actual length of wire L1 (from pulley A -> TP)
theta_1 = atan2(d1_z,d1_x);    % angle between horizontal line between pulleys and L1
q1 = (L_wire-L_1)./R;          % angular position of motor M1

% angular velocity of motor M0
q1_t = (z_path_vel + (x_path_vel.*cos(theta_1)) ./ (sin(theta_1))) ./ (sin(theta_1) + ((cos(theta_1).^2)) ./ (sin(theta_1)));

L_2 = sqrt(d2_x.^2 + d2_z.^2);   % actual length of wire L2 (from pulley B -> TP)
theta_2 =atan2(d2_z,d2_x); % angle between horizontal line between pulleys and L2
q2 = (L_wire-L_2)./R;            % angular position of motor M1

% angular velocity of motor M1
q2_t = (z_path_vel + (x_path_vel.*cos(theta_2)) ./ (sin(theta_2))) ./ (sin(theta_2) + ((cos(theta_2).^2)) ./ (sin(theta_2)));


% L_1 = sqrt(x_path_pos.^2 + z_path_pos.^2);
% theta_1 = atan(z_path_pos./x_path_pos);
% 
% q1 = (L_wire-L_1)./R;
% q1_t = -((z_path_vel+(x_path_vel.*cos(theta_1))./(sin(theta_1))) ./ (sin(theta_1)+((cos(theta_1)).^2./sin(theta_1))));
% 
% 
% L_2 = sqrt((d-x_path_pos).^2+z_path_pos.^2);
% theta_2 = atan(z_path_pos./(d-x_path_pos));
% 
% q2 = (L_wire-L_2)./R;
% q2_t = -((z_path_vel+(x_path_vel.*cos(theta_2))./(sin(theta_2))) ./ (sin(theta_2)+((cos(theta_2)).^2./sin(theta_2))));


if plotting
figure(3)
plot(t_vect, q1)
hold on
plot(t_vect, q2)
title('q')
legend('q_1', 'q_2')

figure(4)
plot(t_vect, q1_t)
hold on
plot(t_vect, q2_t)
title('q dot')
legend('q_1dot', 'q_2dot')

figure(5)
set(5,'Units','Centimeter')
set(5,'Position',[30 1 25 25]);
% cartesian velocity
subplot(2, 2, 1)
plot(t_vect, x_path_vel)
grid on
title('x vel')
xlabel('t [s]')
ylabel("x' [mm/s]")
ylim([min(x_path_vel)-0.05 max(x_path_vel)+0.05])

subplot(2, 2, 2)
plot(t_vect, z_path_vel)
grid on
title('z vel')
xlabel('t [s]')
ylabel("z' [mm/s]")
ylim([min(z_path_vel)-0.05 max(z_path_vel)+0.05])

% angular velocity
subplot(2, 2, 3)
plot(t_vect, q1_t)
grid on
title('q_1 vel')
xlabel('t [s]')
ylabel("q_1' [mm/s]")
ylim([min(q1_t)-0.05 max(q1_t)+0.05])

subplot(2, 2, 4)
plot(t_vect, q2_t)
grid on
title('q_2 vel')
xlabel('t [s]')
ylabel("q_2' [mm/s]")
ylim([min(q2_t)-0.05 max(q2_t)+0.05])
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PLOTS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if plotting
figure(1)
set(1,'Units','Centimeter')
set(1,'Position',[30 1 25 25]);
rows = 4;
cols = 2;

% position
subplot(rows, cols, 1)
plot(t_vect, x_path_pos)
grid on;
title('x pos')
xlabel('t [s]')
ylabel('x [mm]')
ylim([min(x_path_pos)-0.5 max(x_path_pos)+0.5])

subplot(rows, cols, 2)
plot(t_vect, z_path_pos)
grid on
title('z pos')
xlabel('t [s]')
ylabel('z [mm]')
ylim([min(z_path_pos)-0.5 max(z_path_pos)+0.5])

% velocity
subplot(rows, cols, 3)
plot(t_vect, x_path_vel)
grid on
title('x vel')
xlabel('t [s]')
ylabel("x' [mm/s]")
ylim([min(x_path_vel)-0.05 max(x_path_vel)+0.05])

subplot(rows, cols, 4)
plot(t_vect, z_path_vel)
grid on
title('z vel')
xlabel('t [s]')
ylabel("z' [mm/s]")
ylim([min(z_path_vel)-0.05 max(z_path_vel)+0.05])

% acceleration
subplot(rows, cols, 5)
plot(t_vect, x_path_acc)
grid on
title('x acc')
xlabel('t [s]')
ylabel('x" [mm/s^2]')
ylim([min(x_path_acc)-0.05 max(x_path_acc)+0.05])

subplot(rows, cols, 6)
plot(t_vect, z_path_acc)
grid on
title('z acc')
xlabel('t [s]')
ylabel('z" [mm/s^2]')
ylim([min(z_path_acc)-0.05 max(z_path_acc)+0.05])

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
    
    figure(2)
    set(2,'Units','Centimeter')
    set(2,'Position',[2 1 25 25]);
    xlabel('x [mm]')
    ylabel('z [mm]')
    
    for i=1:10:length(t_vect)
        cla

        hold on
            plot(x_path_pos, z_path_pos, '-.b', 'LineWidth', 2)
            plot(x_path_pos(1:i), z_path_pos(1:i), '-g', 'LineWidth', 3)
            plot(x_path_pos(i), z_path_pos(i), 'o', 'MarkerFaceColor', 'red', 'MarkerSize', 10)
        hold off

        text(0,2250,['p_x = ',num2str(round(x_path_pos(i),1))],'Color','red','FontSize',14)
        text(0,2000,['v_x = ',num2str(round(x_path_vel(i),2))],'Color','red','FontSize',14)
        text(0,1750,['a_x = ',num2str(round(x_path_acc(i),2))],'Color','red','FontSize',14)

        text(2000,2250,['p_z = ',num2str(round(z_path_pos(i),1))],'Color','red','FontSize',14)
        text(2000,2000,['v_z = ',num2str(round(z_path_vel(i),2))],'Color','red','FontSize',14)
        text(2000,1750,['a_z = ',num2str(round(z_path_acc(i),2))],'Color','red','FontSize',14)

        xlim([min(x_path_pos)-100, max(x_path_pos)+100])
        axis equal
        box on
        grid on
        movegui(2,[1450 600]);

        title(['t = ',num2str(round(t_vect(i),1))])

% Commented out due to Matlab lagging and crashing...
%         if (i==1)
%             title('Press any key','FontSize',20)
%             pause
%         end
        pause(0.01)
    end
end






