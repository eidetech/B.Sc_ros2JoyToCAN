clc; clear; close all;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PARAMS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

sim = false; % toggle to start/stop playback animations of trajectory

paint_vel = 1000;   % [mm/s] desired constant velocity when painting
ramp_dist = 100;    % [mm]   distance from stationary to start point
ramp_time = 0.25;   % [s]    time from ramp_dist to start point
turn_time = 0.8;    % [s]    time to turn around to pass wall one level above
wall_width = 2300;  % [mm]   width of wall
wall_height = 2000; % [mm]   heigth of wall
wall_vStep = 500;   % [mm]   vertical height step (vertical distance between horizontal lines

%%%%%%%%%%%%%%%%%%% POINTS TO GENERATE PATH BETWEEN %%%%%%%%%%%%%%%%%%%%%%%

% Coordinate system: xz (x = horizontal movement, z = vertical movement
% pt = [x, z; x, z; ...]

pt = [
        -ramp_dist, 0;
        0, 0;
        wall_width, 0;
        wall_width, wall_vStep;
        0, wall_vStep;
        0, 2*wall_vStep;
        wall_width, 2*wall_vStep;
        wall_width, 3*wall_vStep;
        0, 3*wall_vStep;
        -ramp_dist, 3*wall_vStep
];

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
        if pt(i,1) == pt(i+1,1)     % if both x-points for the path is equal, it is a turn curve
            if pt(i,1) == 0         % right-hand turn
                v0_x = -paint_vel;
                v1_x = paint_vel;
                t1 = turn_time;
            else
                v0_x = paint_vel;   % left-hand turn
                v1_x = -paint_vel;
                t1 = turn_time;
            end
        elseif pt(i,1) < pt(i+1, 1) % last kind of trajectories is the straight lines, checking for direction
            v0_x = paint_vel;       % right
            v1_x = paint_vel;
            t1 = wall_width/paint_vel;      % calculates time based on constant velocity and distance (m//m/s = s)          
        else
            v0_x = -paint_vel;      % left
            v1_x = -paint_vel;
            t1 = wall_width/paint_vel;      % calculates time based on constant velocity and distance (m//m/s = s)   
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


L_1 = sqrt(x_path_pos.^2 + z_path_pos.^2);
L_wire = 8000;
theta_1 = atan(z_path_pos/x_path_pos);

q1_t = -((z_path_vel+(x_path_pos.*cos(theta_1))./(sin(theta_1)))./(sin(theta_1)+(cos(theta_1).^2./sin(theta_1))));


L_2 = sqrt(x_path_pos.^2 + z_path_pos.^2);
theta_2 = atan(z_path_pos/x_path_pos);

q2_t = -((z_path_vel+(x_path_pos.*cos(theta_2))./(sin(theta_2)))./(sin(theta_2)+(cos(theta_2).^2./sin(theta_2))));


figure(3)
hold on
plot(t_vect, q1_t)

plot(t_vect, q2_t)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PLOTS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
title('y vel')
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

%%%%%%%%%%%%%%%%%%%%%%%%% ANIMATION %%%%%%%%%%%%%%%%%%%%%%%%%

% NOT REAL TIME PLAYBACK
% Finish trajectory playback completely, or start in and stop it in IDE

if sim
    
    figure(2)
    set(2,'Units','Centimeter')
    set(2,'Position',[2 1 25 25]);
    xlabel('x [m]')
    ylabel('z [m]')
    
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

        xlim([min(x_path_pos)-1, max(x_path_pos)+1])
        axis equal
        box on
        grid on

        title(['t = ',num2str(round(t_vect(i),1))])

        if (i==1)
            title('Press any key','FontSize',20)
            pause
        end
        pause(0.01)
    end
end






