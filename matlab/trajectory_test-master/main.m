clc; clear; close all;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PARAMS %%%%%%%%%%%%%%%%%%%%%%%%%%%%

sim = true; % toggle to start/stop playback animations of trajectory

paint_vel = 0.1;    % desired constant velocity when painting
ramp_dist = 1;      % distance from stationary to start point
ramp_time = 10.0;   % time from ramp_dist to start point
turn_time = 10;     % time to turn around to pass wall one level above
wall_width = 8;     % width of wall


% points to generate path between, ==> (x, y)
pt = [
        -ramp_dist, 0;
        0, 0;
        wall_width, 0;
        wall_width, 1;
        0, 1;
        0, 2;
        wall_width, 2;
        wall_width, 3;
        0, 3;
        -ramp_dist, 3
]

%%%%%%%%%%%%%%%%%%%%% TRAJECTORY GENERATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%

dt = 1e-2;
N = length(pt)-1;   % number of pahts to generate and put together

t0 = 0;             % calculates every path with initial time 0
v0_y = 0;           
v1_y = 0;           % initial and final velocities along y-axis is desired to be 0 for each path
a0_x = 0;
a1_x = 0;           % initial and final acceleration along both axis is desired to be 0
a0_y = 0;
a1_y = 0;

% loop through each set of points to generate trajectory
for i=1:N
    
    % set initial and final positions from the list of point
    x0 = pt(i,1);
    x1 = pt(i+1,1);
    y0 = pt(i,2);
    y1 = pt(i+1,2);
    
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
            t1 = wall_width/paint_vel;      % calculates time based on constant velocty and distance            
        else
            v0_x = -paint_vel;      % left
            v1_x = -paint_vel;
            t1 = wall_width/paint_vel;      % calculates time based on constant velocty and distance      
        end        
    end
    
    % generate the fifth order (quintic) polynomials (function in separate file)
    [x, x_t, x_tt] = Quintic(t0, t1, dt, x0, x1, v0_x, v1_x, a0_x, a1_x);
    [y, y_t, y_tt] = Quintic(t0, t1, dt, y0, y1, v0_y, v1_y, a0_y, a1_y);
    
    if i == 1   % initialize arrays with values from polynomial at first loop
        x_path_pos = x;
        x_path_vel = x_t;
        x_path_acc = x_tt;
        
        y_path_pos = y;
        y_path_vel = y_t;
        y_path_acc = y_tt;
        
        t_vect = (t0:dt:t1)';   % create time-vecor with matching sampletimes
        
    else        % append the other parts of both the path and time-vector
        x_path_pos = [x_path_pos; x];
        x_path_vel = [x_path_vel; x_t];
        x_path_acc = [x_path_acc; x_tt];
        
        y_path_pos = [y_path_pos; y];
        y_path_vel = [y_path_vel; y_t];
        y_path_acc = [y_path_acc; y_tt];
        
        t_vect = [t_vect; t_vect(end)+(0:dt:t1)'];
    end
    
    
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PLOTS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure(1)
set(1,'Units','Centimeter')
set(1,'Position',[30 1 25 25]);
rows = 3;
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
plot(t_vect, y_path_pos)
grid on
title('y pos')
xlabel('t [s]')
ylabel('y [m]')
ylim([min(y_path_pos)-0.5 max(y_path_pos)+0.5])

% velocity
subplot(rows, cols, 3)
plot(t_vect, x_path_vel)
grid on
title('x vel')
xlabel('t [s]')
ylabel("x' [m/s]")
ylim([min(x_path_vel)-0.05 max(x_path_vel)+0.05])

subplot(rows, cols, 4)
plot(t_vect, y_path_vel)
grid on
title('y vel')
xlabel('t [s]')
ylabel("y' [m/s]")
ylim([min(y_path_vel)-0.05 max(y_path_vel)+0.05])

% acceleration
subplot(rows, cols, 5)
plot(t_vect, x_path_acc)
grid on
title('x acc')
xlabel('t [s]')
ylabel('x" [m/s^2]')
ylim([min(x_path_acc)-0.05 max(x_path_acc)+0.05])

subplot(rows, cols, 6)
plot(t_vect, y_path_acc)
grid on
title('y acc')
xlabel('t [s]')
ylabel('y" [m/s^2]')
ylim([min(y_path_acc)-0.05 max(y_path_acc)+0.05])

%%%%%%%%%%%%%%%%%%%%%%%%% ANIMATION %%%%%%%%%%%%%%%%%%%%%%%%%

% NOT REAL TIME PLAYBACK
% Finish trajectory playback completely, or start in and stop it in IDE

if sim
    
    figure(2)
    set(2,'Units','Centimeter')
    set(2,'Position',[2 1 25 25]);
    xlabel('x [m]')
    ylabel('y [m]')
    
    for i=1:10:length(t_vect)
        cla

        hold on
            plot(x_path_pos, y_path_pos, '-.b', 'LineWidth', 2)
            plot(x_path_pos(1:i), y_path_pos(1:i), '-g', 'LineWidth', 3)
            plot(x_path_pos(i), y_path_pos(i), 'o', 'MarkerFaceColor', 'red', 'MarkerSize', 10)
        hold off

        text(0,7,['p_x = ',num2str(round(x_path_pos(i),1))],'Color','red','FontSize',14)
        text(0,5.5,['v_x = ',num2str(round(x_path_vel(i),2))],'Color','red','FontSize',14)
        text(0,4,['a_x = ',num2str(round(x_path_acc(i),2))],'Color','red','FontSize',14)

        text(4,7,['p_y = ',num2str(round(y_path_pos(i),1))],'Color','red','FontSize',14)
        text(4,5.5,['v_y = ',num2str(round(y_path_vel(i),2))],'Color','red','FontSize',14)
        text(4,4,['a_y = ',num2str(round(y_path_acc(i),2))],'Color','red','FontSize',14)

        xlim([min(x_path_pos)-1, max(x_path_pos)+1])
        axis equal
        box on
        grid on

        title(['t = ',num2str(round(t_vect(i),1))])

        if (i==1)
            title('Press any key','FontSize',20)
            pause
        end
        pause(0.001)
    end
end





