clc; clear

%%%%%%%%%%%%%%%% INVERSE KINEMATICS (TRIG APPROACH) %%%%%%%%%%%%%%%%%%%%%%%
wall_width = 1.500;
outer_frame_width = 2.800;                  % [m] outermost frame width defined by telescopic poles
outer_frame_height = 2.625;   

x_offset = 0.150;                           % [m]   offset from (0,0) in x direction
z_offset = 0.200;                           % [m]   offset from (0,0) in z direction

d = wall_width;               % [m] width between the two pulleys
R = 125.5/(2*1000);           % [m] radius of the spool (125.5mm diameter spool, divided by 2 to get radius and then converted to [m]
L_wire = 19.5*2*pi*R;         % [m] max length of wire on spool

xA = 0;                       % x [m] offset from origo to pulley A
zA = outer_frame_height;      % z [m] offset from origo to pulley A
xB = outer_frame_width;       % x [m] offset from origo to pulley B
zB = outer_frame_height;      % z [m] offset from origo to pulley B

d1_x = x_offset - xA;       % x [m] component of distance vector from origo to pulley A
d1_z = -z_offset + zA;    % z [m] component of distance vector from origo to pulley A

d2_x = x_offset - xB;       % x [m] component of distance vector from origo to pulley B
d2_z = -z_offset + zB;    % z [m] component of distance vector from origo to pulley B

L_1 = sqrt(d1_x.^2 + d1_z.^2);% [m] actual length of wire L1 (from pulley A -> TCP)
theta_1 = atan2(d1_z,d1_x);   % [rad] angle between horizontal line between pulleys and L1
q_1 = (L_wire-L_1)./R;        % [rad] angular position of motor M1

q_1_rev = q_1/(2*pi)*10


L_2 = sqrt(d2_x.^2 + d2_z.^2);   % [m] actual length of wire L2 (from pulley B -> TCP)
theta_2 = atan2(d2_z,d2_x); % [rad] angle between horizontal line between pulleys and L2
q_2 = (L_wire-L_2)./R;          % [rad] angular position of motor M1

q_2_rev = q_2/(2*pi)*10
