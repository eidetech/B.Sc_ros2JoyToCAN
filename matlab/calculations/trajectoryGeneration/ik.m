close all;
L_wire = 8000;
wall_width = 2300;
d = wall_width;
R = 75;
gear_ratio = 10;
encoderCountsPerRev = 8192;

L_1 = sqrt(x_path_pos.^2 + z_path_pos.^2);
theta_1 = atan(z_path_pos/x_path_pos);

q1 = (L_wire-L_1)/R;
q1_t = -((z_path_vel+(x_path_pos.*cos(theta_1))./(sin(theta_1)))./(sin(theta_1)+(cos(theta_1).^2./sin(theta_1))));


L_2 = sqrt((d-x_path_pos).^2+z_path_pos.^2);
theta_2 = atan(z_path_pos/x_path_pos);

q2 = (L_wire-L_2)/R;
q2_t = -((z_path_vel+(x_path_pos.*cos(theta_2))./(sin(theta_2)))./(sin(theta_2)+(cos(theta_2).^2./sin(theta_2))));

figure(1)
plot(t_vect, q1/2*pi)
hold on
plot(t_vect, q2/2*pi)

figure(2)
plot(t_vect, q1_t/2*pi)
hold on