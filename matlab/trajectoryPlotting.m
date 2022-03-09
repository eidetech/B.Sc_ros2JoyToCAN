clc; clear; close all;

% x_0 -> x_1
x_0 = 0;       % [mm]
x_1 = 1000;   % [mm]
xDot_0 = 0;    % [mm/s]
xDot_1 = 50;   % [mm/s]
xDotdot_0 = 0; % [mm/s^2]
xDotdot_1 = 10; % [mm/s^2]

% z_0 -> z_1
z_0 = 0;       % [mm]
z_1 = 0;   % [mm]
zDot_0 = 0;    % [mm/s]
zDot_1 = 0;   % [mm/s]
zDotdot_0 = 0; % [mm/s^2]
zDotdot_1 = 0; % [mm/s^2]

v = 1000; % [mm/s]

t_0 = 0; % Start time
t_1 = (x_1-x_0)/(v) % Stop time

T = [1  t_0  t_0^2  t_0^3    t_0^4    t_0^5;
     1  t_1  t_1^2  t_1^3    t_1^4    t_1^5;
     0   1   2*t_0  3*t_0^2  4*t_0^3  5*t_0^4;
     0   1   2*t_1  3*t_1^2  4*t_1^4  5*t_1^4;
     0   0     2    6*t_0    12*t_0^2 20*t_0^3;
     0   0     2    6*t_1    12*t_1^2 20*t_1^3];

X = [x_0 x_1 xDot_0 xDot_1 xDotdot_0 xDotdot_1]';
Z = [z_0 z_1 zDot_0 zDot_1 zDotdot_0 zDotdot_1]';

A = T\X;
B = T\Z;

dt = 1e-4;

idx = 1;

for t=0:dt:t_1

    x_plt(idx) = A(1) + A(2)*t + A(3)*t^2 + A(4)*t^3 + A(5)*t^4 + A(6)*t^5;
    z_plt(idx) = B(1) + B(2)*t + B(3)*t^2 + B(4)*t^3 + B(5)*t^4 + B(6)*t^5;

    %x_plt_dot(idx) = A(2) + 2*A(3)*t + 3*A(4)*t^2 + 4*A(5)*t^3 + 5*A(6)*t^4;
    %z_plt_dot(idx) = B(2) + 2*B(3)*t + 3*B(4)*t^2 + 4*B(5)*t^3 + 5*B(6)*t^4;

    idx = idx + 1;

end

plot(x_plt, z_plt)
hold on
grid on

clear A B

[A, B] = genTraj(1000,1000,0,2,0,2,0,1000,0,2,0,2,1,2);

for t=1:dt:2

    x_plt(idx) = A(1) + A(2)*t + A(3)*t^2 + A(4)*t^3 + A(5)*t^4 + A(6)*t^5;
    z_plt(idx) = B(1) + B(2)*t + B(3)*t^2 + B(4)*t^3 + B(5)*t^4 + B(6)*t^5;

    %x_plt_dot(idx) = A(2) + 2*A(3)*t + 3*A(4)*t^2 + 4*A(5)*t^3 + 5*A(6)*t^4;
    %z_plt_dot(idx) = B(2) + 2*B(3)*t + 3*B(4)*t^2 + 4*B(5)*t^3 + 5*B(6)*t^4;

    idx = idx + 1;

end

plot(x_plt, z_plt)
hold on

xlim([-500 1500])
ylim([-1000 6000])


function [A, B] = genTraj(x_0, x_1, xDot_0, xDot_1, xDotdot_0, xDotdot_1, z_0, z_1, zDot_0, zDot_1, zDotdot_0, zDotdot_1, t_0, t_1)
%genTraj Summary of this function goes here
%   Detailed explanation goes here
T = [1  t_0  t_0^2  t_0^3    t_0^4    t_0^5;
     1  t_1  t_1^2  t_1^3    t_1^4    t_1^5;
     0   1   2*t_0  3*t_0^2  4*t_0^3  5*t_0^4;
     0   1   2*t_1  3*t_1^2  4*t_1^4  5*t_1^4;
     0   0     2    6*t_0    12*t_0^2 20*t_0^3;
     0   0     2    6*t_1    12*t_1^2 20*t_1^3];

X = [x_0 x_1 xDot_0 xDot_1 xDotdot_0 xDotdot_1]';
Z = [z_0 z_1 zDot_0 zDot_1 zDotdot_0 zDotdot_1]';

A = T\X;
B = T\Z;
end