clc; clear all; close all;
xC = csvread("xC.csv")
zC = csvread("zC.csv")



%total time
Tp = 19

%xyz positions that the robot should go through
    x = [2, 2, 1.6, 1.6, 2];
    y = [0.5, -0.5, 0.5, 0, 0.5];
    z = [1.6, 1.6, 0.9, 0.9, 1.6];

    %calculate the length of each path step
    path_len = [    0;
                    norm([x(2)-x(1) y(2)-y(1) z(2)-z(1)]);
                    norm([x(3)-x(2) y(3)-y(2) z(3)-z(2)]);
                    norm([x(4)-x(3) y(4)-y(3) z(4)-z(3)]);
                    norm([x(5)-x(4) y(5)-y(4) z(5)-z(4)])
    ];

    % sum the total path length
    path_len_tot = sum(path_len);

    % calculate how long each path step takes
    timeMat = path_len/path_len_tot * Tp

    % calculate the time for when the robot should be at each position
        for i = 2:length(timeMat)
        timeMat(i) = timeMat(i) + timeMat(i-1);
        end

        % print list of deadlines
        timeMat