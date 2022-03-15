clc; clear; close all;
% Boolean to stop while loop when height is reached
topNotReached = true;

% Define frame size
w = 2350; % width
h = 2000; % height
% Vertical distance between horizontal lines
vStep = 150;

% Resolution/stepping of x and z axis (z for semicircle points)
xRes = 50;
zRes = 10;

% Define start height and width (not implemented zet)
startHeight = 0;
startWidth = 0;

% Set intitial conditions
side = 0; % 0 = left to right, 1 = right to left

% ### DONT EDIT ANYTHING BELOW ###
if(side == 0)
    xPos = 0;
    zPos = 0;
elseif(side == 1)
    xPos = w;
    zPos = 0;
end

% Points for semicircle plotting
th = linspace(-pi/2, pi/2, zRes);

% Initialize counters
i = 1;
t = 1;

xC = [];
zC = [];

% Configure figure settings
fig = figure;
movegui(fig,[1400 1500]);
hold on

% Frame box (red)
plot([0,w],[0,0], 'r')
plot([0,w],[h,h], 'r')
plot([0,0],[0,h], 'r')
plot([w,w],[0,h], 'r')

% Figure window size
xlim([-1000 w+1000])
zlim([-100  h+100])

while topNotReached
    if(side == 0)
        while (xPos < w)
            plot(xPos, zPos, 'o')
            xPos=xPos+xRes;
            xC(end+1) = xPos;
            zC(end+1) = zPos;
        end
    elseif(side == 1)
        while (xPos > 0)
            plot(xPos, zPos, 'o')
            xPos=xPos-xRes;
            xC(end+1) = xPos;
            zC(end+1) = zPos;
        end
    end

    if(vStep*t >= h)
        topNotReached = false;
        break
    end

    if(side == 0)
        k = 1;
        while(xPos >= w && zPos < vStep*t)
            R = vStep/2;
            xPos = R*cos(th(k))+w;
            zPos = R*sin(th(k))+(vStep*i)/2;
            plot(xPos,zPos, '*')
            k = k+1;
            xC(end+1) = xPos;
            zC(end+1) = zPos;
        end
        % Change side and update counter
        side = 1;
        i=i+1;
    elseif(side == 1)
        k = 1;
        while(zPos < t*vStep)
            R = vStep/2;
            xPos = R*cos(th(k));
            zPos = R*sin(th(k))+((i)*vStep)/2;
            plot(-xPos,zPos, '*')
            k = k+1;
            xC(end+1) = -xPos;
            zC(end+1) = zPos;
        end
        % Change side and update counter
        side = 0;
        i=i+1;
    end
% Update counters
i=i+1;
t=t+1;
%pause(1)
end

% Boundry lines (blue)
plot([min(xC),min(xC)],[0,h], 'b')
plot([max(xC),max(xC)],[0,h], 'b')

% ### Line plot using xC and zC ###
% Configure figure settings
fig2 = figure;
movegui(fig2,[1000 1500]);
hold on

% Frame box (red)
plot([0,w],[0,0], 'r')
plot([0,w],[h,h], 'r')
plot([0,0],[0,h], 'r')
plot([w,w],[0,h], 'r')

xlim([-1000 w+1000])
ylim([-100 h+100])
for u = 1:length(xC)
plot(xC(u), zC(u), '*')
hold on 
pause(0.01)
end

% Boundry lines (blue)
plot([min(xC),min(xC)],[0,h], 'b')
plot([max(xC),max(xC)],[0,h], 'b')
%plot(xC,zC)

csvwrite('zC.csv',zC)