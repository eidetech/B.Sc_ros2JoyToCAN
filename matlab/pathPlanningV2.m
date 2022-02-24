clc; clear; close all;
% Boolean to stop while loop when height is reached
topNotReached = true;

% Define frame size
w = 2350; % width
h = 10000; % height
% Vertical distance between horizontal lines
vStep = 500;

% Resolution/stepping of x and y axis (y for semicircle points)
xRes = 50;
yRes = 50;

% Define start height and width (not implemented yet)
startHeight = 0;
startWidth = 0;

% Set intitial conditions
side = 0; % 0 = left to right, 1 = right to left

% ### DONT EDIT ANYTHING BELOW ###
if(side == 0)
    xPos = 0;
    yPos = 0;
elseif(side == 1)
    xPos = w;
    yPos = 0;
end

% Points for semicircle plotting
th = linspace(-pi/2, pi/2, yRes);

% Initialize counters
i = 1;
t = 1;

xC = [];
yC = [];

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
ylim([-100  h+100])

while topNotReached
    if(side == 0)
        while (xPos < w)
            plot(xPos, yPos, 'o')
            xPos=xPos+xRes;
            xC(end+1) = xPos;
            yC(end+1) = yPos;
        end
    elseif(side == 1)
        while (xPos > 0)
            plot(xPos, yPos, 'o')
            xPos=xPos-xRes;
            xC(end+1) = xPos;
            yC(end+1) = yPos;
        end
    end

    if(vStep*t >= h)
        topNotReached = false;
        break
    end

    if(side == 0)
        k = 1;
        while(xPos >= w && yPos < vStep*t)
            R = vStep/2;
            xPos = R*cos(th(k))+w;
            yPos = R*sin(th(k))+(vStep*i)/2;
            plot(xPos,yPos, '*')
            k = k+1;
            xC(end+1) = xPos;
            yC(end+1) = yPos;
        end
        % Change side and update counter
        side = 1;
        i=i+1;
    elseif(side == 1)
        k = 1;
        while(yPos < t*vStep)
            R = vStep/2;
            xPos = R*cos(th(k));
            yPos = R*sin(th(k))+((i)*vStep)/2;
            plot(-xPos,yPos, '*')
            k = k+1;
            xC(end+1) = -xPos;
            yC(end+1) = yPos;
        end
        % Change side and update counter
        side = 0;
        i=i+1;
    end
% Update counters
i=i+1;
t=t+1;
end

% Boundry lines (blue)
plot([min(xC),min(xC)],[0,h], 'b')
plot([max(xC),max(xC)],[0,h], 'b')

% ### Line plot using xC and yC ###
% Configure figure settings
fig2 = figure;
movegui(fig2,[1400+570 1500]);
hold on
xlim([-1000 w+1000])
ylim([-100 h+100])
% Frame box (red)
plot([0,w],[0,0], 'r')
plot([0,w],[h,h], 'r')
plot([0,0],[0,h], 'r')
plot([w,w],[0,h], 'r')

% Boundry lines (blue)
plot([min(xC),min(xC)],[0,h], 'b')
plot([max(xC),max(xC)],[0,h], 'b')
plot(xC,yC)