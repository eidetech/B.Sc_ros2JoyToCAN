clc; clear all; close all;

topNotReached = true
resolution = 10
heightStep = 500
startHeight = 100
w = 2350
h = 10000
side = 0

i = 0


x = linspace(0, w, resolution)
y = linspace(0, h, resolution)
figure
hold on
xlim([-100 w+100])
ylim([-100 h+1000])
while topNotReached
    plot([0 w],[startHeight+heightStep*i startHeight+heightStep*i])
    if(heightStep*i >= h)
        topNotReached = false
        break
    end
    if(side == 0)
        plot([w w], [startHeight+heightStep*i startHeight+heightStep*(i+1)])
        side = 1
    elseif(side == 1)
        plot([0 0], [startHeight+heightStep*i startHeight+heightStep*(i+1)])
        side = 0
    end
i=i+1
end