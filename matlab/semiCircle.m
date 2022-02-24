close all
k = 1
yPos = 0
while (yPos < 500)
th = linspace( -pi/2, pi/2, 100);
R = 250
xPos = R*cos(th(k));
yPos = R*sin(th(k))+250;
plot(xPos,yPos, 'o');
hold on
k = k+1
yPos
end

k = 1
while (yPos < 1000)
th = linspace( -pi/2, pi/2, 100);
R = 250
xPos = R*cos(th(k));
yPos = R*sin(th(k))+3*250;
plot(-xPos,yPos, 'o');
hold on
k = k+1
end