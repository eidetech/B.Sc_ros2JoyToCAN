
c = 2350
px = 500

A = [sind(0) -1; 1 -1]
B = [0; sqrt(c^2-(2*c*px)+(px^2))]

X = A\B

d = px+1850