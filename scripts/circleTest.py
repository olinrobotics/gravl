from math import *
x1 = 2.287
x2 = 2.278
y1 = -1.54
y2 = -1.827
r = 0.22
q = sqrt((x2-x1)**2 + (y2-y1)**2)
x3 = (x1+x2) / 2
y3 = (y1+y2) / 2
xCenter = x3 + sqrt(r**2-(q/2)**2)*(y1-y2)/q
yCenter = y3 + sqrt(r**2-(q/2)**2)*(x2-x1)/q  
print(xCenter,yCenter)
