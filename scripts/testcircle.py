from math import *

def getCenterRadiusFrom3Points(points):
	x1 = points[0][0]
	y1 = points[0][1]
	x2 = points[1][0]
	y2 = points[1][1]
	x3 = points[2][0]
	y3 = points[2][1]
	xCenter = ((x1**2+y1**2)*(y2-y3) +(x2**2+y2**2)*(y3-y1) + (x3**2+y3**2)*(y1-y2)) / (2*(x1*(y2-y3)-y1*(x2-x3)+x2*y3-x3*y2))
	yCenter = ((x1**2+y1**2)*(x3-x2) +(x2**2+y2**2)*(x1-x3) + (x3**2+y3**2)*(x2-x1)) / (2*(x1*(y2-y3)-y1*(x2-x3)+x2*y3-x3*y2))
	radius = sqrt((xCenter-x1)**2+(yCenter-y1)**2)
	return (xCenter,yCenter,radius)

print(getCenterRadiusFrom3Points(((2,1),(0,5),(-1,2))))