# Code is supposed to detect if there is an obstacle, and if so, stop the tractor
# All lines with a *** have arbitrary values as of 10/8/17
# Things do to before completion:
# Actually import data
# Find out how to stop the tractor
# Test various values to make sure it works
import rospy
def listener():
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber('scan',LaserScan,callback)
	rospy.spin()

def otherCode():
	totalDist = range0 # puts the distance away into a separate variable
	degrees = (step - 384) * (180 / 510) # Converts step to degrees
	horizontalDistance = abs(sin(degrees) * totalDist) # Computes the distance parallel to tractor
	verticalDistance = abs(cos(degrees) * totalDist) # Computes the distance from the object
	someDistanceAway = 300 # Essentially the ground ***
	lengthOfTheTractor = 150 # Horizontal length of the tractor ***
	obstaclePoints = 0 # Counts how many points break the threshold
	numberOfPointsNeededToTrigger = 6 # How many points must be seen to trigger a stop? ***
	for i in totalDist: #Sweep throught the distances
		if(totalDist[i] < someDistanceAway and horizontalDistance[i] < 150): # Is there an object that will hit the tractor?
			obstaclePoints += 1 # Add a point into the number of obstacle points
	if(obstaclePoints > numberOfPointsNeededToTrigger):
		stopTheTractor # Whatever code is needed to stop the tractor

def stopTheTractor():
	print ("Tractor is stopped!")

if __name__ == '__main__':
    listener()