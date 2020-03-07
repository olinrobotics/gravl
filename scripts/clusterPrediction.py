#!/usr/bin/env python3
from gravl.msg import Cluster
from std_msgs.msg import int64
from sklearn import svm, datasets
import pickle
import rospy

def clusterPrediction():
	def __init__(self,arg):
		rospy.init_node("clusterPrediction")
		self.s = rospy.Service('predictCluster',predictCluster,self.predictCluster)
		self.fileName = "$(find gravl)/scripts/" + arg
		rospy.spin()

	def predictCluster(self,req):
		# TODO: find a way to change the variables, either automatically or by argument.
		# TODO: Should I make another folder for the models?
		trained_model = pickle.load(open(self.fileName,"rb"))
		var1 = req.density
		var2 = req.vol
		predictedClass = int64()
		predictedClass.data = trained_model.predict([[var1,var2]])[0]
		return predictedClass


if __name__=="__main__":
    if len(sys.argv) < 2:
        print("usage: clusterPrediction.py [model_name]")
    else:
        clust = clusterPrediction(sys.argv[1])
