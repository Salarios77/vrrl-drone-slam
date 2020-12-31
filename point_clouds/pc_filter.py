import roslib
import rospy

from cv_bridge import CvBridge, CvBridgeError
import message_filters
import numpy as np

from helpers import *

def read_from_json(file):
    #sample code to get the object info from json file
    all_objects = json.loads(open(file, 'r').read())
    for obj in all_objects.keys():
        points = all_objects[obj]['points']
        size = all_objects[obj]['size']
        centroid = all_objects[obj]['centroid'] 
        #print(size, centroid)
    return

def callbackROS(velo_ros):
    
    points = get_xyzvalue_points(velo_ros)
    print("Input points shape:", points.shape)
 
    values = points[:,3] # is all -1 if the points dont have a value entry
    
    xyz = points[:,0:3]

    #Rotation to unity perspective
    R = np.array([[0, -1, 0], 
                  [0, 0, -1],
                  [1, 0, 0]])

    xyz = xyz @ R

    #clustering
    n_clusters, labels = cluster_point_cloud(xyz, eps=0.11, min_samples=50, visualize=True) #ignore rgb values for clustering

    #save to json file
    with open('objects.json', 'w') as outfile:
        #save a dictionary of dictionaries (each one corresponds to an object)
        all_objects = {}
        for id in range(n_clusters):
            
            obj = 'object{}'.format(id)
            all_objects[obj] = {}
            obj_points = xyz[np.where(labels==id)]
            xsize = np.max(obj_points[:, 0]) - np.min(obj_points[:, 0]) 
            ysize = np.max(obj_points[:, 1]) - np.min(obj_points[:, 1]) 
            zsize = np.max(obj_points[:, 2]) - np.min(obj_points[:, 2]) 
            centroid = np.average(obj_points, axis=0)
            all_objects[obj]['points'] = obj_points.tolist()
            all_objects[obj]['size'] = [xsize, ysize, zsize]
            all_objects[obj]['centroid'] = centroid.tolist()
        json.dump(all_objects, outfile)


if __name__ == '__main__':
	rospy.init_node('filter_pc')
	pc_sub = message_filters.Subscriber('cloud_pcd', PointCloud2)
	ts = message_filters.ApproximateTimeSynchronizer([pc_sub], queue_size=100, slop=0.07)		
	ts.registerCallback(callbackROS)
	print('Listening...')
	rospy.spin()
