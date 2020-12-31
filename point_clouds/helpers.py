import numpy as np
import math
from scipy.spatial import ConvexHull
import numpy as np
from sklearn.cluster import DBSCAN
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import json
#ROS
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

# A label of -1 corresponds to noise
def cluster_point_cloud(pts, eps=0.3, min_samples=30, visualize=False):
    db = DBSCAN(eps, min_samples, algorithm='ball_tree', n_jobs=2).fit(pts)
    labels = db.labels_
    n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
    n_noise_ = list(labels).count(-1)
    print('Estimated number of clusters: %d' % n_clusters_)
    print('Estimated number of noise points: %d' % n_noise_)

    if visualize:
        core_samples_mask = np.zeros_like(db.labels_, dtype=bool)
        core_samples_mask[db.core_sample_indices_] = True

        unique_labels = set(labels)
        colors = [plt.cm.Spectral(each)
                for each in np.linspace(0, 1, len(unique_labels))]
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        for k, col in zip(unique_labels, colors):
            if k == -1:
                # Black used for noise.
                col = [0, 0, 0, 1]

            class_member_mask = (labels == k)
            core_mask = class_member_mask & core_samples_mask
            others_mask = class_member_mask & ~core_samples_mask
            x = np.array(pts[:,0])
            y = np.array(pts[:,1])
            z = np.array(pts[:,2])
            ax.set_xlabel('x')
            ax.set_ylabel('y')
            ax.set_zlabel('z')
            
            ax.scatter(x[core_mask],y[core_mask],z[core_mask], marker="o",s=3, c=col)
            ax.scatter(x[others_mask],y[others_mask],z[others_mask], marker="o",s=1, c=col)
            
        plt.show()

    return n_clusters_, labels
    
def rgba2rgb(rgba):
	alpha = rgba[3]
	r = alpha * rgba[0] * 255
	g = alpha * rgba[1] * 255
	b = alpha * rgba[2] * 255
	return (r,g,b)


def get_xyzvalue_points(ros_pc2, remove_nans=True, dtype=np.float):
    '''Pulls out x, y, and z columns from the cloud recordarray, and returns
    a 3xN matrix.
    '''
    points = np.zeros((4,), dtype=dtype)
    for point in pc2.read_points(ros_pc2, skip_nans=True):
        pt_x = point[0]
        pt_y = point[1]
        pt_z = point[2]
        if len(point) == 4:
            pt_value = point[3]
        else:
            pt_value = -1
        points = np.vstack((points, np.array([pt_x, pt_y, pt_z, pt_value])))

    return points


def minimum_bounding_rectangle(points):
    """
    Find the smallest bounding rectangle for a set of points.
    Returns a set of points representing the corners of the bounding box.

    :param points: a 4x2 matrix of coordinates
    :rval: (4,2)
    
    """
    from scipy.ndimage.interpolation import rotate
    pi2 = np.pi/2.
   
    # get the convex hull for the points
    hull_points = points[ConvexHull(points).vertices]

    # calculate edge angles
    edges = np.zeros((len(hull_points)-1, 2))
    edges = hull_points[1:] - hull_points[:-1]

    angles = np.zeros((len(edges)))
    angles = np.arctan2(edges[:, 1], edges[:, 0])

    angles = np.abs(np.mod(angles, pi2))
    angles = np.unique(angles)

    # find rotation matrices
    rotations = np.vstack([
        np.cos(angles),
        np.cos(angles-pi2),
        np.cos(angles+pi2),
        np.cos(angles)]).T

    rotations = rotations.reshape((-1, 2, 2))

    # apply rotations to the hull
    rot_points = np.dot(rotations, hull_points.T)

    # find the bounding points
    min_x = np.nanmin(rot_points[:, 0], axis=1)
    max_x = np.nanmax(rot_points[:, 0], axis=1)
    min_y = np.nanmin(rot_points[:, 1], axis=1)
    max_y = np.nanmax(rot_points[:, 1], axis=1)

    # find the box with the best area
    areas = (max_x - min_x) * (max_y - min_y)
    best_idx = np.argmin(areas)

    # return the best box
    x1 = max_x[best_idx]
    x2 = min_x[best_idx]
    y1 = max_y[best_idx]
    y2 = min_y[best_idx]
    r = rotations[best_idx]
  
    rval = np.zeros((4, 2))
    rval[0] = np.dot([x1, y2], r)
    rval[1] = np.dot([x2, y2], r)
    rval[2] = np.dot([x2, y1], r)
    rval[3] = np.dot([x1, y1], r) 
    return rval

def points2centroid_yaw(p1, p2, p3, p4):
    """
    Takes in four points of a rectangle continuous s.t. p1 ->p2->p3->p4 traces a rectangle
    Returns centroid and yaw measured from positive x axis from (-pi/2 to pi/2); tilted to right is negative yaw

    :param points: 4 points in (x,y); each point can be tuple, list or numpy array
    :rval: width, length, centroid_x, centroid_y , yaw in radians
    
    """ 
    cx = (p1[0]+p2[0]+p3[0]+p4[0])/4    
    cy = (p1[1]+p2[1]+p3[1]+p4[1])/4
    
    # to find yaw, first find the longer side of the box
    side1 = np.array([[p2[0] - p1[0]], [p2[1] - p1[1]]])
    side2 = np.array([[p3[0] - p2[0]], [p3[1] - p2[1]]])
    if(np.linalg.norm(side1) > np.linalg.norm(side2)):
      longside = side1
      w = np.linalg.norm(side2)
      l = np.linalg.norm(side1)
    else:
      longside = side2
      w = np.linalg.norm(side1)
      l = np.linalg.norm(side2)

    yaw = math.atan(longside[1]/longside[0])
    return w, l, cx, cy, yaw

def centroid_yaw2points(w, l, cx, cy, yaw):
    """
    Takes in centroid and yaw measured from positive x axis from (-pi/2 to pi/2); tilted to right is negative yaw
    Returns four points of a rectangle continuous s.t. p1 ->p2->p3->p4 traces a rectangle

    :param points: width, length, centroid_x, centroid_y , yaw in radians
    :rval: 4 points in (x,y) rerpesented as numpy arrays
    
    """ 
    box_at_origin = np.matrix([ [w/2, -l/2],
                               [w/2,  l/2],
                               [-w/2, l/2],
                               [-w/2, -l/2] ])


    inv_rotate = np.matrix([[np.cos(yaw), -np.sin(yaw)],     
                        [np.sin(yaw), np.cos(yaw)]])

    box_rotated = box_at_origin * inv_rotate
    box_final = np.squeeze(np.asarray(box_rotated + [cx, cy])) 
    
    return box_final[0], box_final[1], box_final[2], box_final[3] 

