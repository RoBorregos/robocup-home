#!/usr/bin/env python3
# A ROS service that receives a pointcloud and runs k-means clustering on it.
# Returns the center x, y of the largest cluster.

import rospy
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sklearn.cluster import KMeans
from object_detector.srv import Clustering
import matplotlib.pyplot as plt
import cv2


ARGS= {
    "CLUSTERS_PER_OBJECT": 3,
    "SAVE_IMAGE": True
}

class Clustering_Service:
    def __init__(self):
        # Create service 
        self.service = rospy.Service('Clustering', Clustering, self.handle_clustering)
        self.clusters_per_object = ARGS["CLUSTERS_PER_OBJECT"]
        self.save_image = ARGS["SAVE_IMAGE"]
        rospy.loginfo("Clustering service ready")
        rospy.spin()
    
    def handle_clustering(self, req):
        # Convert pointcloud to numpy array, ignore z axis
        point_cloud = req.pointcloud
        #print(point_cloud)
        point_cloud_array = []
        for p in pc2.read_points(point_cloud, field_names = ("x", "y", "z"), skip_nans=True):
            # append point to array
            if not np.isnan(p[0]) and not np.isnan(p[1]):
                point_cloud_array.append([p[0], p[1]])
        point_cloud_array = np.array(point_cloud_array)
        #print(point_cloud_array)

        # Run k-means clustering
        n_clusters = req.n_clusters * self.clusters_per_object
        rospy.loginfo("Running k-means clustering with %d clusters", n_clusters)
        kmeans = KMeans(n_clusters=n_clusters, random_state=0).fit(point_cloud_array)
        rospy.loginfo("Clustering complete, computing largest cluster")

        # Find largest cluster
        # draw the centroids of the biggest cluster as a green circle
        # get the biggest cluster
        biggest_cluster = 0
        biggest_cluster_size = 0
        for i in range(n_clusters):
            if len(kmeans.labels_[kmeans.labels_ == i]) > biggest_cluster_size:
                biggest_cluster = i
                biggest_cluster_size = len(kmeans.labels_[kmeans.labels_ == i])
        # draw the centroid of the biggest cluster
        centroid = kmeans.cluster_centers_[biggest_cluster]
        rospy.loginfo("Largest cluster found, centroid: %s", centroid)

        if True:
            # Draw clusters with biggest cluster centroid drawn in green, save as matplotlib image
            plt.scatter(point_cloud_array[:,0], point_cloud_array[:,1], c=kmeans.labels_, cmap='rainbow')
            plt.scatter(centroid[0], centroid[1], c='green', s=1000, alpha=0.8)
            plt.xlabel('x')
            plt.ylabel('y')
            plt.title('Clusters')
            # set axis scale to be equal
            plt.axis('equal')


            # save as file
            rospy.loginfo("Saving clusters as image")
            plt.savefig('/home/rbrgs-pc/Desktop/Robocup-Home/clusters.png')

        rospy.loginfo("Returning centroid")

        # Return centroid
        return centroid[0], centroid[1], True



def main():
    rospy.init_node('Clustering_Service', anonymous=True)
    for key in ARGS:
        ARGS[key] = rospy.get_param('~' + key, ARGS[key])
    Clustering_Service()

if __name__ == '__main__':
    main()
