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
import math
from scipy.spatial import ConvexHull


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
        n_clusters = max(n_clusters, 1)
        n_clusters = max(n_clusters, 1)
        rospy.loginfo("Running k-means clustering with %d clusters", n_clusters)
        kmeans = KMeans(n_clusters=n_clusters, random_state=0).fit(point_cloud_array)
        rospy.loginfo("Clustering complete, computing largest cluster")

        CALCULATE_AREA = True

        if CALCULATE_AREA:
            # calculate the area of each cluster
            # generate pixel density histogram with 1mm resolution for each cluster
            areas = {}
            largest_area = 0
            largest_cluster = 0
            
            histogram_fig = plt.figure()
            area_fig = plt.figure()
            # one subplot for each cluster
            HISTOGRAM_RESOLUTION_MM = 20

            for i in range(n_clusters):
                x, y = [], []
                for point in point_cloud_array[kmeans.labels_ == i]:
                    x.append(int(point[0]*1000))
                    y.append(int(point[1]*1000))
                # generate histogram with 1mm resolution = 0,001
                hist_range = [range(min(x), max(x)+HISTOGRAM_RESOLUTION_MM, HISTOGRAM_RESOLUTION_MM), range(min(y), max(y)+HISTOGRAM_RESOLUTION_MM, HISTOGRAM_RESOLUTION_MM)]
                hist, xedges, yedges = np.histogram2d(x, y, bins=hist_range, range=[[min(x), max(x)], [min(y), max(y)]])
                # plot
                try:
                    hist_s = histogram_fig.add_subplot(math.ceil(math.sqrt(n_clusters)),math.ceil(math.sqrt(n_clusters)),i+1)
                    #hist_s.imshow(hist.T, interpolation="nearest", origin='lower', extent=[xedges[0], xedges[-1], yedges[0], yedges[-1]], cmap=plt.cm.viridis)
                    
                    # turn off axis values  
                    hist_s.axis('off')
                    # save
                    if self.save_image:
                        print("Saving histogram")
                        histogram_fig.savefig('histogram.png')
                except:
                    print("Cluster " + str(i) + " not plotted")

                # calculate area
                TRESHOLD = 1

                over_treshold = hist > TRESHOLD
                # calculate area
                area_of_square = (HISTOGRAM_RESOLUTION_MM/1000)**2
                area = over_treshold.sum() * area_of_square
                print("Area of cluster " + str(i) + ": " + str(area) + " m^2")
                try:
                    tresh_fig = area_fig.add_subplot(math.ceil(math.sqrt(n_clusters)),math.ceil(math.sqrt(n_clusters)),i+1)
                    tresh_fig.imshow(over_treshold.T, interpolation="nearest", origin='lower', extent=[xedges[0], xedges[-1], yedges[0], yedges[-1]], cmap=plt.cm.viridis)
                    # turn off axis values
                    tresh_fig.axis('off')
                    # write area of cluster to plt image as title
                    tresh_fig.set_title(str(round(area, 3)) + " m^2")
                    if self.save_image:
                        print("Saving area")
                        area_fig.savefig('area.png')
                except:
                    pass

                # save area to dictionary
                areas[i] = area
                
                if area > largest_area:
                    largest_area = area
                    largest_cluster = i
                
            # distribute subplots in window and adjust size
            histogram_fig.tight_layout()
            area_fig.tight_layout()

            # plt.show()

            # from the cluster with the biggest area, make a convex hull and calculate the centroid
            # get the point_cloud_array of the biggest cluster
            biggest_cluster_point_cloud_array = point_cloud_array[kmeans.labels_ == largest_cluster] * 1000
            # calculate the convex hull
            hull = ConvexHull(biggest_cluster_point_cloud_array)

            # draw the convex hull
            hull_fig = plt.figure()
            hull_s = hull_fig.add_subplot(111)
            hull_s.plot(biggest_cluster_point_cloud_array[:,0], biggest_cluster_point_cloud_array[:,1], 'o')
            for simplex in hull.simplices:
                hull_s.plot(biggest_cluster_point_cloud_array[simplex, 0], biggest_cluster_point_cloud_array[simplex, 1], 'k-')

            # get centroid and centrum of convex hull
            class CHull(ConvexHull):

                def __init__(self, point_cloud_array):
                    ConvexHull.__init__(self, point_cloud_array)

                def centrum(self):

                    c = []
                    for i in range(self.point_cloud_array.shape[1]):
                        c.append(np.mean(self.point_cloud_array[self.vertices,i]))

                    return c
            
            centroid_cx = np.mean(biggest_cluster_point_cloud_array[hull.vertices,0])
            centroid_cy = np.mean(biggest_cluster_point_cloud_array[hull.vertices,1])
            centroid = [centroid_cx, centroid_cy]

            #centrum = CHull(biggest_cluster_point_cloud_array).centrum()

            # draw centroid as green circle
            hull_s.scatter(centroid[0], centroid[1], c='green', s=1000, alpha=0.8)
            # draw centrum as red circle
            #hull_s.scatter(centrum[0], centrum[1], c='red', s=500, alpha=0.8)
            hull_s.set_title("Convex hull of biggest cluster")
            hull_s.axis('off')
            hull_fig.tight_layout()

            if self.save_image:
                print("Saving convex hull")
                hull_fig.savefig('hull.png')

            # back to meters
            centroid = np.array(centroid) / 1000

        else:
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
            biggest_centroid = centroid
            
        rospy.loginfo("Largest cluster found, centroid: %s", centroid)

        if self.save_image:
            full_fig = plt.figure()
            ax = full_fig.add_subplot(111)
            ax.scatter(point_cloud_array[:,0], point_cloud_array[:,1], c=kmeans.labels_, cmap='rainbow')
            ax.scatter(centroid[0], centroid[1], c='green', s=1000, alpha=0.8)
            ax.set_xlabel('x')
            ax.set_ylabel('y')
            ax.set_title('Clusters')
            # set axis scale to be equal
            plt.axis('equal')


            # save as file
            rospy.loginfo("Saving clusters as image")
            plt.savefig('/home/afr2903u/ws/Robocup-Home/clusters.png')

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
