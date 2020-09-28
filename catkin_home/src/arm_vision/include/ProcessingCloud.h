#include <IncludersCloud.h>

// hand limits
// float lower[3] = { 0, 0.23, .1 };
// float upper[3] = { 360, .68, .9 };

// float lower[3] = { 0, 0.23, .1 };
// float upper[3] = { 50, .68, .9 };

// GlobalVariables
int resolution = 100;
std::vector<float> NormalHand;
pcl::PointXYZ Centroid;

bool flag = true;
pcl::PointXYZ FingertipPoint;
pcl::PointCloud<pcl::PointXYZ>::Ptr CloudSaver(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr CloudFinal(new pcl::PointCloud<pcl::PointXYZ>);
// std::stringstream UserName;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputcloud_RGB(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZ>::Ptr inputcloud_XYZ(new pcl::PointCloud<pcl::PointXYZ>);

/*pcl::PointCloud<pcl::PointXYZ>::Ptr BiggestCluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr
cloud, double tolerance, bool& flag) {

        int i = 0;
        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);

        //Set parameters for the clustering
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(tolerance); // 5mm
        ec.setMinClusterSize(500);
        ec.setMaxClusterSize(25000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        std::vector<unsigned int> npoints;
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds;

        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it !=
cluster_indices.end(); ++it)
        {
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new
pcl::PointCloud<pcl::PointXYZ>); for (std::vector<int>::const_iterator pit = it->indices.begin();
pit != it->indices.end(); ++pit) cloud_cluster->points.push_back(cloud->points[*pit]); //*
                cloud_cluster->width = cloud_cluster->points.size();
                cloud_cluster->height = 1;
                cloud_cluster->is_dense = true;

                npoints.push_back(cloud_cluster->points.size());
                //std::cout << "PointCloud representing the Cluster: " <<
cloud_cluster->points.size() << " data points." << std::endl;

                //Save the clouds in a vector
                clouds.push_back(cloud_cluster);

        }

        int maxindex;
        int count = 0;
        if (npoints.size() == 0) {
                //std::cout << "We have not found any cluster for the cloud";
                flag = false;
                return cloud;
        }
        else {
                unsigned int maxpoints = npoints[0];
                maxindex = 0;
                for (std::vector<unsigned int>::iterator it = npoints.begin(); it != npoints.end();
++it) { if (*it > maxpoints) { maxpoints = *it; maxindex = i;
                        }
                        i++;
                }
        }
        return clouds[maxindex>0?maxindex:0];
}*/

/*void PasstroughFilterXYZ(const pcl::PointCloud<pcl::PointXYZ>::Ptr& in,
pcl::PointCloud<pcl::PointXYZ>::Ptr& out, double lowboxlimits[3], double upperboxlimits[3]) {
        //Use pass through filters on the H, S and V channels to find points with same colour as the
hand. pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pointsX(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pointsY(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pointsZ(new pcl::PointCloud<pcl::PointXYZ>);

        //apply passthrough filter h
        pcl::PassThrough<pcl::PointXYZ> pass3;
        pass3.setInputCloud(in);
        pass3.setFilterFieldName("x");
        pass3.setFilterLimits(lowboxlimits[0], upperboxlimits[0]);
        //pass.setfilterlimitsnegative (true);
        pass3.filter(*pcl_pointsX);

        //Apply passthrough filter S
        pcl::PassThrough<pcl::PointXYZ> pass4;
        pass4.setInputCloud(pcl_pointsX);
        pass4.setFilterFieldName("y");
        pass4.setFilterLimits(lowboxlimits[1], upperboxlimits[1]);
        //pass.setFilterLimitsNegative (true);
        pass4.filter(*pcl_pointsY);

        //Apply passthrough filter V
        pcl::PassThrough<pcl::PointXYZ> pass5;
        pass5.setInputCloud(pcl_pointsY);
        pass5.setFilterFieldName("z");
        pass5.setFilterLimits(lowboxlimits[2], upperboxlimits[2]);
        //pass.setFilterLimitsNegative (true);
        pass5.filter(*out);

}

/*std::vector<std::vector<float>> VectorPlanesFitted(const pcl::PointCloud<pcl::PointXYZ>::Ptr
RawCloud, int& maxindexp) { pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new
pcl::PointCloud<pcl::PointXYZ>);

        // Create the segmentation object for the planar model and set all the parameters
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        //pcl::PCDWriter writer;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(100);
        seg.setDistanceThreshold(0.02);

        //Vector of clouds to save plane clouds
        std::vector<unsigned int> npointsplane;
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudsplane;

        //Save model coeficcients in a vector
        std::vector<std::vector<float>> vectorcoefficients;

        int i = 0, nr_points = (int)RawCloud->points.size();
        while (RawCloud->points.size() > .1 * nr_points)
        {
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new
pcl::PointCloud<pcl::PointXYZ>());
                // Segment the largest planar component from the remaining cloud
                seg.setInputCloud(RawCloud);
                seg.segment(*inliers, *coefficients);

                vectorcoefficients.push_back(std::vector<float>());

                if (inliers->indices.size() == 0)
                {
                        for (int k = 0; k < 4; k++) {
                                vectorcoefficients[i].push_back(coefficients->values[k]);
                        }
                        std::cout << "Could not estimate a planar model for the given dataset." <<
std::endl; break;
                }

                // Extract the planar inliers from the input cloud
                pcl::ExtractIndices<pcl::PointXYZ> extract;
                extract.setInputCloud(RawCloud);
                extract.setIndices(inliers);
                extract.setNegative(false);

                // Get the points associated with the planar surface
                extract.filter(*cloud_plane);
                cloudsplane.push_back(cloud_plane);

                //Save the coeffiecients of the plane equation
                for (int k = 0; k < 4; k++) {
                        vectorcoefficients[i].push_back(coefficients->values[k]);
                }
                //std::cout << "PointCloud representing the planar component: " <<
cloud_plane->points.size() << " data points." << std::endl;
                npointsplane.push_back(cloud_plane->points.size());


                // Remove the planar inliers, extract the rest
                extract.setNegative(true);
                extract.filter(*cloud_f);
                *RawCloud = *cloud_f;
                i++;
        }
        i = 0;
        //find the maximum planar cloud
        maxindexp;
        int countp = 0;
        if (npointsplane.size() == 0) {
                std::cout << "We have not found any cloud";
        }
        else {
                unsigned int maxpointsp = npointsplane[0];
                maxindexp = 0;
                for (std::vector<unsigned int>::iterator it = npointsplane.begin(); it !=
npointsplane.end(); ++it) { if (*it > maxpointsp) { maxpointsp = *it; maxindexp = i;
                        }
                        i++;
                }
        }
        return vectorcoefficients;
}*/
float dotp(pcl::PointXYZRGB v1, pcl::PointXYZRGB v2) {
    float product = 0;
    product += v1.x * v2.x;
    product += v1.y * v2.y;
    product += v1.z * v2.z;
    return product;
}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr PlanarRANSAC(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double &zPLANE) {
    // perform ransac planar filtration to remove table top
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg1;
    // Optional
    seg1.setOptimizeCoefficients(true);
    // Mandatory
    seg1.setModelType(pcl::SACMODEL_PLANE);
    seg1.setMethodType(pcl::SAC_RANSAC);
    seg1.setDistanceThreshold(0.01);

    seg1.setInputCloud(cloud);
    seg1.segment(*inliers, *coefficients);

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr outcloud_XYZ(new pcl::PointCloud<pcl::PointXYZRGB>);

    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*outcloud_XYZ);

    pcl::PointIndices::Ptr inliers2(new pcl::PointIndices);
    pcl::PointXYZRGB dumb;
    dumb.x = cloud->points[inliers->indices[0]].x;
    dumb.y = cloud->points[inliers->indices[0]].y;
    dumb.z = cloud->points[inliers->indices[0]].z;

    // pcl::PointCloud<pcl::PointXYZRGB>::iterator it = outcloud_XYZ; outcloud_XYZ->end();it++
    for (int i = 0; i < outcloud_XYZ->points.size(); i++) {
        // i++;
        float tempor = dotp(dumb, outcloud_XYZ->points[i]);
        if (tempor <= 0) inliers2->indices.push_back(i);
    }

    pcl::ExtractIndices<pcl::PointXYZRGB> extract2;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr outcloud_XYZ2(new pcl::PointCloud<pcl::PointXYZRGB>);

    extract.setInputCloud(outcloud_XYZ);
    extract.setIndices(inliers2);
    extract.setNegative(true);
    extract.filter(*outcloud_XYZ2);

    /*
    // create a pcl object to hold the passthrough filtered results
    pcl::PointCloud<pcl::PointXYZRGB> * xyz_cloud_filtered_passthrough = new
    pcl::PointCloud<pcl::PointXYZRGB>; pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    xyzCloudPtrPassthroughFiltered(xyz_cloud_filtered_passthrough);
    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(outcloud_XYZ);
    pass.setFilterFieldName("z");

    pass.setFilterLimits((cloud->points[inliers->indices[0]].z -5),
    (cloud->points[inliers->indices[0]].z + 0.002));
    //pass.setFilterLimitsNegative (true);
    zPLANE = cloud->points[inliers->indices[0]].z;
    pass.filter(*xyzCloudPtrPassthroughFiltered);
    */
    return outcloud_XYZ;
}

arm_vision::SegmentedClustersArray Clustering(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
    arm_vision::SegmentedClustersArray CloudClusters;
    /*
    // Create the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(cloud);

    // create the extraction object for the clusters
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    // specify euclidean cluster parameters
    ec.setClusterTolerance(0.01); // 2cm
    ec.setMinClusterSize(500);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    // exctract the indices pertaining to each cluster and store in a vector of pcl::PointIndices
    ec.extract(cluster_indices);

    // declare an instance of the SegmentedClustersArray message
    arm_vision::SegmentedClustersArray CloudClusters;

    // declare the output variable instances
    sensor_msgs::PointCloud2 output;
    pcl::PCLPointCloud2 outputPCL;

    // here, cluster_indices is a vector of indices for each cluster. iterate through each indices
    object to work with them seporately for (std::vector<pcl::PointIndices>::const_iterator it =
    cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {

            // create a new clusterData message object
            //obj_recognition::ClusterData clusterData;

            // create a pcl object to hold the extracted cluster
            pcl::PointCloud<pcl::PointXYZRGB> *cluster = new pcl::PointCloud<pcl::PointXYZRGB>;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusterPtr(cluster);

            // now we are in a vector of indices pertaining to a single cluster.
            // Assign each point corresponding to this cluster in xyzCloudPtrPassthroughFiltered a
    specific color for identification purposes for (std::vector<int>::const_iterator pit =
    it->indices.begin(); pit != it->indices.end(); ++pit)
            {
                    clusterPtr->points.push_back(cloud->points[*pit]);
            }

            // convert to pcl::PCLPointCloud2
            pcl::toPCLPointCloud2(*clusterPtr, outputPCL);

            // Convert to ROS data type
            pcl_conversions::fromPCL(outputPCL, output);

            // add the cluster to the array message
            //clusterData.cluster = output;
            CloudClusters.clusters.push_back(output);
    }*/
    return CloudClusters;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr CenteredCluster(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double tolerance, bool &flag,
    float centroidlabel[2]) {
    int i = 0;
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(cloud);

    // Set parameters for the clustering
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(tolerance);  // 5mm
    ec.setMinClusterSize(500);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    std::vector<unsigned int> npoints;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;

    std::vector<float[2]> centroids;

    std::vector<float> euclideanvec;
    /*
            for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it !=
       cluster_indices.end(); ++it)
            {
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new
       pcl::PointCloud<pcl::PointXYZRGB>);

                    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit !=
       it->indices.end(); ++pit) cloud_cluster->points.push_back(cloud->points[*pit]);

                    cloud_cluster->width = cloud_cluster->points.size();
                    cloud_cluster->height = 1;
                    cloud_cluster->is_dense = true;
                    float center3D[3];
                    int k;
                    for (k = 0; k < cloud_cluster->points.size(); k++)
                    {
                            center3D[0] += cloud_cluster->points[k].x;
                            center3D[1] += cloud_cluster->points[k].y;
                            center3D[2] += cloud_cluster->points[k].y;
                    }
                    for (int i = 0; i < 3; i++)
                            A[i] = A[i] / k;

                    float xSqr = (x1 - x2);
                    float ySqr = (y1 - y2);

                    float dist = std::pow(xSqr, 2) + std::pow(xSqr, 2);

                    dist = std::sqrt(dist);

                    euclideanvec.push_back(dist);

                    npoints.push_back(cloud_cluster->points.size());
                    //std::cout << "PointCloud representing the Cluster: " <<
       cloud_cluster->points.size() << " data points." << std::endl;

                    //Save the clouds in a vector
                    clouds.push_back(cloud_cluster);
            }
    */

    int maxindex;
    int count = 0;
    if (npoints.size() == 0) {
        // std::cout << "We have not found any cluster for the cloud";
        flag = false;
        return cloud;
    } else {
        unsigned int mindist = euclideanvec[0];
        maxindex = 0;
        for (std::vector<float>::iterator it = euclideanvec.begin(); it != euclideanvec.end();
             ++it) {
            if (*it < mindist) {
                mindist = *it;
                maxindex = i;
            }
            i++;
        }
    }
    return clouds[maxindex > 0 ? maxindex : 0];
}
