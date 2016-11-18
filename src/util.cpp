#include "util.h"


//Filter out the values higher than max range in meters. dimension can be "x", "y", "z"
Cloud clampPointCloud(const std::string dimension, Cloud cloud, float maxrange)
{
    Cloud cloud_filtered;
    // Create the filtering object
    pcl::PassThrough<PointType> pass;
    pass.setInputCloud (cloud.makeShared());
    pass.setFilterFieldName (dimension.data());
    pass.setFilterLimits (0.0, maxrange);
    //pass.setFilterLimitsNegative (true);
    pass.filter (cloud_filtered);

    return cloud_filtered;
}

// Function for detecting and storing objects from sweeps
RoomObservation readRGBImagesfromRoomSweep(const std::string &observationpath, tf::Vector3& robotPosition)
{

    RoomObservation obs;


    //The pair of images and clouds that are read from the room folder
    std::vector< std::pair< sensor_msgs::Image, Cloud > > rosimagesclouds;


    // Load the room xml
    SemanticRoom<PointType> aRoom = SemanticRoomXMLParser<PointType>::loadRoomFromXML(observationpath,true);

    // Parse the necessary clouds
    // auto sweep = SimpleXMLParser<PointType>::loadRoomFromXML(observationpath, std::vector<std::string>{"RoomCompleteCloud", "RoomIntermediateCloud"},false, true);
    // auto sweep = SimpleXMLParser<PointType>::loadRoomFromXML(observationpath, std::vector<std::string>{"RoomIntermediateCloud"},false, true);


    obs.room = aRoom;



    //   robotPosition = aRoom.getCentroid();


    semantic_map_load_utilties::IntermediateCloudCompleteData<PointType> dat = semantic_map_load_utilties::loadIntermediateCloudsCompleteDataFromSingleSweep<PointType>(observationpath);


    //ROS_INFO("%s",observations[i].data());


    // Get the intermediate clouds
    std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>> clouds = dat.vIntermediateRoomClouds;//sweep.vIntermediateRoomClouds;//aRoom.getIntermediateClouds();

    /*  std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>> clamped_clouds;
     for(int j = 0 ; j < clouds.size(); j++)
     {
         Cloud ptr = clampPointCloud("z",*clouds[j],5.0);
         clamped_clouds.push_back(ptr.makeShared());
     }*/

    //   std::vector<tf::StampedTransform> transformsregistered = aRoom.getIntermediateCloudTransformsRegistered();

    //   sweepCenter = transformsregistered[transformsregistered.size()/2].getOrigin();


    // Get the cloud transforms
    //  std::vector<tf::StampedTransform> transforms = sweep.getIntermediateCloudTransforms();

    std::vector<tf::StampedTransform> transforms = dat.vIntermediateRoomCloudTransforms; //sweep.vIntermediateRoomCloudTransforms;

    std::vector<tf::StampedTransform> transformsregistered = dat.vIntermediateRoomCloudTransformsRegistered;//sweep.vIntermediateRoomCloudTransformsRegistered;

    tf::StampedTransform center_transform = transforms[0];  //dat.vIntermediateRoomCloudTransformsRegistered[dat.vIntermediateRoomCloudTransformsRegistered.size()/2];

    // center_transform = transforms[0];

    robotPosition = center_transform.getOrigin();


    //std::cout<<"transforms sizes"<<transforms.size()<<" "<<transformsregistered.size()<<" "<<clouds.size()<<std::endl;

    int j  = 0;



    // For each cloud
    for(int i = 0; i < clouds.size(); i++)
    {
        boost::shared_ptr<pcl::PointCloud<PointType>> cloud = clouds[i];


        std::pair<sensor_msgs::Image,Cloud > datapair;

        // Create the RGB and Depth images
        std::pair<cv::Mat,cv::Mat> rgbanddepth =  SimpleXMLParser<PointType>::createRGBandDepthFromPC(cloud);

        cv_bridge::CvImage bridgeimage;

        bridgeimage.image = rgbanddepth.first;

        bridgeimage.encoding ="bgr8";

        // cvimages.push_back(bridgeimage.image);

        sensor_msgs::Image rosimage;

        bridgeimage.toImageMsg(rosimage);

        // This is rgb image
        datapair.first = rosimage;



        tf::Transform atransform = transforms[0];//*transformsregistered[i];

        pcl::PointCloud<PointType> transformed_cloud;


        // Transform the local point cloud to map frame
        pcl_ros::transformPointCloud(*cloud, transformed_cloud ,transformsregistered[i]);



        // Transform the local point cloud to map frame
        pcl_ros::transformPointCloud(transformed_cloud, transformed_cloud,transforms[0]);

        // pcl_ros::transformPointCloud(transformed_cloud, transformed_cloud ,correction);


        j++;

        // Set the cloud
        datapair.second = transformed_cloud;

        rosimagesclouds.push_back(datapair);


        // ROS_INFO("Data extracted...");
    }

    obs.rosimagesclouds = rosimagesclouds;

    // res.push_back(obs);


    // Return the images and corresponding clouds as a vector of pairs
    return obs;

}

std::vector< std::pair<deep_object_detection::Object,Cloud> > refineObjects(const std::vector<deep_object_detection::Object> &objects, const std::vector<Cloud> &clouds,
                                                                            int image_cols, const std::vector<std::string> labels, tf::Vector3 robotPosition)
{
    std::vector< std::pair<deep_object_detection::Object,Cloud> > result;

    std::vector<Cloud> remainingobjectclouds;

    std::vector<Eigen::Vector4f> remainingobjectcentroids;

    std::vector<deep_object_detection::Object> remainingobjects;


    // This part eliminates the objects without requested labels
    for(int i = 0; i < objects.size(); i++)
    {
        bool process = false;

        // If we have any specific labels, otherwise process all of them
        if(labels.size() > 0){

            for(int j = 0; j < labels.size(); j++)
            {
                if (objects[i].label == labels[j])
                {
                    process = true;
                    break;
                }
            }
        }
        else
            process = true;

        if(process)
        {

            Cloud objectpc = crop3DObjectFromPointCloud(objects[i],clouds[objects[i].imageID],image_cols);

            Eigen::Vector4f centroid;

            pcl::compute3DCentroid(objectpc,centroid);

            remainingobjectclouds.push_back(objectpc);

            remainingobjectcentroids.push_back(centroid);

            remainingobjects.push_back(objects[i]);

        }
    }

    // This part eliminates the duplicate objects
    std::vector<bool> addindex;

    // First all the objects with the desired label is deemed to be added
    for(int i = 0; i < remainingobjectclouds.size();i++)
        addindex.push_back(true);

    // Find the duplicates and eliminate
    for(int i = 0; i < remainingobjectclouds.size(); i++)
    {
        if(addindex[i])
        {

            for(int j = i+1 ; j < remainingobjectcentroids.size(); j++)
            {

                if(pcl::distances::l2(remainingobjectcentroids[i],remainingobjectcentroids[j]) <= 1.5)
                {
                    double isize = remainingobjects[i].width*remainingobjects[i].height;
                    double jsize = remainingobjects[j].width*remainingobjects[j].height;

                    std::cout<<pcl::distances::l2(remainingobjectcentroids[i],remainingobjectcentroids[j])<<std::endl;

                    if(remainingobjects[i].label == remainingobjects[j].label && isize < jsize)
                    {
                        //  addindex.push_back(i);
                        addindex[i] = false;
                        break;
                    }
                    else if(remainingobjects[i].label == remainingobjects[j].label)
                    {
                        addindex[j] = false;
                        //  break;
                    }
                    // addindex.push_back(j);

                }

            }
            // if(shouldadd)
            // addindex.push_back(i);
        }
    }

    // auto last = std::unique(addindex.begin(), addindex.end());
    // addindex.erase(last, addindex.end());

    for(int i =0; i < addindex.size(); i++)
    {
        Eigen::Vector4f posxyobj = remainingobjectcentroids[i];
        posxyobj[2] = 0;
        posxyobj[3] = 0;

        Eigen::Vector4f posxyrob;
        posxyrob[0] = robotPosition[0];
        posxyrob[1] = robotPosition[1];
        posxyrob[2] = 0;
        posxyrob[3] = 0;

        double dist = pcl::distances::l2(posxyobj,posxyrob);

        ROS_INFO("Robot Position: %.2f %.2f",posxyrob[0],posxyrob[1]);
        ROS_INFO("Object Position: %.2f %.2f",posxyobj[0],posxyobj[1]);
        ROS_INFO("Distance of object w.r.t robot: %.2f",dist);


        if(addindex[i] == true && dist <= 3.5)
        {

            std::pair<deep_object_detection::Object,Cloud> objectcloudpair;

            objectcloudpair.first = remainingobjects[i];
            objectcloudpair.second = remainingobjectclouds[i];

            result.push_back(objectcloudpair);






            // Eigen::Vector3f vec3obj = remainingobjectcentroids[i].head<3>();


            //   ROS_INFO("Distance of object w.r.t robot: %.2f",pcl::distances::l2(remainingobjectcentroids[i],vec4));

            //   Eigen::Vector2f vec3 = robotPosition.head<2>();



            //  vec3 = vec3/vec3.squaredNorm()

            //   Eigen::Vector2f vec3obj = remainingobjectcentroids[i].head<2>();

            //    vec3obj -=vec3;

            //    Eigen::Vector2f zeros;

            //    zeros[0] = 1.0;
            //    zeros[1]=  0.0;
            //zeros[2] = 0.;

            // vec3.normalize();

            //    vec3obj.normalize();

            //    zeros.normalize();

            //   std::cout<<"Dots"<<" "<<(double)zeros.dot(vec3obj)<<" "<<acos((double)vec3obj.dot(zeros))<<" "<<vec3obj[0]<<" "<<vec3obj[1]<<" "<<std::endl;//vec3obj[2]<<std::endl;

            //    std::cout<<"Dots2"<<" "<<remainingobjects[i].width<<" "<<180*atan2(vec3obj[1],vec3obj[0])/3.14<<std::endl;



            //  ROS_INFO("angles %.2f",((float)180*acos((double)vec3.dot(vec3obj)/(vec3.squaredNorm()*vec3obj.squaredNorm()))/(M_PI)));
        }

    }



    return result;
}

Cloud crop3DObjectFromPointCloud(const deep_object_detection::Object& object, Cloud objectcloud, int image_cols)
{
    Cloud objectpc;


    for(int x = object.x; x < object.x + object.width; x++)
    {
        for(int y = object.y; y < object.y + object.height; y++)
        {

            if(pcl_isfinite(objectcloud.points[y*image_cols + x].x))

                objectpc.push_back(objectcloud.points[y*image_cols + x]);

        }
    }

    return objectpc;

}

// The function that visualizes objects detected by deep-net
void visualizeDeepNetObjects( std::pair<deep_object_detection::Object,Cloud> apair, sensor_msgs::Image image)
{

    ROS_INFO("%s",apair.first.label.data());

    cv_bridge::CvImagePtr ptr = cv_bridge::toCvCopy(image,"bgr8");

    cv_bridge::CvImagePtr ptr2 = cv_bridge::toCvCopy(image,"bgr8");

    cv::imshow("image_org",ptr2->image);


    cv::rectangle(ptr->image,cv::Point(apair.first.x,apair.first.y),
                  cv::Point(apair.first.x+apair.first.width, apair.first.y + apair.first.height),cv::Scalar(255,255,0));
    cv::putText(ptr->image,apair.first.label.data(),cv::Point(apair.first.x,apair.first.y-5),0,2.0,cv::Scalar(255,255,0));

    cv::imshow("image_found",ptr->image);


    //  pcl_ros::transformPointCloud(refinedObjectsCloudsPair[i].second, refinedObjectsCloudsPair[i].second,world_transform);

    pcl::visualization::CloudViewer viewer("pclviewer");
    //pcl::visualization::PCLVisualizer *p = new pcl::visualization::PCLVisualizer ("Labelled data");

    // p->addCoordinateSystem();
    std::stringstream ss;
    ss<<"cloud_"<<1;
    // p->addPointCloud(apair.second.makeShared(),ss.str().data());

    viewer.showCloud(apair.second.makeShared(),ss.str().data());

    while(true) {
        int key =  cv::waitKey(10);
        if((char)key=='q') break;
    }
    cv::destroyAllWindows();


    //p->spin();

    // p->removeAllPointClouds();
    // p->close();

    // while( !p->wasStopped());

    //  delete p;



    return;

}
