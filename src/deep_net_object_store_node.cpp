#include "deep_net_object_store/mongodb_interface.h"
#include "metaroom_xml_parser/load_utilities.h"
#include <semantic_map/RoomObservation.h>
#include "deep_net_object_store/util.h"

#include <std_msgs/String.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/subscriber.h>
#include <soma_msgs/SOMAObject.h>
#include <deep_object_detection/DetectObjects.h>
#include <deep_object_detection/DetectedObjects.h>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <sys/types.h>
#include <sys/stat.h>


using namespace std;

tf::Vector3 sweepCenter;

Eigen::Vector4f robotPosition;

ros::ServiceClient object_detection_service;

MongodbInterface* m_MongodbInterface;

ros::ServiceClient soma_insert_service;

std::string metaroomxmlpath="";

void deepobjectdetectionDoneCallback(const deep_object_detection::DetectedObjects& msg)
{
   // metaroomxmlpath = msg.observation_path.data();
    struct stat info;
    if( stat( metaroomxmlpath.data(), &info ) != 0 )
    {

        cout<<"cannot access "<<metaroomxmlpath<<endl;
        metaroomxmlpath = "";
        return ;
    }

    if(metaroomxmlpath != "")
    {

        ROS_INFO("Processing Metaroom for deep_net objects");

        deep_object_detection::DetectObjects detect_objects;


        RoomObservation roomobservation = readRGBImagesfromRoomSweep(metaroomxmlpath,sweepCenter);

        ROS_INFO("Observation path %s",metaroomxmlpath.data());

        ROS_INFO("Images and clouds size: %lu",roomobservation.rosimagesclouds.size());

        if(roomobservation.rosimagesclouds.size() > 0)
        {

            /************Transform images into ros images***********************/
            std::vector<sensor_msgs::Image > rosimages;

            std::vector<Cloud> clouds;

            for(int i = 0; i < roomobservation.rosimagesclouds.size(); i++)
            {

                rosimages.push_back(roomobservation.rosimagesclouds[i].first);
                clouds.push_back(roomobservation.rosimagesclouds[i].second);

            }
            /*******************************************************************/


            ROS_INFO("Num objects %lu",msg.objects.size());
            if(msg.objects.size() > 0)
            {


                // We refine the detected objects based on distance to not to include same object multiple times
                std::vector< std::pair<deep_object_detection::Object,Cloud> > refinedObjectsCloudsPair = refineObjects(msg.objects,clouds,rosimages[0].width,std::vector<std::string>(),sweepCenter);

                for(int j = 0; j < refinedObjectsCloudsPair.size(); j++)
                {

                    std::pair<deep_object_detection::Object,Cloud> apair = refinedObjectsCloudsPair[j];

                    RoomObject aroomobject;

                    aroomobject.object = apair.first;
                    aroomobject.cloud = apair.second;

                    // aroomobject.image = roomobservation.rosimagesclouds[apair.first.imageID];

                    cv_bridge::CvImagePtr ptr = cv_bridge::toCvCopy(roomobservation.rosimagesclouds[apair.first.imageID].first,"bgr8");


                    cv::rectangle(ptr->image,cv::Point(refinedObjectsCloudsPair[j].first.x,refinedObjectsCloudsPair[j].first.y),
                                  cv::Point(refinedObjectsCloudsPair[j].first.x+refinedObjectsCloudsPair[j].first.width, refinedObjectsCloudsPair[j].first.y + refinedObjectsCloudsPair[j].first.height),cv::Scalar(255,255,0));


                    cv::putText(ptr->image,refinedObjectsCloudsPair[j].first.label.data(),cv::Point(refinedObjectsCloudsPair[j].first.x+refinedObjectsCloudsPair[j].first.width*0.25,refinedObjectsCloudsPair[j].first.y+refinedObjectsCloudsPair[j].first.height*0.5),0,2.0,cv::Scalar(255,255,0));


                    ptr->toImageMsg(aroomobject.rosimage);

                    roomobservation.roomobjects.push_back(aroomobject);



                }
                if(roomobservation.roomobjects.size() > 0)
                    m_MongodbInterface->logSOMAObjectsToDBCallService(soma_insert_service,roomobservation.room,roomobservation);

            }


        }
    }
    else
    {
        ROS_INFO("Cannot process metaroom since the path is not correct");

    }

    metaroomxmlpath = "";
}



void getMetaRoomPath(const semantic_map::RoomObservation& observation)
{

    struct stat info;
    metaroomxmlpath = "";

    if( stat( observation.xml_file_name.data(), &info ) != 0 ){
        cout<<"cannot access "<<observation.xml_file_name<<"..."<<endl;
        return ;
    }

    metaroomxmlpath = observation.xml_file_name;

    ROS_INFO("Received sweep path");





    /* if(!ros::ok())
    {
        ROS_INFO("Breaking loop...");
        break;
    }*/


}

int main(int argc, char** argv)
{


    /******* Initialize the ros node *******************/
    ros::init(argc, argv, "deep_net_object_store_node");

    ros::NodeHandle n;
    /**************************************************/

    m_MongodbInterface = new MongodbInterface(n);

   /* // Wait for deep-net ros service, if not available return
    if(!ros::service::waitForService("/deep_object_detection/detect_objects",30))
    {
        ROS_ERROR("Deep Object Detection service not available. Quitting...");
        return -3;

    }*/

    // Wait for deep-net ros service, if not available return
    if(!ros::service::waitForService("/soma/insert_objects",30))
    {
        ROS_ERROR("SOMA insert service not available. Quitting...");
        return -4;

    }




    ros::Subscriber sub = n.subscribe("/local_metric_map/room_observations", 1, getMetaRoomPath);


    //object_detection_service = n.serviceClient<deep_object_detection::DetectObjects>("/deep_object_detection/detect_objects");

    soma_insert_service = n.serviceClient<soma_manager::SOMAInsertObjs>("soma/insert_objects");

    ros::Subscriber sub2 = n.subscribe("/deep_object_detection/detected_objects", 5, deepobjectdetectionDoneCallback);




    ros::Rate loop(1);

    while(ros::ok())
    {

        ros::spinOnce();
        loop.sleep();

    }


    delete m_MongodbInterface;


    return 0;
}

