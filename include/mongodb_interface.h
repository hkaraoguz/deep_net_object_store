#ifndef __MONGODB_INTERFACE
#define __MONGODB_INTERFACE


#include <mongodb_store/message_store.h>

// ROS includes
#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include <tf/transform_listener.h>

// PCL includes
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <QString>
#include <QVariant>
#include <qjson/serializer.h>
#include <QDateTime>
#include "util.h"
#include <semantic_map/room.h>
#include <semantic_map/semantic_map_summary_parser.h>
#include <metaroom_xml_parser/load_utilities.h>
#include <soma_msgs/SOMAObject.h>
#include <nav_msgs/OccupancyGrid.h>
#include <soma_manager/SOMAInsertObjs.h>

#include <deep_object_detection/Object.h>




class MongodbInterface {
public:


    MongodbInterface(ros::NodeHandle nh)  : m_messageStoreData(nh,"data","metric_maps"), m_messageStoreSummary(nh,"summary","metric_maps") ,
        m_messageStoreObject(nh,"data","labelled_objects"), m_messageStoreCloud(nh,"data","object_clouds")
    {
        m_NodeHandle = nh;
    }

    ~MongodbInterface()
    {

    }

    std::vector<double>  coordsToLngLat(double x, double y){
        std::vector<double> res;
        double earth_radius = 6371000.0; // in meters
        double lng = 90 - acos(float(x) / earth_radius)*180/M_PI;
        double lat = 90 - acos(float(y) / earth_radius)*180/M_PI ;
        res.push_back(lng);
        res.push_back(lat);
        return res;
    }

    template <class PointType>
    bool logSOMAObjectsToDBCallService(ros::ServiceClient cl, SemanticRoom<PointType> aRoom, RoomObservation obs /*std::vector< std::pair< deep_object_detection::Object, Cloud> > data*/)
    {

       // ros::ServiceClient cl = n.serviceClient<soma_manager::SOMAInsertObjs>("soma/insert_objects");

        if(!cl.exists())
        {
            qDebug()<<"SOMA insert_objects service does not exist!! Returning...";
            return false;

        }

       // QString room_name = aRoom.getRoomLogName().c_str() + QString("___") + QString::number(aRoom.getRoomRunNumber());

        std::vector<soma_msgs::SOMAObject> somaobjects;

        for (int j=0; j<obs.roomobjects.size();j++)
        {


            sensor_msgs::PointCloud2 msg_cloud;
            pcl::toROSMsg(obs.roomobjects[j].cloud, msg_cloud);


            soma_msgs::SOMAObject lobj;

            QJson::Serializer serializer;

            QVariantMap map;

            QVariant val(obs.roomobjects[j].object.confidence);

            map.insert("deep_net_confidence",val);

            QVariantMap boundingbox;

            QVariant bx(obs.roomobjects[j].object.x);
            QVariant by(obs.roomobjects[j].object.y);
            QVariant bwidth(obs.roomobjects[j].object.width);
            QVariant bheight(obs.roomobjects[j].object.height);

            boundingbox.insert("x",bx);
            boundingbox.insert("y",by);
            boundingbox.insert("width",bwidth);
            boundingbox.insert("height",bheight);

            map.insert("boundingbox",boundingbox);


            bool ok = false;
            QByteArray json = serializer.serialize(map,&ok);

            if(!ok)
            {
                ROS_WARN("Json for metadata cannot be created!!");
            }
            else
            {
                 QString jsonstr(json);

                 lobj.metadata = jsonstr.toStdString();
            }


            lobj.config ="deep_net_object";






        //    lobj.map_unique_id = map_unique_id;
        //    lobj.map_name = map_name;



            boost::posix_time::ptime epoch(boost::gregorian::date(1970, 1, 1));
            boost::posix_time::time_duration dur = aRoom.getRoomLogStartTime()-epoch;
            ros::Time rost;
            rost.fromSec(dur.total_seconds());

            lobj.header.stamp =rost;
            lobj.logtimestamp = dur.total_seconds();

            QDateTime dt = QDateTime::fromMSecsSinceEpoch(dur.total_milliseconds());

            Eigen::Vector4f centroid;

            pcl::compute3DCentroid(obs.roomobjects[j].cloud,centroid);

            lobj.cloud = msg_cloud;
            lobj.type = obs.roomobjects[j].object.label;

            lobj.pose.position.x = centroid[0];
            lobj.pose.position.y = centroid[1];
            lobj.pose.position.z = centroid[2];

           /* std::stringstream ss;

            ss<<"{\"deep_net_confidence\":"<<obs.roomobjects[j].object.confidence<<"\n}";

            lobj.metadata = ss.str();*/

            lobj.images.push_back(obs.roomobjects[j].rosimage);

            somaobjects.push_back(lobj);



        }


        soma_manager::SOMAInsertObjs srv;

        srv.request.objects = somaobjects;

        if (cl.call(srv))
        {
            if(srv.response.result)
                qDebug()<<"Objects successfully inserted";
            else
                qDebug()<<"Object insertion failed!";

        }
        else
        {
          ROS_ERROR("Failed to call soma/insert_objects service!!");
          return false;
        }



        return true;

    }






private:
    ros::NodeHandle                                                             m_NodeHandle;
    mongodb_store::MessageStoreProxy                                           m_messageStoreData;
    mongodb_store::MessageStoreProxy                                           m_messageStoreSummary;
    mongodb_store::MessageStoreProxy                                           m_messageStoreObject;
    mongodb_store::MessageStoreProxy                                           m_messageStoreCloud;
};




#endif // __MONGODB_INTERFACE
