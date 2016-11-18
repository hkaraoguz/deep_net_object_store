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
#include <semantic_data_store/LabelledObject.h>
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

    /*template <class PointType> */
    std::vector<boost::shared_ptr<semantic_data_store::LabelledObject>> getMongoDBObjects(const std::string& jsonQuery){


        mongo::BSONObjBuilder builder2;

        //std::string str("{\"objectPositionGlobal.x\":{\"$gt\":8.000}}");

        std::string str("{\"objectPositionGlobal.x\":{\"$gt\":-3.9113609790802002}}");

        QVariantMap vm;

        QVariantMap limits;

        QVariantMap sss;

        int xx = 4;
        int yy=  6;

        QList<QVariant> vec;



        vec.push_back(xx);

        vec.push_back(yy);

       // vec.push_back(0.1/3959);



        //ls.push_back(QString::number(yy));


        limits.insert("$near",vec);

        limits.insert("$maxDistance",1000);

       // limits.insert("$centerSphere",vec);

       // sss.insert("$geoWithin",limits);

        vm.insert("loc2",limits);

        QJson::Serializer serializer;
        bool ok;

        QByteArray json = serializer.serialize(vm, &ok);

        if (ok) {
          qDebug() << json;
        } else {
          qCritical() << "Something went wrong:" << serializer.errorMessage();
        }

      //  {loc: {$near: [50,50], $maxDistance: 10}}



        //mongo::BSONObj kkk;

     //  builder2.append("label","chair1");
        QString qstr(json);

        builder2.appendElements(mongo::fromjson(qstr.toStdString()));

        ROS_INFO_STREAM("Hello");


       // builder2.appendElements(q.getFilter());

       // builder << "data.sweepCenter.x" <<"{"<< "$gt" << -3.2 << "}";

        std::vector< boost::shared_ptr<semantic_data_store::LabelledObject> > results;

        if(m_messageStoreObject.query(results,builder2.obj()))
        {
            ROS_INFO_STREAM("Size of the query results"<<results.size());
        }


        return results;

    }


    template <class PointType>
    bool logRoomToDB(SemanticRoom<PointType> aRoom, std::string roomXMLPath)
    {

        typename pcl::PointCloud<PointType> Cloud;
        typename pcl::PointCloud<PointType>::Ptr CloudPtr;

        // first add xml file
        std::ifstream file(roomXMLPath);
        std::stringstream buffer;
        buffer << file.rdbuf();

        std_msgs::String room_name_ros;
        room_name_ros.data = buffer.str();

        mongo::BSONObjBuilder builder;
        builder.append("name", "metric_map_room_xml");
        mongo::BSONObj obj = builder.obj();

        std::string id(m_messageStoreSummary.insert(room_name_ros, "metric_maps" ,"summary" ,obj));
        ROS_INFO_STREAM("Room \""<<roomXMLPath<<"\" inserted with id "<<id);


        QString room_name = aRoom.getRoomLogName().c_str() + QString("___") + QString::number(aRoom.getRoomRunNumber());

        // save complete cloud
        {
            std::stringstream ss;
            ss<<room_name.toStdString();ss<<"___";
            ss<<"complete_cloud";

            mongo::BSONObjBuilder builder;
            builder.append("name", ss.str());
            mongo::BSONObj obj = builder.obj();

            sensor_msgs::PointCloud2 msg_cloud;
            pcl::toROSMsg(*aRoom.getCompleteRoomCloud(), msg_cloud);

            std::string id(m_messageStoreData.insert(msg_cloud, "metric_maps" ,"data" ,obj));
            ROS_INFO_STREAM("Complete cloud \""<<ss.str()<<"\" inserted with id "<<id);
        }

        // save intermediate clouds

        for (int j=0; j<aRoom.getIntermediateClouds().size();j++)
        {
            std::stringstream ss;
            ss<<room_name.toStdString();ss<<"___";
            ss<<"intermediate_cloud_";ss<<j;

            mongo::BSONObjBuilder builder;
            builder.append("name", ss.str());
            mongo::BSONObj obj = builder.obj();

            sensor_msgs::PointCloud2 msg_cloud;
            pcl::toROSMsg(*aRoom.getIntermediateClouds()[j], msg_cloud);

            std::string id(m_messageStoreData.insert(msg_cloud, "metric_maps" ,"data" ,obj));
            ROS_INFO_STREAM("Intermediate cloud \""<<ss.str()<<"\" inserted with id "<<id);

        }


        return true;
    }

    template <class PointType>
    std::vector<typename SemanticMapSummaryParser::EntityStruct> getMongodbRooms()
    {
        typedef typename SemanticMapSummaryParser::EntityStruct Entities;

        std::vector<Entities> toRet;

        std::vector< boost::shared_ptr<std_msgs::String> > results;
        if(m_messageStoreSummary.queryNamed<std_msgs::String>("metric_map_room_xml", results,false)) {
            ROS_INFO_STREAM("Found "<<results.size()<<" results.");

            BOOST_FOREACH(boost::shared_ptr<std_msgs::String> p,  results)
            {
                Entities new_room;
                 // Test room log name, run number and centroid
                QString room_file(p->data.c_str());

                int index1 = room_file.indexOf("<RoomLogName>");
                int index2 = room_file.indexOf("</RoomLogName>");
                int start = index1 + QString("<RoomLogName>").length();
                QString room_log_name_from_xml = room_file.mid(start,index2-start);

                index1 = room_file.indexOf("<RoomRunNumber>");
                index2 = room_file.indexOf("</RoomRunNumber>");
                start = index1 + QString("<RoomRunNumber>").length();
                QString room_run_number_from_xml = room_file.mid(start,index2-start);

                index1 = room_file.indexOf("<Centroid>");
                index2 = room_file.indexOf("</Centroid>");
                start = index1 + QString("<Centroid>").length();
                QString centroidS = room_file.mid(start,index2-start);

//                ROS_INFO_STREAM("Room saved in mongodb: "<<room_log_name_from_xml.toStdString()<<"  "<<room_run_number_from_xml.toStdString()<<"  "<<centroidS.toStdString());

                new_room.entityType = SEMANTIC_MAP_ROOM;
                QString room_xml_file = room_log_name_from_xml + "/room_" + room_run_number_from_xml;
                Eigen::Vector4f centroid;
                QStringList centroidSlist = centroidS.split(' ');
                centroid(0) = centroidSlist[0].toDouble();centroid(1) = centroidSlist[1].toDouble();
                centroid(2) = centroidSlist[2].toDouble();centroid(3) = centroidSlist[3].toDouble();

                new_room.roomXmlFile = room_xml_file.toStdString();
                new_room.roomLogName = room_log_name_from_xml.toStdString();
                new_room.centroid = centroid;
                new_room.isMetaRoom = false;

                toRet.push_back(new_room);
            }

        }

        ROS_INFO_STREAM(toRet.size()<<" rooms found in mongodb.");

        return toRet;
    }



private:
    ros::NodeHandle                                                             m_NodeHandle;
    mongodb_store::MessageStoreProxy                                           m_messageStoreData;
    mongodb_store::MessageStoreProxy                                           m_messageStoreSummary;
    mongodb_store::MessageStoreProxy                                           m_messageStoreObject;
    mongodb_store::MessageStoreProxy                                           m_messageStoreCloud;
};




#endif // __MONGODB_INTERFACE
