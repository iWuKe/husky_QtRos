/**
 *
 * Based on https://github.com/xpharry/ros-qt-gui  &  https://github.com/allenh1/ros-qt-controller
 *
 * @brief Communications central!
 *
 * @date October 2022
 **/

/*****************************************************************************
** Includes
*****************************************************************************/
#include <ros/ros.h>
#include <ros/network.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui.hpp"
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <rosgraph_msgs/Log.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>

#include <stdlib.h>
#include <string>
#include <sstream>
#include <iostream>
#include "assert.h"

#include "../include/husky_gui2/logger.h"
#include "../include/husky_gui2/qnode.h"
#include "./include/husky_gui2/public_params.h"
#include "./include/husky_gui2/ros_topics.h"

/*****************************************************************************
** Global variables
*****************************************************************************/

double PI = 3.1415926;

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace HUSKY_APP {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv)
    : init_argc(argc)
    , init_argv(argv)
    {/** Constructor for the main robot thread **/}

QNode::~QNode() {
    if(ros::isStarted()) {
      vel_publisher.shutdown();
      pose_listener.shutdown();
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
      delete nh_;
    }//end if
    wait();
}//end destructor of main robot thread

bool QNode::init() {
    try{
        ros::init(init_argc,init_argv,"husky_gui");
        if ( ! ros::master::check() ) {
            qDebug() << "[DEBUG]" << "Bad ROS state";
            emit sendLog2Gui(Logger::Excp,"Bad ROS state");
            return false;
        } //do not start without ros.
        ros::start(); // explicitly needed since our nodehandle is going out of scope.
        ros::NodeHandle nh_;

        qInfo() << "[INFO]" << "Connected husky_gui to ROS network";
        emit sendLog2Gui(Logger::Info, std::string("Connected husky_gui to ROS network"));

        ///* Add your ros communications here:
        /// * 1. Register publisher to velocity for movement control
        if (PubPrms::_DFLT_CB_INDEX_ != 0)
        {
            //vel_publisher = nh_.advertise<geometry_msgs::Twist>(vel_topic, 1000);
            //std::string vel_topic = PubPrms::_DFLT_VEL_TOPICS_[PubPrms::_DFLT_CB_INDEX_].toStdString();
            vel_publisher = nh_.advertise<geometry_msgs::Twist>(PubPrms::_DFLT_VEL_TOPICS_[PubPrms::_DFLT_CB_INDEX_].toStdString(),
                                                1000,
                                                (ros::SubscriberStatusCallback)TopicsHandle::topicConnectedCallback,
                                                (ros::SubscriberStatusCallback)TopicsHandle::topicConnectedCallback,
                                                ros::VoidConstPtr(), false);
        }

        ///* 2. register subscriber to pose for position monitoring
        ///* Pair the proper callback function depending on the pose_topic type
        if (PubPrms::_DFLT_CB_INDEX_ != 0)
        {
            //std::string pose_topic = PubPrms::_DFLT_POSE_TOPICS_[PubPrms::_DFLT_CB_INDEX_].toStdString();
            if (PubPrms::PubPrms::_DFLT_CB_INDEX_ == 2)
                pose_listener = nh_.subscribe(PubPrms::_DFLT_POSE_TOPICS_[PubPrms::_DFLT_CB_INDEX_].toStdString(), 10, &QNode::poseOdomCallback, this);
            else
                pose_listener = nh_.subscribe(PubPrms::_DFLT_POSE_TOPICS_[PubPrms::_DFLT_CB_INDEX_].toStdString(), 10, &QNode::poseCallback, this);
        }
        start();
        return true;
    } catch (char *excp){
        std::cout << "Exception caught: " << excp << std::endl;
        qDebug() << "[EXCP]" << excp;
        std::string s(excp);
        emit sendLog2Gui(Logger::Excp,std::string("Exception caught: ")+s);
        return false;
    }

}//default set up the main robot thread - used when use environment variables is checked

bool QNode::init(const std::string &master_url, const std::string &host_url,
                 const std::string &vel_topic, const std::string &pose_topic) {
    try{
        std::map<std::string,std::string> remappings;
        remappings["__master"] = master_url;
        remappings["__hostname"] = host_url;
        ros::init(remappings,"husky_gui");
        if ( ! ros::master::check() ) {
            qDebug() << "[DEBUG]" << "Bad ROS state";
            emit sendLog2Gui(Logger::Excp,"Bad ROS state");
            return false;
        }//do not start without ros.
        ros::start(); // explicitly needed since our nodehandle is going out of scope.
        ros::NodeHandle nh_;

        qInfo() << "[INFO]" << "Connected husky_gui to ROS network";
        emit sendLog2Gui(Logger::Info, std::string("Connected husky_gui to ROS network"));

        ///* Add your ros communications here:
        /// * 1. Register publisher to velocity for movement control
        if (PubPrms::_DFLT_CB_INDEX_ != 0)
        {
            //vel_publisher = nh_.advertise<geometry_msgs::Twist>(vel_topic, 1000);
            //std::string vel_topic = PubPrms::_DFLT_VEL_TOPICS_[PubPrms::_DFLT_CB_INDEX_].toStdString();
            vel_publisher = nh_.advertise<geometry_msgs::Twist>(vel_topic, 1000,
                                        (ros::SubscriberStatusCallback)TopicsHandle::topicConnectedCallback,
                                        (ros::SubscriberStatusCallback)TopicsHandle::topicConnectedCallback,
                                        ros::VoidConstPtr(), false);
        }

        ///* 2. register subscriber to pose for position monitoring
        ///* Pair the proper callback function depending on the pose_topic type
        if (PubPrms::_DFLT_CB_INDEX_ != 0)
        {
            //std::string pose_topic = PubPrms::_DFLT_POSE_TOPICS_[PubPrms::_DFLT_CB_INDEX_].toStdString();
            if (PubPrms::PubPrms::_DFLT_CB_INDEX_ == 2)
                pose_listener = nh_.subscribe(pose_topic, 10, &QNode::poseOdomCallback, this);
            else
                pose_listener = nh_.subscribe(pose_topic, 10, &QNode::poseCallback, this);
        }
        start();
        return true;
    } catch (char *excp){
        std::cout << "Exception caught: " << excp << std::endl;
        qDebug() << "[EXCP]" << excp;
        std::string s(excp);
        emit sendLog2Gui(Logger::Excp,std::string("Exception caught: ")+s);
        return false;
    }
}//customized setup for the main robot thread - used when use environment variables is UNCHECKED

bool QNode::initImageSubscriber(const std::string &node_name, const std::string &master_url,
                 const std::string &host_url, const std::string &topic_info) {
    try{
        std::map<std::string,std::string> remappings;
        remappings["__master"] = master_url;
        remappings["__hostname"] = host_url;
        ros::init(remappings,node_name);
        if ( ! ros::master::check() ) {
            qDebug() << "[DEBUG]" << "Bad ROS state";
            emit sendLog2Gui(Logger::Excp,"Bad ROS state");
            return false;
        }//do not start without ros.
        ros::start(); // explicitly needed since our nodehandle is going out of scope.
        ros::NodeHandle nSb_;

        std::stringstream ss;
        ss << "Connected node " << node_name.c_str() << " to ROS network";
        qInfo() << "[INFO]" << QString::fromStdString(ss.str());
        emit sendLog2Gui(Logger::Info, std::string(ss.str()));

        ///* Add your ros subscription here:
        ///* Pair the proper callback function depending on the listen_topic type
        /*QString t_n = topic_info.name.c_str();
        QString t_t = topic_info.datatype.c_str();
        if (t_t.contains("sensor_msgs/Image", Qt::CaseSensitive))
            image_listener = nSb_.subscribe(topic_info.name, 10, &QNode::imageCallback, this);
        else
            chatter_listener = nSb_.subscribe(topic_info.name, 10, &QNode::chatterCallback, this);*/
        image_listener = nSb_.subscribe(topic_info, 10, &QNode::imageCallback, this);
        start();
        return true;
    } catch (char *excp){
        std::cout << "Exception caught: " << excp << std::endl;
        qDebug() << "[EXCP]" << excp;
        std::string s(excp);
        emit sendLog2Gui(Logger::Excp,std::string("Exception caught: ")+s);
        return false;
    }
}//customized setup for the main robot thread - used when use environment variables is UNCHECKED

void QNode::run() {
    try{
        ros::Rate loop_rate(10);
        QMutex *pMutex;
        int i = 0;
        //ROS_INFO_STREAM("New publisher to topic " << topicName.toStdString() << " created."); //this sends messages to rosout, i.e., after the node is terminated
        //ROS_INFO("New topic created. Sending zero velocity command."); //ATTENTION: does this send live messages??
        //qInfo() << "Number of subcribers:" << vel_publisher.getNumSubscribers() << "Topic:" << vel_publisher.getTopic().c_str();
        while (ros::ok())
        {
            pMutex = new QMutex();
            // Wait until publisher has established connection, otherwise first sent messages get lost
            while((vel_publisher.getNumSubscribers() < 1) && (i < 20)){
                i++;
                if (i >= 50) {
                    qWarning() << "[WARN]" << "Ros publisher 'vel_publisher' failed to subscribe" << "n_subs:" << vel_publisher.getNumSubscribers();
                    //qWarning() << "[WARN]" << "Ros publisher 'vel_publisher' failed to subscribe";
                    emit sendLog2Gui(Logger::Error, std::string("Ros publisher 'vel_publisher' failed to subscribe"));
                    break;
                    emit rosShutdown();
                }
            }

            // Check if robot to be interfaced exists?? --ATTENTION--

            geometry_msgs::Twist cmd_msg;
            pMutex->lock();
            cmd_msg.linear.x = m_speed;
            cmd_msg.angular.z = m_angle;
            pMutex->unlock();

            if (PubPrms::_DFLT_CB_INDEX_ != 0)
            {
                vel_publisher.publish(cmd_msg);
                ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", cmd_msg.linear.x,cmd_msg.angular.z);
            }

            std::stringstream ss;
            ss << "New velocity -> Linear: " << cmd_msg.linear.x << ", Angular: " << cmd_msg.angular.z;
            //qInfo() << "[INFO]" << QString::fromStdString(ss.str());
            std::string s(ss.str());
            //emit sendLog2Gui(Logger::Info,std::string("I sent: ")+s);
            ////emit sendLog2Gui2(Logger::Info,std::string("I sent: ")+s, Q_FUNC_INFO);

            ros::spinOnce();
            loop_rate.sleep();
            delete pMutex;
        }//do ros things
        emit sendLog2Gui(Logger::Info,std::string("ROS shutdown.")); //ATTENTN:this does not work
        std::cout << "ROS shutdown." << std::endl;
        qInfo() << "[INFO]" << "ROS shutdown.";
        emit rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
    } catch (char *excp){
        std::cout << "Exception caught: " << excp << std::endl;
        qDebug() << "[EXCP]" << excp;
        std::string s(excp);
        emit sendLog2Gui(Logger::Excp,std::string("Exception caught: ")+s);
    }
}

/*void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
    try{
        std::stringstream ss;
        ss << "I heard: " << msg->data.c_str();
        //std::string s(ss.str());
        //qInfo() << "[INFO]" << QString::fromStdString(s);
        //emit sendLog2Gui(Logger::Info,s);
        ROS_INFO("I heard: [%s]", ss);
    } catch (char *excp){
        std::cout << "Exception caught: " << excp << std::endl;
        qDebug() << "[EXCP]" << excp;
        std::string s(excp);
        emit sendLog2Gui(Logger::Excp,std::string("Exception caught: ")+s);
    }
}//simple chatter callback method*/

void QNode::poseCallback(const turtlesim::Pose &msg)
{
    try{
        QMutex * pMutex = new QMutex();
        pMutex->lock();
        m_xPos = msg.x;
        m_yPos = msg.y;
        m_aPos = msg.theta;
        /*m_xPos = msg.pose.pose.position.x;
        m_yPos = msg.pose.pose.position.y;
        m_aPos = msg.pose.pose.orientation.w;*/
        pMutex->unlock();
        delete pMutex;
        emit newPose(m_xPos, m_yPos, m_aPos);
    } catch (char *excp){
        std::cout << "Exception caught: " << excp << std::endl;
        qDebug() << "[EXCP]" << excp;
        std::string s(excp);
        emit sendLog2Gui(Logger::Excp,std::string("Exception caught: ")+s);
    }
}//callback method to update the robot's position.

void QNode::poseOdomCallback(const nav_msgs::Odometry &msg)
{
    try{
        QMutex * pMutex = new QMutex();
        pMutex->lock();
        m_xPos = msg.pose.pose.position.x;
        m_yPos = msg.pose.pose.position.y;
        m_aPos = msg.pose.pose.orientation.z;
        pMutex->unlock();
        delete pMutex;
        emit newPose(m_xPos, m_yPos, m_aPos);
    } catch (char *excp){
        std::cout << "Exception caught: " << excp << std::endl;
        qDebug() << "[EXCP]" << excp;
        std::string s(excp);
        emit sendLog2Gui(Logger::Excp,std::string("Exception caught: ")+s);
    }
}//callback method to update the robot's position.

void QNode::imageCallback(const sensor_msgs::Image& msg)
{
    try {
        cv::Mat cv2_img = cv_bridge::toCvCopy(msg, "bgr8")->image;
        //cv::imshow("view", cv_bridge::toCvCopy(msg, "bgr8")->image);
        //cv::Mat dImg =  cv2_img->image;
        //cv::imwrite("camera_image.jpeg", dImg);
        /*double min = 0;
        double max = 1000;
        cv::Mat img_scaled_8u;
        cv::Mat(cv_ptr->image-min).convertTo(img_scaled_8u, CV_8UC1, 255. / (max - min));
        cv::cvtColor(img_scaled_8u, dImg, CV_GRAY2RGB);*/
        emit newImage(cv2_img);
    }
    catch (cv_bridge::Exception& e){
        std::cout << "Cv_bridge Exception " << msg.encoding.c_str() <<
                     " to 'bgr8':" << e.what() << std::endl;
        qDebug() << "[EXCP]" << e.what();
        std::string s(e.what());
        emit sendLog2Gui(Logger::Excp,std::string("Exception caught: ")+s);
    }
    catch (char *excp){
        std::cout << "Exception caught: " << excp << std::endl;
        qDebug() << "[EXCP]" << excp;
        std::string s(excp);
        emit sendLog2Gui(Logger::Excp,std::string("Exception caught: ")+s);
    }
} //callback method to get the camera feed and show in the GUI

void QNode::SetSpeed(double speed, double angle)
{
    try{
        //QMutex * pMutex = new QMutex();
        //pMutex->lock();
        m_speed = speed;
        m_angle = angle;
        //pMutex->unlock();
        //delete pMutex;
    } catch (char *excp){
        std::cout << "Exception caught: " << excp << std::endl;
        qDebug() << "[EXCP]" << excp;
        std::string s(excp);
        emit sendLog2Gui(Logger::Excp,std::string("Exception caught: ")+s);
    }
}//set the speed of the robot.

double QNode::getXSpeed(){ return m_speed; }
double QNode::getASpeed(){ return m_angle; }

double QNode::getXPos(){ return m_xPos; }
double QNode::getYPos(){ return m_yPos; }
double QNode::getAPos(){ return m_aPos; }

}  // namespace HUSKY_APP
