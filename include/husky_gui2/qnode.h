/**
 *
 * Based on https://github.com/xpharry/ros-qt-gui & https://github.com/allenh1/ros-qt-controller
 *
 * @brief Communications central!
 *
 * @date October 2022
 **/

#ifndef HUSKY_APP_QNODE_H_
#define HUSKY_APP_QNODE_H_

/*****************************************************************************
** Includes
*****************************************************************************/
#include <QtCore>
#include <QThread>
#include <QStringListModel>
#include <QDebug>
#include <QString>
#include <QMutex>
#include <stdlib.h>
#include <string>
#include <sstream>
#include <iostream>
#include "assert.h"
#include "logger.h"

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

/*****************************************************************************
** Namespaces
*****************************************************************************/
QT_BEGIN_NAMESPACE
namespace HUSKY_APP { class QNode; }
QT_END_NAMESPACE

namespace HUSKY_APP {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT

public:
    QNode(int argc, char** argv);
    virtual ~QNode();
    bool init();
    bool init(const std::string &master_url, const std::string &host_url,
            const std::string &vel_topic, const std::string &pose_topic);
    //bool initSubscriber(const std::string &node_name, const std::string &master_url,
    //          const std::string &host_url, const ros::master::TopicInfo &topic_info);
    bool initImageSubscriber(const std::string &node_name, const std::string &master_url,
                        const std::string &host_url, const std::string &topic_info);
    Q_SLOT void run();

    double getXPos();
    double getXSpeed();
    double getASpeed();
    double getYPos();
    double getAPos();

    //void chatterCallback(const std_msgs::String::ConstPtr& msg);
    void poseCallback(const turtlesim::Pose & msg);
    void poseOdomCallback(const nav_msgs::Odometry & msg);
    void imageCallback(const sensor_msgs::Image & msg);
    //void rvizCallback(const rosgraph_msgs::Log & message_holder);
    void SetSpeed(double speed, double angle);
    //void setPose(QList<double> to_set);

    ros::NodeHandle* nh_;
    ros::NodeHandle* nSb_;
    ros::Publisher vel_publisher;
    ros::Subscriber pose_listener;
    ros::Subscriber image_listener;
    ros::Subscriber chatter_listener;

signals://Q_SIGNALS:
    void newImage(cv::Mat);
    void newPose(double,double,double);
    void rosShutdown();
    void sendLog2Gui(const HUSKY_APP::Logger::LogLevel, const std::string);
    void sendLog2Gui2(const HUSKY_APP::Logger::LogLevel, const std::string, const std::string);

private:
    //QStringListModel logging_model;
    int init_argc;
    char** init_argv;

    double m_speed;
    double m_angle;

    double m_xPos;
    double m_yPos;
    double m_aPos;

    double m_maxRange;
    double m_minRange;
};

}  // namespace HUSKY_APP

Q_DECLARE_METATYPE(cv::Mat)

#endif /* HUSKY_APP_QNODE_H_ */
