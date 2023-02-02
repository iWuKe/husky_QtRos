#include <QObject>
#include <iostream>
#include <algorithm>
#include <sstream>
#include <vector>
#include <numeric>
#include "./include/husky_gui2/public_params.h"

namespace HUSKY_APP{

///* the next lines are temporary for setting a dummy gas concentration value - to be removed
double PubPrms::a[12] = {7.0, 0.5, 1.0, 1.5, 6.5, 2.5, 3.0, 6.5, 4.0, 4.5, 5.0, 6.5};
int PubPrms::LENGTH = sizeof PubPrms::a / sizeof(*PubPrms::a);
//assert(LENGTH > 1); // doesn't work for fewer than 2 elements
int PubPrms::CYCLE = PubPrms::LENGTH - 1;
int PubPrms::gas_i = 0;
///* end of temporarily dummy gas concentration  ATTENTION

QStringList PubPrms::_ROBOTS_ =  {"None", "TurtleBot", "Simulator_Husky", "Husky"};

/// @brief this hash table maps the supported robots with an index
/// that is used in _DFLT_VEL_TOPICS_ and _DFLT_POSE_TOPICS_ (defined below)
QHash<QString, int> PubPrms::robot_index = {
    {"None", 0},
    {"TurtleBot", 1},
    {"Simulator_Husky", 2},
    {"Husky", 3}
};

/// @brief this hash table mapsl the supported robots with
/// the subscriber that the correcponding cmd_vel topic must have
///  if the robot is online
QHash<QString, QString> PubPrms::robot_musthavesubscriber = {
    {"None", ""},
    {"TurtleBot", "/turtlesim"},
    {"Simulator_Husky", "/gazebo"},
    {"Husky", ""} //ATTENTIOM: find this one
};

ros::master::V_TopicInfo PubPrms::topics_type;
V_TopicPublishers PubPrms::topics_pubs;
V_TopicSubscribers PubPrms::topics_subs;
V_ServiceProviders PubPrms::service_providers;

QString PubPrms::_DFLT_ROS_IP_ = "129.138.161.208";
//const QString& _DFLT_ROS_IP_ = "localhost";
QString PubPrms::_DFLT_MASTER_URI_ = "http://129.138.175.59:11311/";
//const QString& _DFLT_MASTER_URI_ = "http://localhost:11311/";

int PubPrms::_DFLT_CB_INDEX_ = 0;

QStringList PubPrms::_DFLT_VEL_TOPICS_ = {"","/turtle1/cmd_vel", "/husky_velocity_controller/cmd_vel",
                                        "/husky_velocity_controller/cmd_vel"};
//const QString& _DFLT_VEL_TOPIC_ = "/twist_marker_server/cmd_vel";
//const QString& _DFLT_VEL_TOPIC_ = "/husky_velocity_controller/cmd_vel";
//const QString& _DFLT_VEL_TOPIC_ = "/turtle1/cmd_vel";

QStringList PubPrms::_DFLT_POSE_TOPICS_ = {"","/turtle1/pose", "/husky_velocity_controller/odom",
                                        "/husky_velocity_controller/odom"};
//const QString& _DFLT_POSE_TOPIC_ = "/twist_marker_server/pose";
//const QString& _DFLT_POSE_TOPIC_ = "/husky_velocity_controller/pose";
//const QString& _DFLT_POSE_TOPIC_ = "/turtle1/pose";

void PubPrms::getMyIpAddress() {
    //PubPrms::_DFLT_ROS_IP_ = "129.138.161.208";
    try{
        QList<QHostAddress> list = QNetworkInterface::allAddresses();
        for(int nIter=0; nIter<list.count(); nIter++) {
            if(!list[nIter].isLoopback())
                if (list[nIter].protocol() == QAbstractSocket::IPv4Protocol ) {
                   //qDebug() << list[nIter].toString();
                    PubPrms::_DFLT_ROS_IP_ = list[nIter].toString();
                    std::cout << "My IP: " << _DFLT_ROS_IP_.toUtf8().constData()<< std::endl;
                }
        }

    }
    catch(char *excp){
        std::cout << "Exception caught: " << excp << std::endl;
        std::string s(excp);
        //myLog(Logger::Excp, s);
        qDebug() << "[EXCP]" << excp;
    }
}//find my IP address

}//end namespace

