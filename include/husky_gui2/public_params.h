#ifndef PUBLIC_PARAMS_H
#define PUBLIC_PARAMS_H

#include <QObject>
#include <QStringList>
#include <QString>
#include <QHostAddress>
#include <QNetworkInterface>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QHash>
#include <QObject>
#include <iostream>
#include <algorithm>
#include <sstream>
#include <ros/ros.h>
#include <ros/master.h>
#include "ros_topics.h"

namespace HUSKY_APP {

using namespace std;

class PubPrms : public QObject
{
    Q_OBJECT

public:
    ///@brief The different topic datatypes and the corresponding callbacks
    //static QHash<QString, ??> topic_callback; ATTENTION: implement in the future

    /// @brief The different robots that can be interfaced.
    static QStringList _ROBOTS_;

    static QHash<QString, int> robot_index;
    static QHash<QString, QString> robot_musthavesubscriber;

    static ros::master::V_TopicInfo topics_type;
    static V_TopicPublishers topics_pubs;
    static V_TopicSubscribers topics_subs;
    static V_ServiceProviders service_providers;

    static QString _DFLT_ROS_IP_;
    static QString _DFLT_MASTER_URI_;

    static int _DFLT_CB_INDEX_;

    static QStringList _DFLT_VEL_TOPICS_;
    //static QString& _DFLT_VEL_TOPIC_;
    static QStringList _DFLT_POSE_TOPICS_;
    //static QString& _DFLT_POSE_TOPIC_;

    static void getMyIpAddress(); //get the IP of the machine

    ///* Temporary -to be removed
    static double a[12];
    static int LENGTH;
    static int CYCLE;
    static int gas_i;
    ///*end  of temproraty block - ATTENTION

private:


};//end class PubPrms

}//end namespace

#endif // PUBLIC_PARAMS_H
