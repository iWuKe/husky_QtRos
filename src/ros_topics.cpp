/**
 *
 * Based on
 *
 * @brief Interact and handle subscriptions to ROS topics
 *
 * @date December 2022
 *
 **/

#include <QObject>
#include <sstream>
#include <XmlRpc.h>
#include "./include/husky_gui2/ros_topics.h"

namespace HUSKY_APP{

/// @brief This function retrieves system state (i.e. publishers, subscribers, and services).
bool TopicsHandle::getSystemState(V_TopicPublishers& topics_pubs,
                                  V_TopicSubscribers& topics_subs,
                                  V_ServiceProviders& service_providers){
    XmlRpc::XmlRpcValue args, result, payload;
    args[0] = ros::this_node::getName();

    if (!ros::master::execute("getSystemState", args, result, payload, true)){
        std::cout << "Failed to get the system state!" << std::endl;
        return false;
    }
    std::cout << "Got the system state!" << std::endl;

    topics_pubs.clear(); topics_subs.clear(); service_providers.clear();

    for (int i = 0; i < payload[0].size(); ++i){
        std::string topic = payload[0][i][0];
        XmlRpc::XmlRpcValue val = payload[0][i][1];
        std::vector<string> publishers(val.size());
        for (int k = 0; k < val.size(); ++k){
            std::string pub = payload[0][i][1][k];
            publishers[k] = pub;
        }
        topics_pubs.push_back(TopicPublishers(topic, publishers));
        std::cout << "Topic : " << i << ": " << topic << " -> Has " << val.size() << " publishers."<< std::endl;
    }
    for (int i = 0; i < payload[1].size(); ++i){
        std::string topic = payload[1][i][0];
        XmlRpc::XmlRpcValue val = payload[1][i][1];
        std::vector<string> subscribers(val.size());
        for (int k = 0; k < val.size(); ++k){
            std::string sub = payload[1][i][1][k];
            subscribers[k] = sub;
        }
        topics_subs.push_back(TopicSubscribers(topic, subscribers));
        std::cout << "Topic : " << i << ": " << topic << " -> Has " << val.size() << " subscribers." << std::endl;
    }
    for (int i = 0; i < payload[2].size(); ++i){
        std::string service = payload[2][i][0];
        XmlRpc::XmlRpcValue val = payload[2][i][1];
        std::vector<string> providers(val.size());
        for (int k = 0; k < val.size(); ++k){
            std::string prov = payload[2][i][1][k];
            providers[k] = prov;
        }
        service_providers.push_back(ServiceProviders(service, providers));
        std::cout << "Service : " << i << ": " << service << " -> Has " << val.size() << " services." << std::endl;
    }
    return true;
} //end getSystemState()

/// @brief This function retrieves all the !published! topics
bool TopicsHandle::getPublishedRosTopics(ros::master::V_TopicInfo& topics_type){
    ros::master::V_TopicInfo master_topics;

    if (!ros::master::getTopics(master_topics)){
        std::cout << "Failed to get the ROS topics!" << std::endl;
        return false;
    }

    topics_type.clear();
    std::cout << "Got the ROS topics!" << std::endl;
    for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++) {
      const ros::master::TopicInfo& info = *it;
      topics_type.push_back(ros::master::TopicInfo(info.name, info.datatype));
      std::cout << "Topic_" << it - master_topics.begin() << ": " << info.name << " -> " << info.datatype << std::endl;
    }
    return true;
} //end getPublishedROsTopics()

/// @brief This function retrieves all the topics and their types (!whether published or not!)
bool TopicsHandle::getRosTopics(ros::master::V_TopicInfo& topics_type){
    XmlRpc::XmlRpcValue args, result, payload;
    args[0] = ros::this_node::getName();

    if (!ros::master::execute("getTopicTypes", args, result, payload, true)){
        std::cout << "Failed to get the topic types!" << std::endl;
        return false;
    }

    topics_type.clear();
    std::cout << "Got the topic types!" << std::endl;
    for (int i = 0; i < payload.size(); ++i){
        topics_type.push_back(ros::master::TopicInfo(std::string(payload[i][0]), std::string(payload[i][1])));
        std::cout << "Topic : " << i << ": " << payload[i][0] << " -> " << payload[i][1] << std::endl;
    }
    return true;
} //end getRosTopics()

/// @brief This function retrieves the topics (only) & gets !only! published ones
ros::master::V_TopicInfo TopicsHandle::getRosTopics(){
    //QVector<string> topicsList;
    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);

    for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++) {
      const ros::master::TopicInfo& info = *it;
      //topicsList.push_back(info.name);
      std::cout << "topic_" << it - master_topics.begin() << ": " << info.name << std::endl;
    }
    return master_topics;
}

bool TopicsHandle::publish2Topic(ros::NodeHandle n, ros::master::TopicInfo topic){
    try{
        if ( ! ros::master::check() ) {
            qDebug() << "[DEBUG]" << "Bad ROS state detected by 'publish2Topic'";
            return false;
        }//do not start without ros.

        // Add the new ROS publisher.
        ros::Publisher new_ros_publisher;
        //new_ros_publisher = n.advertise<topic.datatype>(topic.name, 1000);
        new_ros_publisher = n.advertise<geometry_msgs::Twist>(topic.name, 1000);
        ros::start();
        return true;
    } catch (char *excp){
        std::cout << "Exception caught: " << excp << std::endl;
        qDebug() << "[EXCP]" << excp;
        return false;
    }
}

bool TopicsHandle::subscribe2Topic(ros::NodeHandle& n, ros::master::TopicInfo topic){
    try{
        if ( ! ros::master::check() ) {
            qDebug() << "[DEBUG]" << "Bad ROS state detected by 'subscribe2Topic'";
            return false;
        }//do not start without ros.

        // Add new ROS subscriber.
        ros::Subscriber new_ros_subscriber;
        //new_ros_subscriber = n.subscribe(topic, 10, &QNode::poseCallback);
        //start();
        return true;
    } catch (char *excp){
        std::cout << "Exception caught: " << excp << std::endl;
        qDebug() << "[EXCP]" << excp;
        return false;
    }
}

void TopicsHandle::topicConnectedCallback(const ros::SingleSubscriberPublisher& pub)
{
    std::string s = "Connection to " + pub.getTopic() + " established.";
    ROS_INFO_STREAM(s);
    std::cout << s << std::endl;
    qInfo() << "[INFO]" << s.c_str();
}

void TopicsHandle::topicDisconnectedCallback(const ros::SingleSubscriberPublisher& pub)
{
    std::string s = "Warning: Connection to " + pub.getTopic() + " closed.";
    ROS_INFO_STREAM(s);
    std::cout << s << std::endl;
    qInfo() << "[INFO]" << s.c_str();
}

bool TopicsHandle::topicHasSubscriber(const ros::SingleSubscriberPublisher& pub, QString& node_name)
{

    if ((pub.getSubscriberName().size() > 0))
        return true;
    else
        return false;
}

bool TopicsHandle::isRobotOnline(QString& node_name)
{
    return false;
}

}//end namespace
