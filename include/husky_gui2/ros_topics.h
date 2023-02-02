/**
 *
 * Based on
 *
 * @brief Interact and handle subscriptions to ROS topics
 *
 * @date December 2022
 *
 **/

#ifndef ROS_TOPICS_H
#define ROS_TOPICS_H

#include <QObject>
#include <sstream>
#include <ros/ros.h>
#include <ros/master.h>
#include "qnode.h"
#include "logger.h"

namespace HUSKY_APP {

using namespace std;
//*** STRUCTURES ***//

/**
 * \brief Contains information retrieved from the master about a topics publishers
 */
struct  TopicPublishers
{
  TopicPublishers() {}
  TopicPublishers(std::string& _name, vector<std::string>& _publishers)
  : name(_name)
  , publishers(_publishers)
  {}
  std::string name;        ///< Name of the topic
  vector<std::string> publishers;    ///< Array/vector of publishers of the topic
};
typedef std::vector<TopicPublishers> V_TopicPublishers;

struct TopicSubscribers
{
  TopicSubscribers() {}
  TopicSubscribers(std::string& _name, vector<std::string>& _subscribers)
  : name(_name)
  , subscribers(_subscribers)
  {}
  std::string name;        ///< Name of the topic
  vector<std::string> subscribers;    ///< Array/vector of subscribers of the topic
};
typedef std::vector<TopicSubscribers> V_TopicSubscribers;

struct ServiceProviders
{
  ServiceProviders() {}
  ServiceProviders(std::string& _name, vector<std::string>& _providers)
  : name(_name)
  , providers(_providers)
  {}
  std::string name;        ///< Name of the service
  vector<std::string> providers;    ///< Array/vector of providers of the service
};
typedef std::vector<ServiceProviders> V_ServiceProviders;


using namespace std;

class TopicsHandle : public QObject
{
    Q_OBJECT

public:

    static QVector<string> topicsVector;

    static bool getSystemState(V_TopicPublishers& topics_pubs,
                               V_TopicSubscribers& topics_subs,
                               V_ServiceProviders& service_providers);
    static ros::master::V_TopicInfo getRosTopics();
    static bool getPublishedRosTopics(ros::master::V_TopicInfo& topics_type);
    static bool getRosTopics(ros::master::V_TopicInfo& topics_type);

    static bool publish2Topic(ros::NodeHandle n, ros::master::TopicInfo topic_name);
    static bool subscribe2Topic(ros::NodeHandle& n, ros::master::TopicInfo topic_name);

    static void topicConnectedCallback(const ros::SingleSubscriberPublisher& pub);
    static void topicDisconnectedCallback(const ros::SingleSubscriberPublisher& pub);

    /// @brief The function checks if a topic has any subscriber except the one specified in the second input.
    static bool topicHasSubscriber(const ros::SingleSubscriberPublisher& pub, QString& node_name);

    /// @brief The function checks if a specified robot is online the ROS network (.
    static bool isRobotOnline(QString& robot_name);

private:

};//end class TopicsHandle

}//end namespace

#endif // ROS_TOPICS_H
