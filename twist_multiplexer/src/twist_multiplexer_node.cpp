#include <chrono>
#include <cstdio>
#include <fstream>
#include <geometry_msgs/msg/twist.hpp>
#include <getopt.h>
#include <iostream>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging_macros.h>
#include <string>
#include <unistd.h>
#include <vector>
#include "yaml-cpp/yaml.h"

#include "msgs/msg/bluetooth_joystick.hpp"

using namespace std;
using namespace YAML;
using namespace std::placeholders;

rclcpp::Node::SharedPtr rosNode;

typedef struct CmdVelMessage
{
    geometry_msgs::msg::Twist message;
    int priority;
    float timeout;
    string topic;

    CmdVelMessage()
    {
        priority = -1;
    }

    CmdVelMessage(geometry_msgs::msg::Twist msg, int p, float t, string to)
    {
        message = msg;
        priority = p;
        timeout = t;
        topic = to;
    }

    bool operator<(const CmdVelMessage &other) const
    {
        return priority < other.priority;
    }
} CmdVelMessage;

bool deadmanSwitch = false;
rclcpp::Subscription<msgs::msg::BluetoothJoystick>::SharedPtr joystickSubscriber;
double loopRateHz;
priority_queue<CmdVelMessage> messageQueue;

CmdVelMessage currentMessage = CmdVelMessage();
auto lastCommandPoppedTime = std::chrono::steady_clock::now();

bool canAcceptNewMessage(const string &topic, int priority)
{
    // Can accept a new message if any of the following is true:
    // 1. No previous messag) has been received yet.
    // 2. The current message is a higher priority than the previous message.
    // 3. The current message is the same topic as the previous message.
    // 4. The previous message timeout has expired.
    if (currentMessage.priority == -1)
        return true; // No message seen yet.

    std::chrono::duration<double> secondsSinceLastMessage = std::chrono::steady_clock::now() - lastCommandPoppedTime;
    if (priority > currentMessage.priority) return true;
    if (currentMessage.topic == topic)
        return true;
    if (secondsSinceLastMessage.count() > currentMessage.timeout)
        return true;
    else
        return false;
}

class CmdVelSubscriber : public rclcpp::Node
{
public:
    CmdVelSubscriber(string nodeName = "<NoName>") : Node(nodeName)
    {
        priority_ = -1;
        subscription_ = nullptr;
    }

    void init(string &topic, int priority, float timeout)
    {
        priority_ = priority;
        timeout_ = timeout;
        topic_ = topic;
        subscription_ = rosNode->create_subscription<geometry_msgs::msg::Twist>(
            topic, 10, std::bind(&CmdVelSubscriber::topicCallback, this, placeholders::_1));
    }

    int priority_;
    float timeout_;
    string topic_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;

    void topicCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (!deadmanSwitch)
        {
            CmdVelMessage m(*msg, priority_, timeout_, topic_);
            if (canAcceptNewMessage(topic_, priority_))
            {
                messageQueue.push(m);
            }
        }
    }
};

typedef struct MultiplexerElement
{
    string name;
    string topic;
    int priority;
    float timeout;
    CmdVelSubscriber* subscriber;

    MultiplexerElement(string name, string topic, int priority, float timeout,
                       CmdVelSubscriber* subscriber)
    {
        this->name = name;
        this->topic = topic;
        this->priority = priority;
        this->timeout = timeout;
        this->subscriber = subscriber;
    }
} MultiplexerElement;

vector<MultiplexerElement> multiplexerElements;

rclcpp::QoS qos(
    rclcpp::QoSInitialization(
        RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        10));

template <typename T>
T getTopicElementValue(YAML::Node topics, int sequence, string elementName)
{
    try
    {
        return topics[sequence][elementName.c_str()].as<T>();
    }
    catch (Exception &e)
    {
        RCUTILS_LOG_FATAL("[twist_multiplexer_node] topic sequence %d does not have a valid '%s' value", (int)sequence, elementName.c_str());
        exit(-1);
    }
}

void processConfiguration(int argc, char *argv[])
{
    static const struct option long_options[] = {
        {"config", required_argument, NULL, 'c'},
        {"ros-args", optional_argument, NULL, 'r'},
        {NULL, 0, NULL, 0}};

    char *yamlPath = NULL;
    int opt = 0;
    int optIndex = 0;
    while ((opt = getopt_long_only(argc, argv, "c:r", long_options, &optIndex)) != -1)
    {
        switch (opt)
        {
        case 'c':
            yamlPath = optarg;
            break;
        }
    }

    ifstream yamlFile;
    yamlFile.open(yamlPath, ifstream::in);
    YAML::Node config = Load(yamlFile);
    if (!config.IsMap())
    {
        RCUTILS_LOG_FATAL("[twist_multiplexer_node] YAML file is not a map kind");
        exit(-1);
    }

    YAML::Node twistMultiplexerNode = config["twist_multiplexer_node"];
    if (!twistMultiplexerNode.IsMap())
    {
        RCUTILS_LOG_FATAL("[twist_multiplexer_node] YAML file does not contain a 'twist_multiplexer_node' map");
        exit(-1);
    }

    YAML::Node rosParameters = twistMultiplexerNode["ros__parameters"];
    if (!rosParameters.IsMap())
    {
        RCUTILS_LOG_FATAL("[twist_multiplexer_node] YAML file does not contain a 'ros__parameters' map within the 'twist_multiplexer_node' map");
        exit(-1);
    }

    YAML::Node loopRateNode = rosParameters["loop_rate"];
    if (!loopRateNode.IsScalar())
    {
        RCUTILS_LOG_FATAL("[twist_multiplexer_node] YAML file 'loop_roate' is an not an scalar");
        exit(-1);
    }

    loopRateHz = loopRateNode.as<double>();

    YAML::Node topics = rosParameters["topics"];
    if (!topics.IsSequence() || (topics.size() < 1))
    {
        RCUTILS_LOG_FATAL("[twist_multiplexer_node] YAML file 'topics' is an not an array of at least one entry");
        exit(-1);
    }

    for (std::size_t i = 0; i < topics.size(); i++)
    {
        std::string name = getTopicElementValue<string>(topics, i, "name");
        std::string topic = getTopicElementValue<string>(topics, i, "topic");
        int priority = getTopicElementValue<int>(topics, i, "priority");
        float timeout = getTopicElementValue<float>(topics, i, "timeout");

        //rclcpp::Subscription<geometry_msgs::Twist>::SharedPtr
        CmdVelSubscriber* subscriber = new CmdVelSubscriber(name);
        subscriber->init(topic, priority, timeout);
        MultiplexerElement element(name, topic, priority, timeout, subscriber);
        multiplexerElements.push_back(element);
    }
}

void bluetoothJoystickCallback(const msgs::msg::BluetoothJoystick::SharedPtr msg)
{
    deadmanSwitch = msg->button_l2 == 0;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rosNode = rclcpp::Node::make_shared("twist_multiplexer_node");

    processConfiguration(argc, argv);

    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    qos.avoid_ros_namespace_conventions(false);

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twistPublisher = rosNode->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    joystickSubscriber = rosNode->create_subscription<msgs::msg::BluetoothJoystick>("/bluetoothJoystick", qos, bluetoothJoystickCallback);

    rclcpp::Rate loopRate(loopRateHz);
    while (rclcpp::ok())
    {
        if (!messageQueue.empty())
        {
            currentMessage = messageQueue.top();
            twistPublisher->publish(currentMessage.message);
            messageQueue.pop();
            lastCommandPoppedTime = std::chrono::steady_clock::now();
        }
        rclcpp::spin_some(rosNode);
        loopRate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}