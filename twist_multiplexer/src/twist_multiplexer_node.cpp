// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 Wimblerobotics
// https://github.com/wimblerobotics/Sigyn

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
    if (currentMessage.priority == -1) {
        RCUTILS_LOG_DEBUG("[twist_multiplexer_node] Accepting new message: no previous message");
        return true; // No message seen yet.
    }
    std::chrono::duration<double> secondsSinceLastMessage = std::chrono::steady_clock::now() - lastCommandPoppedTime;
    if (priority > currentMessage.priority) {
        RCUTILS_LOG_DEBUG("[twist_multiplexer_node] Accepting new message: higher priority (%d > %d)", priority, currentMessage.priority);
        return true;
    }
    if (currentMessage.topic == topic) {
        RCUTILS_LOG_DEBUG("[twist_multiplexer_node] Accepting new message: same topic (%s)", topic.c_str());
        return true;
    }
    if (secondsSinceLastMessage.count() > currentMessage.timeout) {
        RCUTILS_LOG_DEBUG("[twist_multiplexer_node] Accepting new message: timeout expired (%.2f > %.2f)", secondsSinceLastMessage.count(), currentMessage.timeout);
        return true;
    }
    RCUTILS_LOG_DEBUG("[twist_multiplexer_node] Rejecting new message: lower priority and not timed out");
    return false;
}

class CmdVelSubscriber
{
public:
    CmdVelSubscriber(string nodeName = "<NoName>")
    {
        priority_ = -1;
        subscription_ = nullptr;
        nodeName_ = nodeName;
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
    string nodeName_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;

    void topicCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (!deadmanSwitch)
        {
            RCUTILS_LOG_DEBUG("[twist_multiplexer_node] Received Twist message on topic: %s, priority: %d", topic_.c_str(), priority_);
            CmdVelMessage m(*msg, priority_, timeout_, topic_);
            if (canAcceptNewMessage(topic_, priority_))
            {
                RCUTILS_LOG_DEBUG("[twist_multiplexer_node] Pushing message to queue from topic: %s", topic_.c_str());
                messageQueue.push(m);
            }
            else
            {
                RCUTILS_LOG_DEBUG("[twist_multiplexer_node] Message from topic %s not accepted (priority: %d)", topic_.c_str(), priority_);
            }
        }
        else
        {
            RCUTILS_LOG_DEBUG("[twist_multiplexer_node] Deadman switch active, ignoring Twist message on topic: %s", topic_.c_str());
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
    bool prevDeadman = deadmanSwitch;
    // L2 trigger is axis2_ud: +32767 when pressed, negative when released
    // Deadman is TRUE when L2 is NOT pressed (blocks joystick)
    deadmanSwitch = msg->axis2_ud < 16000;  // L2 not pressed -> block joystick
    RCUTILS_LOG_DEBUG("[twist_multiplexer_node] BluetoothJoystick callback: axis2_ud=%d, deadmanSwitch=%d", msg->axis2_ud, deadmanSwitch);
    if (deadmanSwitch && !prevDeadman) {
        // Deadman just activated (L2 released): clear queue and reset current message
        RCUTILS_LOG_DEBUG("[twist_multiplexer_node] Deadman switch activated: clearing joystick messages from queue and stopping output");
        // Remove all messages from cmd_vel_joystick from the queue
        std::priority_queue<CmdVelMessage> newQueue;
        while (!messageQueue.empty()) {
            CmdVelMessage m = messageQueue.top();
            messageQueue.pop();
            if (m.topic != "cmd_vel_joystick") {
                newQueue.push(m);
            }
        }
        messageQueue = std::move(newQueue);
        // If the current message is from joystick, reset it
        if (currentMessage.topic == "cmd_vel_joystick") {
            currentMessage = CmdVelMessage();
        }
    }
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
            // If deadman is active, skip joystick messages
            if (deadmanSwitch && messageQueue.top().topic == "cmd_vel_joystick") {
                RCUTILS_LOG_DEBUG("[twist_multiplexer_node] Deadman switch active: skipping joystick message");
                messageQueue.pop();
                continue;
            }
            currentMessage = messageQueue.top();
            // Prevent publishing if deadman is active and current message is from joystick
            if (deadmanSwitch && currentMessage.topic == "cmd_vel_joystick") {
                RCUTILS_LOG_DEBUG("[twist_multiplexer_node] Deadman switch active: not publishing joystick message");
                messageQueue.pop();
                currentMessage = CmdVelMessage();
                continue;
            }
            RCUTILS_LOG_DEBUG("[twist_multiplexer_node] Publishing Twist message from topic: %s, priority: %d", currentMessage.topic.c_str(), currentMessage.priority);
            twistPublisher->publish(currentMessage.message);
            messageQueue.pop();
            lastCommandPoppedTime = std::chrono::steady_clock::now();
        }
        else {
            RCLCPP_DEBUG_THROTTLE(rosNode->get_logger(), *rosNode->get_clock(), 2000, "[twist_multiplexer_node] Message queue empty, nothing to publish");
        }
        rclcpp::spin_some(rosNode);
        loopRate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}