/*
 * Copyright 2018 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 * You may not use this file except in compliance with the License.
 * A copy of the License is located at
 *
 *  http://aws.amazon.com/apache2.0
 *
 * or in the "license" file accompanying this file. This file is distributed
 * on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
 * express or implied. See the License for the specific language governing
 * permissions and limitations under the License.
 */

#include <aws/core/Aws.h>
#include <aws/core/utils/logging/LogMacros.h>
#include <aws/core/utils/logging/AWSLogging.h>

#include <aws_common/sdk_utils/client_configuration_provider.h>
#include <aws_ros2_common/sdk_utils/logging/aws_ros_logger.h>
#include <aws_ros2_common/sdk_utils/ros2_node_parameter_reader.h>
#include <cloudwatch_logger/log_node.h>
#include <cloudwatch_logs_common/log_manager.h>
#include <cloudwatch_logs_common/log_manager_factory.h>
#include <cloudwatch_logs_common/log_publisher.h>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/log.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <iostream>

constexpr char kNodeName[] = "cloudwatch_logger";
constexpr size_t kNodeSubQueueSize = 100;
constexpr char kNodeRosoutAggregatedTopicName[] = "rosout_agg";

const Aws::Client::ParameterPath kNodeParamLogStreamNameKey{"log_stream_name"};
const Aws::Client::ParameterPath kNodeParamPublishFrequencyKey{"publish_frequency"};
const Aws::Client::ParameterPath kNodeParamSubscribeToRosoutKey{"sub_to_rosout"};
const Aws::Client::ParameterPath kNodeParamLogGroupNameKey{"log_group_name"};
const Aws::Client::ParameterPath kNodeParamLogTopicsListKey{"topics"};
const Aws::Client::ParameterPath kNodeParamMinLogVerbosityKey{"min_log_verbosity"};

constexpr char kNodeLogGroupNameDefaultValue[] = "ros_log_group";
constexpr char kNodeLogStreamNameDefaultValue[] = "ros_log_stream";
constexpr int8_t kNodeMinLogVerbosityDefaultValue = rcl_interfaces::msg::Log::DEBUG;
constexpr double kNodePublishFrequencyDefaultValue = 5.0;
constexpr bool kNodeSubscribeToRosoutDefaultValue = true;

Aws::AwsError ReadPublishFrequency(
  const std::shared_ptr<Aws::Client::ParameterReaderInterface> & parameter_reader,
  double & publish_frequency)
{
  Aws::AwsError ret = parameter_reader->ReadParam(kNodeParamPublishFrequencyKey, publish_frequency);
  if (ret == Aws::AWS_ERR_NOT_FOUND) {
    publish_frequency = kNodePublishFrequencyDefaultValue;
    AWS_LOGSTREAM_WARN(__func__,
                       "Publish frequency configuration not found, setting to default value: " << publish_frequency);
  } else {
    AWS_LOGSTREAM_INFO(__func__, "Publish frequency is set to: " << publish_frequency);
  }

  return ret;
}

Aws::AwsError ReadLogGroup(
        const std::shared_ptr<Aws::Client::ParameterReaderInterface> & parameter_reader,
        std::string & log_group)
{
  Aws::AwsError ret = parameter_reader->ReadParam(kNodeParamLogGroupNameKey, log_group);
  if (ret == Aws::AWS_ERR_NOT_FOUND) {
    log_group = kNodeLogGroupNameDefaultValue;
    AWS_LOGSTREAM_WARN(__func__,
                       "Log group name configuration not found, setting to default value: "
                         << kNodeLogGroupNameDefaultValue);
  } else {
    AWS_LOGSTREAM_INFO(__func__, "Log group name is set to: " << log_group);
  }
  return ret;
}

Aws::AwsError ReadLogStream(
        const std::shared_ptr<Aws::Client::ParameterReaderInterface> & parameter_reader,
        std::string & log_stream)
{
  Aws::AwsError ret = parameter_reader->ReadParam(kNodeParamLogStreamNameKey, log_stream);
  if (ret == Aws::AWS_ERR_NOT_FOUND) {
    log_stream = kNodeLogStreamNameDefaultValue;
    AWS_LOGSTREAM_WARN(__func__,
                       "Log stream name configuration not found, setting to default value: "
                         << kNodeLogStreamNameDefaultValue);
  } else {
    AWS_LOGSTREAM_INFO(__func__, "Log stream name is set to: " << log_stream);
  }
  return ret;
}

Aws::AwsError ReadSubscribeToRosout(
  const std::shared_ptr<Aws::Client::ParameterReaderInterface> & parameter_reader,
  bool & subscribe_to_rosout)
{
  Aws::AwsError ret =
    parameter_reader->ReadParam(kNodeParamSubscribeToRosoutKey, subscribe_to_rosout);
  if (ret == Aws::AWS_ERR_NOT_FOUND) {
    subscribe_to_rosout = kNodeSubscribeToRosoutDefaultValue;
    AWS_LOGSTREAM_WARN(
      __func__,
      "Whether to subscribe to rosout_agg topic configuration not found, setting to default value: "
        << kNodeSubscribeToRosoutDefaultValue);
  } else {
    AWS_LOGSTREAM_INFO(
      __func__, "Whether to subscribe to rosout_agg topic is set to: " << subscribe_to_rosout);
  }
  return ret;
}

Aws::AwsError ReadMinLogVerbosity(
  const std::shared_ptr<Aws::Client::ParameterReaderInterface> & parameter_reader,
  int8_t & min_log_verbosity)
{
  min_log_verbosity = kNodeMinLogVerbosityDefaultValue;

  std::string specified_verbosity;
  Aws::AwsError ret =
    parameter_reader->ReadParam(kNodeParamMinLogVerbosityKey, specified_verbosity);
  if (ret == Aws::AWS_ERR_NOT_FOUND) {
    AWS_LOGSTREAM_WARN(__func__, "Log verbosity configuration not found, setting to default value: "
                                   << kNodeMinLogVerbosityDefaultValue);
  } else {
    if ("DEBUG" == specified_verbosity) {
      min_log_verbosity = rcl_interfaces::msg::Log::DEBUG;
      AWS_LOG_INFO(__func__, "Log verbosity is set to DEBUG.");
    } else if ("INFO" == specified_verbosity) {
      min_log_verbosity = rcl_interfaces::msg::Log::INFO;
      AWS_LOG_INFO(__func__, "Log verbosity is set to INFO.");
    } else if ("WARN" == specified_verbosity) {
      min_log_verbosity = rcl_interfaces::msg::Log::WARN;
      AWS_LOG_INFO(__func__, "Log verbosity is set to WARN.");
    } else if ("ERROR" == specified_verbosity) {
      min_log_verbosity = rcl_interfaces::msg::Log::ERROR;
      AWS_LOG_INFO(__func__, "Log verbosity is set to ERROR.");
    } else if ("FATAL" == specified_verbosity) {
      min_log_verbosity = rcl_interfaces::msg::Log::FATAL;
      AWS_LOG_INFO(__func__, "Log verbosity is set to FATAL.");
    } else {
      AWS_LOGSTREAM_WARN(__func__,
                         "Log verbosity configuration not valid, setting to default value: "
                           << kNodeMinLogVerbosityDefaultValue);
    }
  }

  return ret;
}

Aws::AwsError ReadSubscriberList(
  const std::shared_ptr<Aws::Client::ParameterReaderInterface> & parameter_reader,
  std::vector<rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr> & subscriptions,
  std::shared_ptr<Aws::CloudWatchLogs::Utils::LogNode> & cloudwatch_logger_node,
  bool & subscribe_to_rosout,
  std::function<void(const rcl_interfaces::msg::Log::SharedPtr)> callback)
{
  std::vector<std::string> topics;
  Aws::AwsError ret = parameter_reader->ReadParam(kNodeParamLogTopicsListKey, topics);

  for (const auto & topic: topics) {
    if (topic != std::string(kNodeRosoutAggregatedTopicName)) {
      subscriptions.emplace_back(cloudwatch_logger_node->create_subscription<rcl_interfaces::msg::Log>(
          topic, callback, kNodeSubQueueSize));
      AWS_LOGSTREAM_INFO(__func__, "Subscribing to topic: " << topic);
    }
  }
  if (subscribe_to_rosout) {
    subscriptions.emplace_back(cloudwatch_logger_node->create_subscription<rcl_interfaces::msg::Log>(
        kNodeRosoutAggregatedTopicName, callback, kNodeSubQueueSize));

    AWS_LOG_INFO(__func__, "Subscribing to rosout_agg");
  }

  return ret;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto cloudwatch_logger_node = std::make_shared<Aws::CloudWatchLogs::Utils::LogNode>(kNodeName);

  Aws::Utils::Logging::InitializeAWSLogging(
    Aws::MakeShared<Aws::Utils::Logging::AWSROSLogger>(
            kNodeName,
            Aws::Utils::Logging::LogLevel::Debug,
            cloudwatch_logger_node));
  AWS_LOGSTREAM_INFO(__func__, "Starting " << kNodeName << "...");

  // required values
  double publish_frequency;
  std::string log_group;
  std::string log_stream;
  bool subscribe_to_rosout;
  int8_t min_log_verbosity;
  std::vector<rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr> subscriptions;

  std::shared_ptr<Aws::Client::ParameterReaderInterface> parameter_reader =
    std::make_shared<Aws::Client::Ros2NodeParameterReader>(cloudwatch_logger_node);

  // checking configurations to set values or set to default values;
  ReadPublishFrequency(parameter_reader, publish_frequency);
  ReadLogGroup(parameter_reader, log_group);
  ReadLogStream(parameter_reader, log_stream);
  ReadSubscribeToRosout(parameter_reader, subscribe_to_rosout);
  ReadMinLogVerbosity(parameter_reader, min_log_verbosity);

  // configure aws settings
  Aws::Client::ClientConfigurationProvider client_config_provider(parameter_reader);
  Aws::Client::ClientConfiguration config = client_config_provider.GetClientConfiguration();
  Aws::SDKOptions sdk_options;

  cloudwatch_logger_node->Initialize(log_group, log_stream, config, sdk_options, min_log_verbosity);

  // callback function
  std::function<void(const rcl_interfaces::msg::Log::SharedPtr &)> callback;
  callback = [cloudwatch_logger_node](const rcl_interfaces::msg::Log::SharedPtr log_msg) -> void {
    std::cout << "got message" << std::endl;
    cloudwatch_logger_node->RecordLogs(log_msg);
  };

  // subscribe to additional topics, if any
  ReadSubscriberList(parameter_reader, subscriptions, cloudwatch_logger_node, subscribe_to_rosout, callback);
  AWS_LOGSTREAM_INFO(__func__, "Initialized " << kNodeName << ".");

  // a ros timer that triggers log publisher to publish periodically
  auto timer = cloudwatch_logger_node->create_wall_timer(
      std::chrono::duration_cast<std::chrono::duration<int64_t>>(std::chrono::duration<double>(publish_frequency)),
      std::bind(&Aws::CloudWatchLogs::Utils::LogNode::TriggerLogPublisher, cloudwatch_logger_node));
  rclcpp::spin(cloudwatch_logger_node);
  AWS_LOGSTREAM_INFO(__func__, "Shutting down " << kNodeName << ".");
  Aws::Utils::Logging::ShutdownAWSLogging();
  rclcpp::shutdown();
  return 0;
}
