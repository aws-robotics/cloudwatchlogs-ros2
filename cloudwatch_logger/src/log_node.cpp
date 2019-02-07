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

#include <aws/core/utils/logging/AWSLogging.h>
#include <aws/core/utils/logging/LogMacros.h>
#include <aws_common/sdk_utils/client_configuration_provider.h>
#include <aws_ros2_common/sdk_utils/logging/aws_ros_logger.h>
#include <aws_ros2_common/sdk_utils/ros2_node_parameter_reader.h>
#include <cloudwatch_logger/log_node.h>
#include <cloudwatch_logs_common/log_manager.h>
#include <cloudwatch_logs_common/log_manager_factory.h>
#include <cloudwatch_logs_common/log_publisher.h>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/log.hpp>

using namespace Aws::CloudWatchLogs::Utils;

LogNode::LogNode(const std::string & node_name) : rclcpp::Node(node_name)
{
  this->log_manager_ = nullptr;
  this->min_log_severity_ = rcl_interfaces::msg::Log::DEBUG;
}

LogNode::~LogNode() { this->log_manager_ = nullptr; }

void LogNode::Initialize(
    const std::string & log_group,
    const std::string & log_stream,
    Aws::Client::ClientConfiguration & config,
    Aws::SDKOptions & sdk_options,
    const int8_t min_log_severity)
{
  LogManagerFactory factory;
  this->log_manager_ = factory.CreateLogManager(log_group, log_stream, config, sdk_options);
  this->min_log_severity_ = min_log_severity;
}

void LogNode::RecordLogs(const rcl_interfaces::msg::Log::SharedPtr log_msg)
{
  if (log_msg->name != "/cloudwatch_logger" && log_msg->name != "/cloudwatch_metrics_collector") {
    if (nullptr == this->log_manager_) {
      AWS_LOG_ERROR(__func__,
                    "Cannot publish CloudWatch logs with NULL CloudWatch LogManager instance.");
      return;
    }
    if (ShouldSendToCloudWatchLogs(log_msg->level)) {
      this->log_manager_->RecordLog(FormatLogs(log_msg));
    }
  }
}

void LogNode::TriggerLogPublisher() { this->log_manager_->Service(); }

bool LogNode::ShouldSendToCloudWatchLogs(const int8_t log_severity_level)
{
  return log_severity_level >= this->min_log_severity_;
}

const std::string LogNode::FormatLogs(const rcl_interfaces::msg::Log::SharedPtr & log_msg)
{
  std::stringstream ss;
  ss << std::chrono::duration_cast<std::chrono::duration<double>>(
      std::chrono::seconds(log_msg->stamp.sec) +
      std::chrono::nanoseconds(log_msg->stamp.nanosec)
      ).count() << " ";

  switch (log_msg->level) {
    case rcl_interfaces::msg::Log::FATAL:
      ss << "FATAL ";
      break;
    case rcl_interfaces::msg::Log::ERROR:
      ss << "ERROR ";
      break;
    case rcl_interfaces::msg::Log::WARN:
      ss << "WARN ";
      break;
    case rcl_interfaces::msg::Log::DEBUG:
      ss << "DEBUG ";
      break;
    case rcl_interfaces::msg::Log::INFO:
      ss << "INFO ";
      break;
    default:
      ss << log_msg->level << " ";
  }
  ss << "[node name: " << log_msg->name << "] ";

  ss << log_msg->msg << "\n";
  std::cout << log_msg->msg << std::endl;

  return ss.str();
}