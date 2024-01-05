/*
 * @Description: glog module
 * @Autor: Zijie Chen
 * @Date: 2022-10-08 19:32:27
 */

#pragma once

// #include <filesystem>
#include <iostream>
#include <string>

#include <stdlib.h>

#include <boost/filesystem.hpp>

#include <glog/logging.h>

class Logger {
 public:
  Logger(int argc, char** argv);

  Logger(int argc, char** argv, std::string current_path);

  ~Logger();
};

// for common
Logger::Logger(int argc, char** argv) {
  // config
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InitGoogleLogging(argv[0]);

  FLAGS_colorlogtostderr = true;
  FLAGS_logbufsecs = 0;
  FLAGS_max_log_size = 100;
  FLAGS_stop_logging_if_full_disk = true;
  FLAGS_alsologtostderr = true;
  FLAGS_log_prefix = true;

  std::string current_path = boost::filesystem::current_path().c_str();
  std::string log_path = current_path + "/log";
  if (!boost::filesystem::exists(log_path)) {
    boost::filesystem::create_directories(log_path);
  }

  std::cout << "log path: " << log_path << std::endl;
  FLAGS_log_dir = log_path;
}

// for ROS
Logger::Logger(int argc, char** argv, std::string current_path) {
  // config
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InitGoogleLogging(argv[0]);

  FLAGS_colorlogtostderr = true;
  FLAGS_logbufsecs = 0;
  FLAGS_max_log_size = 100;
  FLAGS_stop_logging_if_full_disk = true;
  FLAGS_alsologtostderr = true;
  FLAGS_log_prefix = true;

  std::string log_path = current_path + "/log";
  if (!boost::filesystem::exists(log_path)) {
    boost::filesystem::create_directories(log_path);
  }

  std::cout << "log path: " << log_path << std::endl;
  FLAGS_log_dir = log_path;
}

Logger::~Logger() {
  google::ShutdownGoogleLogging();
  std::cout << "Logger Is Finshed!" << std::endl;
}
