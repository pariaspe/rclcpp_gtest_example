// Copyright 2024 Pedro Arias-Perez
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RCLCPP_GTEST_EXAMPLE__ADD_TWO_INTS_SERVER_HPP_
#define RCLCPP_GTEST_EXAMPLE__ADD_TWO_INTS_SERVER_HPP_

#include <chrono>
#include <cstdlib>
#include <memory>

#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

class AddTwoIntsServer : public rclcpp::Node {
public:
  AddTwoIntsServer();
  ~AddTwoIntsServer();

private:
  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_;

  void
  add(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request>
          request,
      std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response);
};

#endif  // RCLCPP_GTEST_EXAMPLE__ADD_TWO_INTS_SERVER_HPP_
