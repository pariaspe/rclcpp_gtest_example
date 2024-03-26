// MIT License

// Copyright (c) 2024 Pedro Arias-Perez

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "rclcpp_gtest_example/add_two_ints_client.hpp"

AddTwoIntsClient::AddTwoIntsClient() : Node("add_two_ints_client") {
  client_ =
      this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
}

AddTwoIntsClient::~AddTwoIntsClient() {}

rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture
AddTwoIntsClient::call_client(int a, int b) {
  auto request =
      std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
  request->a = a;
  request->b = b;

  while (!client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Interrupted while waiting for the service. Exiting.");
      return rclcpp::Client<
          example_interfaces::srv::AddTwoInts>::SharedFuture();
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "service not available, waiting again...");
  }

  auto result_future = client_->async_send_request(request);
  return result_future;
}
