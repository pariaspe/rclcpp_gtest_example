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

#include "rclcpp/rclcpp.hpp"
#include "gtest/gtest.h"

#include "add_two_ints_client.hpp"
#include "add_two_ints_server.hpp"

/* Test fixture */
class MyTest : public testing::Test {
protected:
  std::shared_ptr<AddTwoIntsServer> server_node;
  std::shared_ptr<AddTwoIntsClient> client_node;
  rclcpp::executors::SingleThreadedExecutor executor;

  void SetUp() {
    server_node = std::make_shared<AddTwoIntsServer>();
    client_node = std::make_shared<AddTwoIntsClient>();

    executor.add_node(server_node);
    executor.add_node(client_node);
  }

  void TearDown() {
    executor.cancel();
    executor.remove_node(client_node);
    executor.remove_node(server_node);
    client_node.reset();
    server_node.reset();
  }
};

/* Test cases */
TEST_F(MyTest, CallService) {
  auto result_future = client_node->call_client(2, 3);

  executor.spin_some();

  auto result = executor.spin_until_future_complete(result_future);
  ASSERT_EQ(result, rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_EQ(result_future.get()->sum, 5);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  auto result = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return result;
}