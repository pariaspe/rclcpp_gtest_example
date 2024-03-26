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
