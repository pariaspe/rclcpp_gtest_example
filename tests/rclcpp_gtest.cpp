#include <chrono>
#include <cstdlib>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "gtest/gtest.h"

#include "example_interfaces/srv/add_two_ints.hpp"

using namespace std::chrono_literals;

class ServerNode : public rclcpp::Node {
public:
  ServerNode() : rclcpp::Node("add_two_ints_server") {
    service = this->create_service<example_interfaces::srv::AddTwoInts>(
        "add_two_ints",
        std::bind(&ServerNode::add, this,
                  std::placeholders::_1, // Corresponds to the 'request'  input
                  std::placeholders::_2  // Corresponds to the 'response' input
                  ));

    RCLCPP_INFO(this->get_logger(), "Ready to add two ints.");
  }

  void
  add(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request>
          request,
      std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response) {
    response->sum = request->a + request->b;
    RCLCPP_INFO(this->get_logger(), "Incoming request\na: %ld b: %ld ",
                request->a, request->b);
    RCLCPP_INFO(this->get_logger(), "sending back response: [%ld]",
                (int64_t)response->sum);
  }

public:
  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service;
};

class ClientNode : public rclcpp::Node {
public:
  ClientNode() : rclcpp::Node("add_two_ints_client") {
    client = this->create_client<example_interfaces::srv::AddTwoInts>(
        "add_two_ints");

    RCLCPP_INFO(this->get_logger(), "Ready to send request.");
  }

public:
  rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client;
};

/* Test fixture */
class MyTest : public testing::Test {
protected:
  std::shared_ptr<ServerNode> server_node;
  std::shared_ptr<ClientNode> client_node;
  rclcpp::executors::SingleThreadedExecutor executor;

  void SetUp() {
    server_node = std::make_shared<ServerNode>();
    client_node = std::make_shared<ClientNode>();

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
  auto request =
      std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
  request->a = 2;
  request->b = 3;

  auto result_future = client_node->client->async_send_request(request);
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