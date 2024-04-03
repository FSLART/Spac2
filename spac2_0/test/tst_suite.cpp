/** ==========================================
* @title:  Testing Main File
* @author: João Vieira
* @brief Last Updated:   2023-07-31
==========================================**/

#include <gtest/gtest.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>


static rclcpp::Node::SharedPtr node_;



int main(int argc, char** argv){
	rclcpp::init(argc, argv);
	

	testing::InitGoogleTest(&argc, argv);
	int ret = RUN_ALL_TESTS();
	

	rclcpp::shutdown();
	return ret;
}
