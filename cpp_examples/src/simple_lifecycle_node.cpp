#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>>

class SimpleLifecycleNode : public rclcpp_lifecycle::LifecycleNode{
public:
    explicit SimpleLifecycleNode(const std::string &node_name, bool intra_process_comms = false) 
    : rclcpp_lifecycle::LifecycleNode(node_name,rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
    {

    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &)

};