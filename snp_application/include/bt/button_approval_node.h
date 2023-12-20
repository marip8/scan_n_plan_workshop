#pragma once

#include <atomic>
#include <behaviortree_cpp/action_node.h>

namespace snp_application
{
class ButtonApprovalNode : public BT::StatefulActionNode
{
public:
  inline static std::string BUTTON_PORT_KEY = "button";
  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>(BUTTON_PORT_KEY) };
  }

  explicit ButtonApprovalNode(const std::string& instance_name,
                              const BT::NodeConfig& config);

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::atomic_bool complete_;
  std::atomic_bool cancel_;
};

} // namespace snp_application
