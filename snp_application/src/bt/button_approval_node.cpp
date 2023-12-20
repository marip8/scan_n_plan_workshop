#include "bt/button_approval_node.h"
#include "bt/utils.h"

#include <QAbstractButton>

namespace snp_application
{
ButtonApprovalNode::ButtonApprovalNode(const std::string& instance_name,
                                       const BT::NodeConfig& config)
  : BT::StatefulActionNode(instance_name, config)
  , complete_(false)
  , cancel_(false)
{
  auto button_key = getBTInput<std::string>(this, BUTTON_PORT_KEY);
  auto button = this->config().blackboard->get<QAbstractButton*>(button_key);

  auto cancel_button = this->config().blackboard->get<QAbstractButton*>("cancel_button");

  // Connect
  QObject::connect(button, &QAbstractButton::clicked, [this](const bool){ complete_ = true; });
  QObject::connect(cancel_button, &QAbstractButton::clicked, [this](const bool){ cancel_ = true; });
}

BT::NodeStatus ButtonApprovalNode::onStart()
{
  std::cout << "-- " << name() << " started --" << std::endl;
  complete_ = false;
  cancel_ = false;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ButtonApprovalNode::onRunning()
{
  if(cancel_)
  {
    std::cout << "-- " << name() << " cancelled --" << std::endl;
    complete_ = false;
    cancel_ = false;
    return BT::NodeStatus::FAILURE;
  }

  if (complete_)
  {
    std::cout << "-- " << name() << " finished --" << std::endl;
    complete_ = false;
    cancel_ = false;
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

void ButtonApprovalNode::onHalted()
{
  std::cout << "-- " << name() << " halted --" << std::endl;
}

} // namespace snp_application
