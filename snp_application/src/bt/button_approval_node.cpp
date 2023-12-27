#include "bt/button_approval_node.h"
#include "bt/utils.h"

#include <QAbstractButton>

namespace snp_application
{
ButtonApprovalNode::ButtonApprovalNode(const std::string& instance_name,
                                       const BT::NodeConfig& config)
  : BT::StatefulActionNode(instance_name, config)
  , approved_(false)
  , disapproved_(false)
{
  auto approve_button_key = getBTInput<std::string>(this, APPROVE_BUTTON_PORT_KEY);
  approve_button_ = this->config().blackboard->get<QAbstractButton*>(approve_button_key);
  QObject::connect(approve_button_, &QAbstractButton::clicked, [this](const bool){ approved_ = true; });

  auto disapprove_button_key = getBTInput<std::string>(this, DISAPPROVE_BUTTON_PORT_KEY);
  disapprove_button_ = this->config().blackboard->get<QAbstractButton*>(disapprove_button_key);
  QObject::connect(disapprove_button_, &QAbstractButton::clicked, [this](const bool){ disapproved_ = true; });
}

BT::NodeStatus ButtonApprovalNode::onStart()
{
  std::cout << "-- " << name() << " started --" << std::endl;
  approved_ = false;
  disapproved_ = false;

  approve_button_->setEnabled(true);
  disapprove_button_->setEnabled(true);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ButtonApprovalNode::onRunning()
{
  if(disapproved_)
  {
    std::cout << "-- " << name() << " not approved --" << std::endl;
    approve_button_->setEnabled(false);
    disapprove_button_->setEnabled(false);
    return BT::NodeStatus::FAILURE;
  }

  if (approved_)
  {
    std::cout << "-- " << name() << " approved --" << std::endl;
    approve_button_->setEnabled(false);
    disapprove_button_->setEnabled(false);
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

void ButtonApprovalNode::onHalted()
{
  std::cout << "-- " << name() << " halted --" << std::endl;
  approve_button_->setEnabled(false);
  disapprove_button_->setEnabled(false);
}

} // namespace snp_application
