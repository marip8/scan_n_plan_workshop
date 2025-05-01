#pragma once

#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/loggers/abstract_logger.h>
#include <QWidget>
#include <rclcpp/node.hpp>
#include <rclcpp_action/client.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

namespace Ui
{
class SNPWidget;
}

class QStackedWidget;
class QTextEdit;

namespace snp_application
{
class SNPWidget : public QWidget
{
public:
  explicit SNPWidget(rclcpp::Node::SharedPtr rviz_node, QWidget* parent = nullptr);

protected:
  void runTreeWithThread(const std::string& bt_tree_name);

  virtual BT::BehaviorTreeFactory createBTFactory(int ros_timeout);
  QStackedWidget* getStackedWidget();
  QTextEdit* getTextEdit();

  /**
   * @brief Node provided to the behavior tree
   * @details In versions of `rclcpp` that do not support the spinning of callback groups with executors, a new node instance
   * (rather than the Rviz node) must be provided to the behavior tree, which attempts to spin the node directly. In
   * later versions of `rclcpp` that do support the spinning of callback groups with executors, this simply points to the Rviz node,
   * and the behavior tree will spin a callback group instead of spinning the node itself.
   */
  rclcpp::Node::SharedPtr bt_node_;

  Ui::SNPWidget* ui_;
  BT::Blackboard::Ptr board_;
  std::shared_ptr<BT::StatusChangeLogger> logger_;
};

}  // namespace snp_application
