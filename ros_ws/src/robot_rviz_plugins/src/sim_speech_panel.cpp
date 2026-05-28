#include "robot_rviz_plugins/sim_speech_panel.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

namespace robot_rviz_plugins
{

SimSpeechPanel::SimSpeechPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * layout = new QVBoxLayout();

  hint_label_ = new QLabel(
    "模拟语音（调试）：选择指令后点「发送」，文本将发布到\n"
    "/interaction/speech_text → llm_router → command_executor");
  hint_label_->setWordWrap(true);
  layout->addWidget(hint_label_);

  const QStringList presets = {
    "进行房间巡检",
    "帮我拿个水杯",
    "帮我拿个花瓶",
    "停止",
  };

  for (const QString & text : presets) {
    auto * btn = new QPushButton(text);
    preset_buttons_.push_back(btn);
    layout->addWidget(btn);
    connect(btn, &QPushButton::clicked, [this, text]() {
      if (custom_edit_) {
        custom_edit_->setText(text);
      }
    });
  }

  custom_edit_ = new QLineEdit();
  custom_edit_->setPlaceholderText("自定义句子（可编辑后发送）");
  layout->addWidget(custom_edit_);

  send_button_ = new QPushButton("发送 → LLM");
  layout->addWidget(send_button_);
  connect(send_button_, &QPushButton::clicked, this, &SimSpeechPanel::on_send_clicked);

  setLayout(layout);
}

void SimSpeechPanel::onInitialize()
{
  rviz_common::Panel::onInitialize();
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  if (!node_->has_parameter("sim_speech_topic")) {
    node_->declare_parameter("sim_speech_topic", speech_topic_);
  }
  speech_topic_ = node_->get_parameter("sim_speech_topic").as_string();
  speech_pub_ = node_->create_publisher<std_msgs::msg::String>(speech_topic_, 10);
  RCLCPP_INFO(
    node_->get_logger(), "[SimSpeechPanel] 就绪，发布话题: %s", speech_topic_.c_str());
}

std::string SimSpeechPanel::selected_text() const
{
  if (!custom_edit_) {
    return {};
  }
  return custom_edit_->text().trimmed().toStdString();
}

void SimSpeechPanel::on_send_clicked()
{
  if (!speech_pub_ || !node_) {
    return;
  }
  const std::string text = selected_text();
  if (text.empty()) {
    RCLCPP_WARN(node_->get_logger(), "[SimSpeechPanel] 句子为空，未发送");
    return;
  }
  std_msgs::msg::String msg;
  msg.data = text;
  speech_pub_->publish(msg);
  RCLCPP_INFO(node_->get_logger(), "[SimSpeechPanel] 已发送: %s", text.c_str());
}

}  // namespace robot_rviz_plugins

PLUGINLIB_EXPORT_CLASS(robot_rviz_plugins::SimSpeechPanel, rviz_common::Panel)
