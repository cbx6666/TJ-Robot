#ifndef ROBOT_RVIZ_PLUGINS__SIM_SPEECH_PANEL_HPP_
#define ROBOT_RVIZ_PLUGINS__SIM_SPEECH_PANEL_HPP_

#include <memory>
#include <string>
#include <vector>

#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/string.hpp>

namespace robot_rviz_plugins
{

class SimSpeechPanel : public rviz_common::Panel
{
public:
  explicit SimSpeechPanel(QWidget * parent = nullptr);
  void onInitialize() override;

private:
  void on_send_clicked();
  std::string selected_text() const;

  QLabel * hint_label_{nullptr};
  QPushButton * send_button_{nullptr};
  QLineEdit * custom_edit_{nullptr};
  std::vector<QPushButton *> preset_buttons_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr speech_pub_;
  std::string speech_topic_{"/interaction/speech_text"};
};

}  // namespace robot_rviz_plugins

#endif  // ROBOT_RVIZ_PLUGINS__SIM_SPEECH_PANEL_HPP_
