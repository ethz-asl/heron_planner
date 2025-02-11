#include "heron_ui/BTPanel.h"

#include <ros/names.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QTimer>
#include <QVBoxLayout>
#include <string>
#include <vector>

namespace heron_ui {
BTPanel::BTPanel(QWidget *parent)
    : rviz::Panel(parent),
      start_button_(new QPushButton("Start BT")),
      pause_button_(new QPushButton("Pause BT")),
      stop_button_(new QPushButton("Stop BT")) {
  auto *layout = new QVBoxLayout;
  layout->addWidget(start_button_);
  layout->addWidget(pause_button_);
  layout->addWidget(stop_button_);
  setLayout(layout);

  connect(start_button_, SIGNAL(clicked()), this, SLOT(onStartButtonClicked()));
  connect(pause_button_, SIGNAL(clicked()), this, SLOT(onPauseButtonClicked()));
  connect(stop_button_, SIGNAL(clicked()), this, SLOT(onStopButtonClicked()));
  
  start_button_->setStyleSheet("background-color: green; color: black;");
  pause_button_->setStyleSheet("background-color: orange; color: black;");
  stop_button_->setStyleSheet("background-color: red; color: black;");
}

void BTPanel::save(rviz::Config config) const {
  rviz::Panel::save(config);  // Ensure the base class's save method is called
}

void BTPanel::load(const rviz::Config &config) {
  rviz::Panel::load(config);  // Ensure the base class's load method is called
}

void BTPanel::onStartButtonClicked() {
  ros::NodeHandle nh;
  ros::ServiceClient start_bt_client =
      nh.serviceClient<std_srvs::Trigger>("/hlp/start");
  std_srvs::Trigger srv;

  is_started_ = !is_started_;

  if (is_started_) {
    start_button_->setText("BT Started");
  } else {
    start_button_->setText("Start BT");
  }

  if (!start_bt_client.call(srv)) {
    ROS_WARN("Failed to call service /hlp/start");
  }
}

void BTPanel::onPauseButtonClicked() {
  ros::NodeHandle nh;
  ros::ServiceClient pause_bt_client =
      nh.serviceClient<std_srvs::Trigger>("/hlp/pause");
  std_srvs::Trigger srv;

  is_paused_ = !is_paused_;

  if (is_paused_) {
    pause_button_->setText("Continue BT");
  } else {
    pause_button_->setText("Pause BT");
  }

  if (!pause_bt_client.call(srv)) {
    ROS_WARN("Failed to call service /hlp/pause");
  }
}

void BTPanel::onStopButtonClicked() {
  ros::NodeHandle nh;
  ros::ServiceClient stop_bt_client =
      nh.serviceClient<std_srvs::Trigger>("/hlp/stop");
  std_srvs::Trigger srv;

  is_stopped_ = !is_stopped_;

  if (is_stopped_) {
    stop_button_->setText("BT Stopped");
  } else {
    stop_button_->setText("Stop BT");
  }

  if (!stop_bt_client.call(srv)) {
    ROS_WARN("Failed to call service /hlp/stop");
  }
}
}  // namespace heron_ui

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(heron_ui::BTPanel, rviz::Panel)
