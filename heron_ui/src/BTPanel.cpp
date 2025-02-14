#include "heron_ui/BTPanel.h"

#include <ros/names.h>
#include <ros/param.h>
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
      stop_button_(new QPushButton("Stop BT")),
      choose_sequence_button_(new QPushButton("Choose Sequence")),
      sequence_label_(new QLabel("Chosen Sequence: None")),
      sequence_dropdown_(new QComboBox()),
      is_paused_(false) {
  auto *layout = new QVBoxLayout;
  layout->addWidget(start_button_);
  layout->addWidget(pause_button_);
  layout->addWidget(stop_button_);
  layout->addWidget(new QLabel("Select a pothole roller sequence:"));
  layout->addWidget(sequence_dropdown_);
  layout->addWidget(choose_sequence_button_);
  layout->addWidget(sequence_label_);
  setLayout(layout);

  connect(start_button_, SIGNAL(clicked()), this, SLOT(onStartButtonClicked()));
  connect(pause_button_, SIGNAL(clicked()), this, SLOT(onPauseButtonClicked()));
  connect(stop_button_, SIGNAL(clicked()), this, SLOT(onStopButtonClicked()));
  connect(choose_sequence_button_, SIGNAL(clicked()), this,
          SLOT(onChooseSequenceButtonClicked()));

  loadPotholeSequences();

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

void BTPanel::loadPotholeSequences() {
  ros::NodeHandle nh;
  std::vector<std::string> sequences;

  if (nh.getParam("/pothole/roller_sequence", sequences)) {
    for (const auto &seq : sequences) {
      sequence_dropdown_->addItem(QString::fromStdString(seq));
    }
  } else {
    ROS_WARN("Failed to load /pothole/roller_sequence from ROS param server!");
  }
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

void BTPanel::onChooseSequenceButtonClicked() {
  QString selected_sequence = sequence_dropdown_->currentText();
  sequence_label_->setText("Chosen sequence: " + selected_sequence);

  ros::NodeHandle nh;
  ros::ServiceClient pothole_seq_client =
      nh.serviceClient<std_srvs::Trigger>("/hlp/pothole_sequence");
  std_srvs::Trigger srv;

  if (!pothole_seq_client.call(srv)) {
     ROS_WARN("Failed to call service /hlp/pothole_sequence");
  }
  nh.setParam("/ugv/chosen_pothole_sequence", selected_sequence.toStdString());
  ROS_INFO_STREAM(
      "Chosen pothole sequence: " << selected_sequence.toStdString());
}

}  // namespace heron_ui

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(heron_ui::BTPanel, rviz::Panel)
