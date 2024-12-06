#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>

#include <vector>
#include <string>

#include <ros/names.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

#include "bt_ui/BTPanel.h"

namespace bt_ui
{
    BTPanel::BTPanel(QWidget *parent)
        : rviz::Panel(parent), start_button_(new QPushButton("Start BT")), stop_button_(new QPushButton("Stop BT"))
    {
        auto *layout = new QVBoxLayout;
        layout->addWidget(start_button_);
        layout->addWidget(stop_button_);
        setLayout(layout);

        connect(start_button_, SIGNAL(clicked()), this, SLOT(onStartButtonClicked()));
        connect(stop_button_, SIGNAL(clicked()), this, SLOT(onStopButtonClicked()));
    }

    void BTPanel::save(rviz::Config config) const
    {
        rviz::Panel::save(config); // Ensure the base class's save method is called
    }

    void BTPanel::load(const rviz::Config &config)
    {
        rviz::Panel::load(config); // Ensure the base class's load method is called
    }

    void BTPanel::onStartButtonClicked()
    {
        ros::NodeHandle nh;
        ros::ServiceClient start_bt_client = nh.serviceClient<std_srvs::Trigger>("/start_bt");
        std_srvs::Trigger srv;
        if (!start_bt_client.call(srv))
        {
            ROS_WARN("Failed to call service /start_bt");
        }
    }

    void BTPanel::onStopButtonClicked()
    {
        ros::NodeHandle nh;
        ros::ServiceClient stop_bt_client = nh.serviceClient<std_srvs::Trigger>("/stop_bt");
        std_srvs::Trigger srv;
        if (!stop_bt_client.call(srv))
        {
            ROS_WARN("Failed to call service /stop_bt");
        }
    }
} // namespace bt_ui

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(bt_ui::BTPanel, rviz::Panel)
