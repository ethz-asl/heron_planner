#ifndef BT_PANEL_H
#define BT_PANEL_H

#include <ros/ros.h>
#include <rviz/panel.h>
#include <QLineEdit>
#include <QPushButton>
#include <QCheckBox>

namespace bt_ui
{
  class BTPanel : public rviz::Panel
  {
    Q_OBJECT
  public:
    BTPanel(QWidget *parent = nullptr);

    // Override the save and load methods to retain settings between RViz sessions
    void save(rviz::Config config) const override;
    void load(const rviz::Config &config) override;

  private Q_SLOTS:
    void onStartButtonClicked();
    void onStopButtonClicked();

  private:
    QPushButton *start_button_;
    QPushButton *stop_button_;
  };
} // end namespace bt_ui

#endif // BT_PANEL_H
