#ifndef BT_PANEL_H
#define BT_PANEL_H

#include <ros/ros.h>
#include <rviz/panel.h>

#include <QCheckBox>
#include <QLineEdit>
#include <QPushButton>

namespace bt_ui {
class BTPanel : public rviz::Panel {
  Q_OBJECT
 public:
  BTPanel(QWidget* parent = nullptr);

  // Override the save and load methods to retain settings between RViz sessions
  void save(rviz::Config config) const override;
  void load(const rviz::Config& config) override;

 private Q_SLOTS:
  void onStartButtonClicked();
  void onPauseButtonClicked();
  void onStopButtonClicked();

 private:
  QPushButton* start_button_;
  QPushButton* pause_button_;
  QPushButton* stop_button_;
  bool is_started_ {false};
  bool is_paused_ {false};
  bool is_stopped_ {false};
};
}  // end namespace bt_ui

#endif  // BT_PANEL_H
