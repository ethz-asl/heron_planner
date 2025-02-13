#ifndef BT_PANEL_H
#define BT_PANEL_H

#include <ros/ros.h>
#include <rviz/panel.h>

#include <QCheckBox>
#include <QComboBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>

namespace heron_ui {
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
  void onChooseSequenceButtonClicked();

 private:
  void loadPotholeSequences();  // loads ros params

  QPushButton* start_button_;
  QPushButton* pause_button_;
  QPushButton* stop_button_;
  QPushButton* choose_sequence_button_;

  QLabel* sequence_label_;
  QComboBox* sequence_dropdown_;

  bool is_started_{false};
  bool is_paused_{false};
  bool is_stopped_{false};
};
}  // end namespace heron_ui

#endif  // BT_PANEL_H
