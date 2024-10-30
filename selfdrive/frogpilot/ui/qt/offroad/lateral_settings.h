#pragma once

#include <set>

#include "selfdrive/frogpilot/ui/qt/offroad/frogpilot_settings.h"

class FrogPilotLateralPanel : public FrogPilotListWidget {
  Q_OBJECT

public:
  explicit FrogPilotLateralPanel(FrogPilotSettingsWindow *parent);

signals:
  void openParentToggle();

private:
  void hideToggles();
  void showToggles(const std::set<QString> &keys);
  void updateMetric();
  void updateCarToggles();
  void updateState(const UIState &s);

  std::set<QString> aolKeys = {
    "AlwaysOnLateralLKAS", "AlwaysOnLateralMain",
    "HideAOLStatusBar", "PauseAOLOnBrake"
  };

  std::set<QString> laneChangeKeys = {
    "LaneChangeTime", "LaneDetectionWidth",
    "MinimumLaneChangeSpeed", "NudgelessLaneChange",
    "OneLaneChange"
  };

  std::set<QString> lateralTuneKeys = {
    "NNFF", "NNFFLite"
  };

  std::set<QString> qolKeys = {
    "PauseLateralSpeed"
  };

  FrogPilotSettingsWindow *parent;

  Params params;

  bool hasAutoTune;
  bool hasNNFFLog;
  bool isMetric = params.getBool("IsMetric");
  bool isSubaru;
  bool started;

  std::map<QString, AbstractControl*> toggles;
};
