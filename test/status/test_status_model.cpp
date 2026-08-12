#include <gtest/gtest.h>

#include <algorithm>

#include <mrs_uav_status/status/status_model.hpp>
#include <mrs_uav_status/tui/constants.hpp>

namespace mrs_uav_status::status
{

namespace
{

// | ----------------------- Test doubles ---------------------- |

class FakeTui : public tui::TuiActions {
public:
  // programmable responses
  tui::MenuEvent next_menu_event;
  tui::GotoEvent next_goto_event;
  bool           display_menu_should_close = false;

  // recorded calls
  std::vector<std::string> main_menu_labels;
  std::vector<std::string> sub_menu_labels;
  std::vector<std::string> goto_menu_labels;
  std::vector<double>      goto_initial_values;
  std::vector<std::string> service_results;

  int  show_main_menu_calls        = 0;
  int  show_sub_menu_calls         = 0;
  int  close_sub_menu_calls        = 0;
  int  show_goto_menu_calls        = 0;
  int  setup_display_menu_calls    = 0;
  int  clear_menus_calls           = 0;
  int  refresh_after_menu_calls    = 0;
  int  refresh_bottom_window_calls = 0;
  int  toggle_help_calls           = 0;
  int  cycle_panes_calls           = 0;
  int  select_pane_last_idx        = -1;
  int  toggle_mini_calls           = 0;
  int  flush_input_calls           = 0;
  int  remote_banner_calls         = 0;
  bool remote_banner_turbo         = false;
  bool remote_banner_global        = false;
  bool remote_mode_active          = false;

  void showMainMenu(const std::vector<std::string> &labels) override {
    main_menu_labels = labels;
    show_main_menu_calls++;
  }
  void showSubMenu(const std::vector<std::string> &labels) override {
    sub_menu_labels = labels;
    show_sub_menu_calls++;
  }
  void closeSubMenu() override {
    close_sub_menu_calls++;
  }
  tui::MenuEvent handleMainMenuKey(int) override {
    return next_menu_event;
  }
  void showGotoMenu(const std::vector<std::string> &labels, const std::vector<double> &initial_values) override {
    goto_menu_labels    = labels;
    goto_initial_values = initial_values;
    show_goto_menu_calls++;
  }
  tui::GotoEvent handleGotoMenuKey(int) override {
    return next_goto_event;
  }
  void setupDisplayMenu() override {
    setup_display_menu_calls++;
  }
  bool displayMenuHandler(int) override {
    return display_menu_should_close;
  }
  void clearMenus() override {
    clear_menus_calls++;
  }
  void refreshAfterMenu() override {
    refresh_after_menu_calls++;
  }
  void setRemoteMode(bool in_remote_mode) override {
    remote_mode_active = in_remote_mode;
  }
  void renderRemoteBanner(bool turbo, bool global) override {
    remote_banner_turbo  = turbo;
    remote_banner_global = global;
    remote_banner_calls++;
  }
  // NOTE: TuiActions::renderServiceResult gained a third parameter (`now_seconds`, the
  // post-service-call wall clock used to stamp the bottom-window clear timer) after this test's
  // literal brief text was written. Overriding a pure virtual with a mismatched signature is a
  // compile error, so the third parameter is added here and ignored -- no test below inspects it.
  void renderServiceResult(bool success, const std::string &message, double /*now_seconds*/) override {
    service_results.push_back((success ? "OK: " : "FAIL: ") + message);
  }
  void toggleHelp() override {
    toggle_help_calls++;
  }
  void cyclePanes() override {
    cycle_panes_calls++;
  }
  void selectPane(std::size_t idx) override {
    select_pane_last_idx = static_cast<int>(idx);
  }
  void toggleMini() override {
    toggle_mini_calls++;
  }
  void setupWindows() override {
  }
  void renderFast() override {
  }
  void renderSlow() override {
  }
  void flushInput() override {
    flush_input_calls++;
  }
  void refreshBottomWindow() override {
    refresh_bottom_window_calls++;
  }
};

// Records every outbound service call the model makes.
struct SinkLog
{
  std::vector<std::string> calls;

  double      vx = 0.0, vy = 0.0, vz = 0.0, vhdg = 0.0;
  std::string velocity_frame;

  double      goto_x = 0.0, goto_y = 0.0, goto_z = 0.0, goto_hdg = 0.0;
  std::string goto_frame;
};

CommandSink makeSink(SinkLog &log) {
  CommandSink sink;

  sink.sendGoto = [&log](double x, double y, double z, double heading, const std::string &frame_id) {
    log.goto_x     = x;
    log.goto_y     = y;
    log.goto_z     = z;
    log.goto_hdg   = heading;
    log.goto_frame = frame_id;
    log.calls.push_back("sendGoto");
    return CommandSink::ServiceResult{true, "goto ok"};
  };

  sink.sendVelocityReference = [&log](double vx, double vy, double vz, double heading_rate, const std::string &frame_id) {
    log.vx             = vx;
    log.vy             = vy;
    log.vz             = vz;
    log.vhdg           = heading_rate;
    log.velocity_frame = frame_id;
    log.calls.push_back("sendVelocityReference");
  };

  auto recorder = [&log](const std::string &name) {
    return [&log, name](const std::string &value) {
      log.calls.push_back(name + "(" + value + ")");
      return CommandSink::ServiceResult{true, value};
    };
  };
  sink.setConstraints = recorder("setConstraints");
  sink.setGains       = recorder("setGains");
  sink.setController  = recorder("setController");
  sink.setTracker     = recorder("setTracker");
  sink.setEstimator   = recorder("setEstimator");

  sink.hover = [&log]() {
    log.calls.push_back("hover");
    return CommandSink::ServiceResult{true, "hovering"};
  };
  sink.toggleOutput = [&log]() {
    log.calls.push_back("toggleOutput");
    return CommandSink::ServiceResult{true, "toggled"};
  };

  sink.extra_services.push_back({"Land", [&log]() {
                                   log.calls.push_back("Land");
                                   return CommandSink::ServiceResult{true, "landing"};
                                 }});
  sink.extra_services.push_back({"Takeoff", [&log]() {
                                   log.calls.push_back("Takeoff");
                                   return CommandSink::ServiceResult{true, "taking off"};
                                 }});

  // NOTE: CommandSink gained a `nowSeconds` member (used to stamp renderServiceResult()'s
  // post-call clear-timer, see status_model.cpp) after this test's literal brief text was
  // written. It is called unconditionally at every service-result call site, so leaving it
  // default-constructed would throw std::bad_function_call the first time a submenu/turbo action
  // runs; wired to a fixed value since no test below inspects it.
  sink.nowSeconds = []() { return 0.0; };

  return sink;
}

// Mirrors config/public/default.yaml.
StatusModel::Params defaultParams() {
  return StatusModel::Params{.turbo_remote_constraints = "fast", .goto_values = {0.0, 0.0, 2.0, 1.57}};
}

// A ControlInfo that reads as "airborne, under MpcTracker, on the 'medium' constraint set".
ControlInfoData flyingControlInfo() {
  ControlInfoData ci;
  ci.flying_normally       = true;
  ci.active_tracker        = "MpcTracker";
  ci.available_trackers    = {"MpcTracker", "LandoffTracker"};
  ci.active_constraints    = "medium";
  ci.available_constraints = {"slow", "medium", "fast"};
  ci.active_gains          = "supersoft";
  ci.available_gains       = {"supersoft", "soft"};
  ci.active_controller     = "Se3Controller";
  ci.available_controllers = {"Se3Controller", "MpcController"};
  return ci;
}

void tick(StatusModel &model, FakeTui &tui, int key, double now_seconds = 0.0) {
  model.setFreshness(Freshness{});
  model.tick(now_seconds, key, tui);
}

int indexOf(const std::vector<std::string> &labels, const std::string &label) {
  const auto it = std::find(labels.begin(), labels.end(), label);
  return (it == labels.end()) ? -1 : static_cast<int>(std::distance(labels.begin(), it));
}

tui::MenuEvent selectMain(int index) {
  return tui::MenuEvent{tui::MenuEvent::Kind::Selected, false, index};
}

tui::MenuEvent selectSub(int index) {
  return tui::MenuEvent{tui::MenuEvent::Kind::Selected, true, index};
}

} // namespace

// | --------------------- Top-level FSM ---------------------- |

TEST(StatusModel, StartsInStandardState) {
  SinkLog     log;
  StatusModel sm(makeSink(log), defaultParams());
  EXPECT_EQ(sm.state(), StatusState::STANDARD);
}

TEST(StatusModel, UnrecognizedKeyInStandardFlushesInput) {
  SinkLog     log;
  StatusModel sm(makeSink(log), defaultParams());
  FakeTui     tui;

  tick(sm, tui, 'z');

  EXPECT_EQ(sm.state(), StatusState::STANDARD);
  EXPECT_EQ(tui.flush_input_calls, 1);
}

TEST(StatusModel, NumberKeySelectsPaneByZeroBasedIndex) {
  SinkLog     log;
  StatusModel sm(makeSink(log), defaultParams());
  FakeTui     tui;

  tick(sm, tui, '3');

  EXPECT_EQ(tui.select_pane_last_idx, 2);
}

TEST(StatusModel, RefreshesBottomWindowOnlyOutsideMenuStates) {
  SinkLog     log;
  StatusModel sm(makeSink(log), defaultParams());
  FakeTui     tui;

  tick(sm, tui, 'h'); // STANDARD -> STANDARD
  EXPECT_EQ(tui.refresh_bottom_window_calls, 1);
  EXPECT_EQ(tui.toggle_help_calls, 1);

  tick(sm, tui, 'm');                            // STANDARD -> MAIN_MENU
  EXPECT_EQ(tui.refresh_bottom_window_calls, 1); // unchanged: now in a menu state
}

TEST(StatusModel, DisplayMenuOpensAndClosesOnHandlerReturningTrue) {
  SinkLog     log;
  StatusModel sm(makeSink(log), defaultParams());
  FakeTui     tui;

  tick(sm, tui, 'D');
  ASSERT_EQ(sm.state(), StatusState::DISPLAY_MENU);
  EXPECT_EQ(tui.setup_display_menu_calls, 1);

  tui.display_menu_should_close = true;
  tick(sm, tui, static_cast<int>(tui::Key::Enter));

  EXPECT_EQ(sm.state(), StatusState::STANDARD);
  EXPECT_EQ(tui.clear_menus_calls, 1);
  // Unlike MAIN_MENU, DISPLAY_MENU close must not call refreshAfterMenu().
  EXPECT_EQ(tui.refresh_after_menu_calls, 0);
}

// | ------------------------ Main menu ------------------------ |

TEST(StatusModel, MainMenuHidesTakeoffWhileFlying) {
  SinkLog     log;
  StatusModel sm(makeSink(log), defaultParams());
  FakeTui     tui;

  sm.onControlInfo(flyingControlInfo());

  tick(sm, tui, 'm');

  ASSERT_EQ(sm.state(), StatusState::MAIN_MENU);
  EXPECT_EQ(tui.show_main_menu_calls, 1);
  EXPECT_NE(indexOf(tui.main_menu_labels, "Land"), -1);
  EXPECT_EQ(indexOf(tui.main_menu_labels, "Takeoff"), -1);
  EXPECT_NE(indexOf(tui.main_menu_labels, "Set Constraints"), -1);
}

TEST(StatusModel, MainMenuHidesLandUnderNullTracker) {
  SinkLog     log;
  StatusModel sm(makeSink(log), defaultParams());
  FakeTui     tui;

  ControlInfoData ci;
  ci.active_tracker = "NullTracker";
  sm.onControlInfo(ci);

  tick(sm, tui, 'm');

  EXPECT_EQ(indexOf(tui.main_menu_labels, "Land"), -1);
  EXPECT_NE(indexOf(tui.main_menu_labels, "Takeoff"), -1);
}

TEST(StatusModel, SelectingASetterRowOpensASubmenuWithTheActiveOptionFirst) {
  SinkLog     log;
  StatusModel sm(makeSink(log), defaultParams());
  FakeTui     tui;

  sm.onControlInfo(flyingControlInfo());
  tick(sm, tui, 'm');

  tui.next_menu_event = selectMain(indexOf(tui.main_menu_labels, "Set Constraints"));
  tick(sm, tui, static_cast<int>(tui::Key::Enter));

  EXPECT_EQ(sm.state(), StatusState::MAIN_MENU);
  EXPECT_EQ(tui.show_sub_menu_calls, 1);
  EXPECT_EQ(tui.sub_menu_labels, (std::vector<std::string>{"medium", "slow", "fast"}));
}

TEST(StatusModel, SubmenuLabelsAreBuiltAtSelectionTimeNotMenuBuildTime) {
  SinkLog     log;
  StatusModel sm(makeSink(log), defaultParams());
  FakeTui     tui;

  sm.onControlInfo(flyingControlInfo());
  tick(sm, tui, 'm');

  // A new ControlInfo arrives while the menu is open.
  ControlInfoData ci       = flyingControlInfo();
  ci.active_constraints    = "slow";
  ci.available_constraints = {"slow", "fast"};
  sm.onControlInfo(ci);

  tui.next_menu_event = selectMain(indexOf(tui.main_menu_labels, "Set Constraints"));
  tick(sm, tui, static_cast<int>(tui::Key::Enter));

  EXPECT_EQ(tui.sub_menu_labels, (std::vector<std::string>{"slow", "fast"}));
}

TEST(StatusModel, SubmenuSelectionCallsTheServiceAndClosesTheWholeMenu) {
  SinkLog     log;
  StatusModel sm(makeSink(log), defaultParams());
  FakeTui     tui;

  sm.onControlInfo(flyingControlInfo());
  tick(sm, tui, 'm');

  tui.next_menu_event = selectMain(indexOf(tui.main_menu_labels, "Set Constraints"));
  tick(sm, tui, static_cast<int>(tui::Key::Enter));
  ASSERT_EQ(tui.sub_menu_labels, (std::vector<std::string>{"medium", "slow", "fast"}));

  tui.next_menu_event = selectSub(2); // "fast"
  tick(sm, tui, static_cast<int>(tui::Key::Enter));

  EXPECT_EQ(sm.state(), StatusState::STANDARD);
  EXPECT_EQ(log.calls, (std::vector<std::string>{"setConstraints(fast)"}));
  EXPECT_EQ(tui.service_results, (std::vector<std::string>{"OK: fast"}));
  EXPECT_EQ(tui.close_sub_menu_calls, 1);
  EXPECT_EQ(tui.clear_menus_calls, 1);
  EXPECT_EQ(tui.refresh_after_menu_calls, 1);
}

TEST(StatusModel, SubmenuCancelRowBacksOutToTheMainMenu) {
  SinkLog     log;
  StatusModel sm(makeSink(log), defaultParams());
  FakeTui     tui;

  sm.onControlInfo(flyingControlInfo());
  tick(sm, tui, 'm');

  tui.next_menu_event = selectMain(indexOf(tui.main_menu_labels, "Land"));
  tick(sm, tui, static_cast<int>(tui::Key::Enter));
  ASSERT_EQ(tui.sub_menu_labels, (std::vector<std::string>{"CANCEL", "Land"}));

  tui.next_menu_event = selectSub(0); // "CANCEL"
  tick(sm, tui, static_cast<int>(tui::Key::Enter));

  EXPECT_EQ(sm.state(), StatusState::MAIN_MENU);
  EXPECT_TRUE(log.calls.empty());
  EXPECT_EQ(tui.close_sub_menu_calls, 1);

  tui.next_menu_event = selectSub(1); // "Land"
  tick(sm, tui, static_cast<int>(tui::Key::Enter));

  EXPECT_EQ(sm.state(), StatusState::STANDARD);
  EXPECT_EQ(log.calls, (std::vector<std::string>{"Land"}));
}

TEST(StatusModel, EscapeInASubmenuClosesOnlyTheSubmenu) {
  SinkLog     log;
  StatusModel sm(makeSink(log), defaultParams());
  FakeTui     tui;

  sm.onControlInfo(flyingControlInfo());
  tick(sm, tui, 'm');

  tui.next_menu_event = selectMain(indexOf(tui.main_menu_labels, "Set Gains"));
  tick(sm, tui, static_cast<int>(tui::Key::Enter));

  tui.next_menu_event = tui::MenuEvent{tui::MenuEvent::Kind::Exit, true, -1};
  tick(sm, tui, static_cast<int>(tui::Key::Escape));

  EXPECT_EQ(sm.state(), StatusState::MAIN_MENU);
  EXPECT_EQ(tui.close_sub_menu_calls, 1);
  EXPECT_EQ(tui.clear_menus_calls, 0);
}

TEST(StatusModel, EscapeInTheMainMenuClosesEverything) {
  SinkLog     log;
  StatusModel sm(makeSink(log), defaultParams());
  FakeTui     tui;

  sm.onControlInfo(flyingControlInfo());
  tick(sm, tui, 'm');
  ASSERT_EQ(sm.state(), StatusState::MAIN_MENU);

  tui.next_menu_event = tui::MenuEvent{tui::MenuEvent::Kind::Exit, false, -1};
  tick(sm, tui, static_cast<int>(tui::Key::Escape));

  EXPECT_EQ(sm.state(), StatusState::STANDARD);
  EXPECT_EQ(tui.clear_menus_calls, 1);
  EXPECT_EQ(tui.refresh_after_menu_calls, 1);
}

// | -------------------------- Goto --------------------------- |

TEST(StatusModel, GotoMenuSeedsFromParamsAndLabelsTheCurrentFrame) {
  SinkLog     log;
  StatusModel sm(makeSink(log), defaultParams());
  FakeTui     tui;

  StateEstimationInfoData est;
  est.frame_id = "uav1/world_origin";
  sm.onStateEstimationInfo(est);

  tick(sm, tui, 'g');

  ASSERT_EQ(sm.state(), StatusState::GOTO_MENU);
  EXPECT_EQ(tui.show_goto_menu_calls, 1);
  ASSERT_EQ(tui.goto_menu_labels.size(), 5u);
  EXPECT_EQ(tui.goto_menu_labels[4], " uav1/world_origin ");
  EXPECT_EQ(tui.goto_initial_values, (std::vector<double>{0.0, 0.0, 2.0, 1.57}));
}

TEST(StatusModel, GotoCommitSendsTheReferenceInTheCurrentFrameAndIsRemembered) {
  SinkLog     log;
  StatusModel sm(makeSink(log), defaultParams());
  FakeTui     tui;

  StateEstimationInfoData est;
  est.frame_id = "uav1/world_origin";
  sm.onStateEstimationInfo(est);

  tick(sm, tui, 'g');

  tui.next_goto_event = tui::GotoEvent{tui::GotoEvent::Kind::Committed, 1.5, -2.5, 3.0, 0.75};
  tick(sm, tui, static_cast<int>(tui::Key::Enter));

  EXPECT_EQ(sm.state(), StatusState::STANDARD);
  EXPECT_EQ(log.calls, (std::vector<std::string>{"sendGoto"}));
  EXPECT_DOUBLE_EQ(log.goto_x, 1.5);
  EXPECT_DOUBLE_EQ(log.goto_y, -2.5);
  EXPECT_DOUBLE_EQ(log.goto_z, 3.0);
  EXPECT_DOUBLE_EQ(log.goto_hdg, 0.75);
  EXPECT_EQ(log.goto_frame, "uav1/world_origin");
  EXPECT_EQ(tui.service_results, (std::vector<std::string>{"OK: goto ok"}));

  // Reopening offers what was last entered, not the config defaults.
  tui.next_goto_event = tui::GotoEvent{};
  tick(sm, tui, 'g');
  EXPECT_EQ(tui.goto_initial_values, (std::vector<double>{1.5, -2.5, 3.0, 0.75}));
}

TEST(StatusModel, GotoEscapeClosesWithoutCallingTheService) {
  SinkLog     log;
  StatusModel sm(makeSink(log), defaultParams());
  FakeTui     tui;

  tick(sm, tui, 'g');

  tui.next_goto_event = tui::GotoEvent{tui::GotoEvent::Kind::Exit, 0.0, 0.0, 0.0, 0.0};
  tick(sm, tui, static_cast<int>(tui::Key::Escape));

  EXPECT_EQ(sm.state(), StatusState::STANDARD);
  EXPECT_TRUE(log.calls.empty());
  EXPECT_EQ(tui.clear_menus_calls, 1);
  EXPECT_EQ(tui.refresh_after_menu_calls, 0);
}

// | ------------------------- Remote -------------------------- |

TEST(StatusModel, EntersRemoteModeOnRWhenFlyingNormally) {
  SinkLog     log;
  StatusModel sm(makeSink(log), defaultParams());
  FakeTui     tui;

  sm.onControlInfo(flyingControlInfo());

  tick(sm, tui, 'R');

  EXPECT_EQ(sm.state(), StatusState::REMOTE);
  EXPECT_TRUE(tui.remote_mode_active);
}

TEST(StatusModel, DoesNotEnterRemoteModeWhenNotFlyingNormally) {
  SinkLog     log;
  StatusModel sm(makeSink(log), defaultParams());
  FakeTui     tui;

  tick(sm, tui, 'R'); // flying_normally defaults to false

  EXPECT_EQ(sm.state(), StatusState::STANDARD);
  EXPECT_FALSE(tui.remote_mode_active);
}

TEST(StatusModel, ExitsRemoteModeOnSecondR) {
  SinkLog     log;
  StatusModel sm(makeSink(log), defaultParams());
  FakeTui     tui;

  sm.onControlInfo(flyingControlInfo());

  tick(sm, tui, 'R');
  ASSERT_EQ(sm.state(), StatusState::REMOTE);

  tick(sm, tui, 'R');

  EXPECT_EQ(sm.state(), StatusState::STANDARD);
  EXPECT_FALSE(tui.remote_mode_active);
}

TEST(StatusModel, ExitsRemoteModeOnEscape) {
  SinkLog     log;
  StatusModel sm(makeSink(log), defaultParams());
  FakeTui     tui;

  sm.onControlInfo(flyingControlInfo());

  tick(sm, tui, 'R');
  ASSERT_EQ(sm.state(), StatusState::REMOTE);

  tick(sm, tui, static_cast<int>(tui::Key::Escape));

  EXPECT_EQ(sm.state(), StatusState::STANDARD);
  EXPECT_FALSE(tui.remote_mode_active);
}

TEST(StatusModel, RemoteMotionSendsVelocityInTheFcuFrameAndDrawsTheBanner) {
  SinkLog     log;
  StatusModel sm(makeSink(log), defaultParams());
  FakeTui     tui;

  GeneralRobotInfoData gri;
  gri.robot_name = "uav1";
  sm.onGeneralRobotInfo(gri);
  sm.onControlInfo(flyingControlInfo());

  tick(sm, tui, 'R');
  tick(sm, tui, 'w');

  EXPECT_EQ(log.calls, (std::vector<std::string>{"sendVelocityReference"}));
  EXPECT_DOUBLE_EQ(log.vx, 2.0);
  EXPECT_DOUBLE_EQ(log.vy, 0.0);
  EXPECT_EQ(log.velocity_frame, "uav1/fcu_untilted");
  EXPECT_GE(tui.remote_banner_calls, 1);
  EXPECT_FALSE(tui.remote_banner_turbo);
  EXPECT_FALSE(tui.remote_banner_global);
}

TEST(StatusModel, ArrowKeysMapToTheSameMotionsAsWasd) {
  SinkLog     log;
  StatusModel sm(makeSink(log), defaultParams());
  FakeTui     tui;

  GeneralRobotInfoData gri;
  gri.robot_name = "uav1";
  sm.onGeneralRobotInfo(gri);
  sm.onControlInfo(flyingControlInfo());

  tick(sm, tui, 'R');
  tick(sm, tui, static_cast<int>(tui::Key::Right));

  EXPECT_DOUBLE_EQ(log.vy, -2.0);
}

TEST(StatusModel, GlobalToggleSwitchesTheVelocityFrame) {
  SinkLog     log;
  StatusModel sm(makeSink(log), defaultParams());
  FakeTui     tui;

  GeneralRobotInfoData gri;
  gri.robot_name = "uav1";
  sm.onGeneralRobotInfo(gri);
  sm.onControlInfo(flyingControlInfo());

  tick(sm, tui, 'R');
  tick(sm, tui, 'G');
  tick(sm, tui, 'w');

  EXPECT_EQ(log.velocity_frame, "uav1/world_origin");
  EXPECT_TRUE(tui.remote_banner_global);
}

TEST(StatusModel, TurboSwapsConstraintsAndRestoresThePreviousSet) {
  SinkLog     log;
  StatusModel sm(makeSink(log), defaultParams());
  FakeTui     tui;

  sm.onControlInfo(flyingControlInfo()); // active_constraints == "medium"

  tick(sm, tui, 'R');
  tick(sm, tui, 'T');

  EXPECT_EQ(log.calls, (std::vector<std::string>{"setConstraints(fast)"}));

  // renderRemoteBanner() runs at the top of remoteHandler(), before the key is applied, so the
  // new turbo state only shows on the following tick (matches the real render loop, and predates
  // this refactor -- see TUI::remoteHandler()/drawRemoteBanner() before the logic moved here).
  tick(sm, tui, 'w');
  EXPECT_DOUBLE_EQ(log.vx, 5.0); // turbo steps are larger
  EXPECT_TRUE(tui.remote_banner_turbo);

  tick(sm, tui, 'T');
  EXPECT_EQ(log.calls.back(), "setConstraints(medium)");

  tick(sm, tui, 'w');
  EXPECT_DOUBLE_EQ(log.vx, 2.0); // back to the non-turbo step
  EXPECT_FALSE(tui.remote_banner_turbo);
}

TEST(StatusModel, UnmappedRemoteKeyHoversOnceAfterAMotionCommand) {
  SinkLog     log;
  StatusModel sm(makeSink(log), defaultParams());
  FakeTui     tui;

  GeneralRobotInfoData gri;
  gri.robot_name = "uav1";
  sm.onGeneralRobotInfo(gri);
  sm.onControlInfo(flyingControlInfo());

  tick(sm, tui, 'R');
  tick(sm, tui, 'z'); // nothing flown yet -> no hover
  EXPECT_TRUE(log.calls.empty());

  tick(sm, tui, 'w');
  tick(sm, tui, 'z');
  EXPECT_EQ(log.calls, (std::vector<std::string>{"sendVelocityReference", "hover"}));

  tick(sm, tui, 'z'); // hover is not repeated until something is flown again
  EXPECT_EQ(log.calls.size(), 2u);
}

// | --------------------- Snapshot / strings ------------------ |

TEST(StatusModel, SnapshotCarriesLatestDataFreshnessAndBorderStatus) {
  SinkLog     log;
  StatusModel sm(makeSink(log), defaultParams());

  GeneralRobotInfoData gri;
  gri.robot_name     = "uav1";
  gri.ready_to_start = true;
  sm.onGeneralRobotInfo(gri);

  ControlInfoData ci;
  ci.active_tracker = "NullTracker";
  sm.onControlInfo(ci);

  CollisionAvoidanceInfoData cai;
  cai.bumper_active = true;
  sm.onCollisionAvoidanceInfo(cai);

  Freshness freshness;
  freshness.general_robot_info = true;
  sm.setFreshness(freshness);

  const RenderSnapshot snap = sm.snapshot(12.5);

  EXPECT_DOUBLE_EQ(snap.now_seconds, 12.5);
  EXPECT_EQ(snap.general_robot_info.robot_name, "uav1");
  EXPECT_TRUE(snap.freshness.general_robot_info);
  EXPECT_FALSE(snap.freshness.uav_info);
  EXPECT_TRUE(snap.border_status.null_tracker);
  EXPECT_TRUE(snap.border_status.can_takeoff);
  EXPECT_TRUE(snap.border_status.bumper_active);
}

TEST(StatusModel, DisplayStringIsStoredWithoutItsFlagPreamble) {
  SinkLog     log;
  StatusModel sm(makeSink(log), defaultParams());

  sm.onString(1.0, "-id gps -p hello world");

  const RenderSnapshot snap = sm.snapshot(1.0);
  ASSERT_EQ(snap.display_strings.size(), 1u);
  EXPECT_EQ(snap.display_strings[0], "hello world");
}

TEST(StatusModel, DisplayStringIsDedupedById) {
  SinkLog     log;
  StatusModel sm(makeSink(log), defaultParams());

  sm.onString(1.0, "-id gps first");
  sm.onString(2.0, "-id gps second");

  const RenderSnapshot snap = sm.snapshot(2.0);
  ASSERT_EQ(snap.display_strings.size(), 1u);
  EXPECT_EQ(snap.display_strings[0], "second");
}

TEST(StatusModel, NonPersistentDisplayStringExpiresAfterTenSeconds) {
  SinkLog     log;
  StatusModel sm(makeSink(log), defaultParams());
  FakeTui     tui;

  sm.onString(1.0, "-id gps transient");
  ASSERT_EQ(sm.snapshot(1.0).display_strings.size(), 1u);

  tick(sm, tui, -1, 12.0); // pruning runs at the top of tick()

  EXPECT_TRUE(sm.snapshot(12.0).display_strings.empty());
}

TEST(StatusModel, PersistentDisplayStringSurvivesExpiry) {
  SinkLog     log;
  StatusModel sm(makeSink(log), defaultParams());
  FakeTui     tui;

  sm.onString(1.0, "-id gps -p forever");

  tick(sm, tui, -1, 120.0);

  ASSERT_EQ(sm.snapshot(120.0).display_strings.size(), 1u);
  EXPECT_EQ(sm.snapshot(120.0).display_strings[0], "forever");
}

} // namespace mrs_uav_status::status
