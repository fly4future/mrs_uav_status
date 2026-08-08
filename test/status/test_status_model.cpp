#include <gtest/gtest.h>

#include <mrs_uav_status/status/status_model.hpp>
#include <mrs_uav_status/tui/constants.hpp>

namespace mrs_uav_status::status
{

namespace
{

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
  void renderServiceResult(bool success, const std::string &message, double) override {
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

tui::CommandSink emptySink() {
  tui::CommandSink sink;
  sink.sendGoto              = [](double, double, double, double, const std::string &) { return tui::CommandSink::ServiceResult{true, ""}; };
  sink.sendVelocityReference = [](double, double, double, double, const std::string &) {};
  sink.setConstraints        = [](const std::string &) { return tui::CommandSink::ServiceResult{true, ""}; };
  sink.setGains              = [](const std::string &) { return tui::CommandSink::ServiceResult{true, ""}; };
  sink.setController         = [](const std::string &) { return tui::CommandSink::ServiceResult{true, ""}; };
  sink.setTracker            = [](const std::string &) { return tui::CommandSink::ServiceResult{true, ""}; };
  sink.setEstimator          = [](const std::string &) { return tui::CommandSink::ServiceResult{true, ""}; };
  sink.hover                 = []() { return tui::CommandSink::ServiceResult{true, ""}; };
  sink.toggleOutput          = []() { return tui::CommandSink::ServiceResult{true, ""}; };
  sink.nowSeconds            = []() { return 0.0; };
  return sink;
}

StatusModel::Params defaultParams() {
  return StatusModel::Params{.turbo_remote_constraints = "fast", .goto_values = {0.0, 0.0, 2.0, 1.57}};
}

void tick(StatusModel &model, FakeTui &tui, int key) {
  model.setFreshness(Freshness{});
  model.tick(0.0, key, tui);
}

} // namespace

TEST(StatusModel, StartsInStandardState) {
  StatusModel sm(emptySink(), defaultParams());
  EXPECT_EQ(sm.state(), StatusState::STANDARD);
}

TEST(StatusModel, EntersRemoteModeOnRWhenFlyingNormally) {
  StatusModel sm(emptySink(), defaultParams());
  FakeTui     tui;

  ControlInfoData ci;
  ci.flying_normally = true;
  sm.onControlInfo(ci);

  tick(sm, tui, 'R');

  EXPECT_EQ(sm.state(), StatusState::REMOTE);
  EXPECT_TRUE(tui.remote_mode_active);
}

TEST(StatusModel, DoesNotEnterRemoteModeWhenNotFlyingNormally) {
  StatusModel sm(emptySink(), defaultParams());
  FakeTui     tui;

  tick(sm, tui, 'R'); // flying_normally defaults to false

  EXPECT_EQ(sm.state(), StatusState::STANDARD);
}

TEST(StatusModel, ExitsRemoteModeOnSecondR) {
  StatusModel sm(emptySink(), defaultParams());
  FakeTui     tui;

  ControlInfoData ci;
  ci.flying_normally = true;
  sm.onControlInfo(ci);

  tick(sm, tui, 'R');
  ASSERT_EQ(sm.state(), StatusState::REMOTE);

  tick(sm, tui, 'R');

  EXPECT_EQ(sm.state(), StatusState::STANDARD);
  EXPECT_FALSE(tui.remote_mode_active);
}

TEST(StatusModel, ExitsRemoteModeOnEscape) {
  StatusModel sm(emptySink(), defaultParams());
  FakeTui     tui;

  ControlInfoData ci;
  ci.flying_normally = true;
  sm.onControlInfo(ci);

  tick(sm, tui, 'R');
  ASSERT_EQ(sm.state(), StatusState::REMOTE);

  tick(sm, tui, static_cast<int>(mrs_uav_status::tui::Key::Escape));

  EXPECT_EQ(sm.state(), StatusState::STANDARD);
  EXPECT_FALSE(tui.remote_mode_active);
}

TEST(StatusModel, MainMenuOpensAndClosesOnHandlerReturningTrue) {
  StatusModel sm(emptySink(), defaultParams());
  FakeTui     tui;

  tick(sm, tui, 'm');
  ASSERT_EQ(sm.state(), StatusState::MAIN_MENU);
  EXPECT_EQ(tui.show_main_menu_calls, 1);

  tui.next_menu_event = tui::MenuEvent{tui::MenuEvent::Kind::Exit, false, -1};
  tick(sm, tui, static_cast<int>(mrs_uav_status::tui::Key::Enter));

  EXPECT_EQ(sm.state(), StatusState::STANDARD);
  EXPECT_EQ(tui.clear_menus_calls, 1);
  EXPECT_EQ(tui.refresh_after_menu_calls, 1);
}

TEST(StatusModel, GotoMenuOpensAndClosesOnHandlerReturningTrue) {
  StatusModel sm(emptySink(), defaultParams());
  FakeTui     tui;

  tick(sm, tui, 'g');
  ASSERT_EQ(sm.state(), StatusState::GOTO_MENU);
  EXPECT_EQ(tui.show_goto_menu_calls, 1);

  tui.next_goto_event = tui::GotoEvent{tui::GotoEvent::Kind::Exit, 0.0, 0.0, 0.0, 0.0};
  tick(sm, tui, static_cast<int>(mrs_uav_status::tui::Key::Enter));

  EXPECT_EQ(sm.state(), StatusState::STANDARD);
  EXPECT_EQ(tui.clear_menus_calls, 1);
  // Unlike MAIN_MENU, GOTO_MENU close must not call refreshAfterMenu().
  EXPECT_EQ(tui.refresh_after_menu_calls, 0);
}

TEST(StatusModel, DisplayMenuOpensAndClosesOnHandlerReturningTrue) {
  StatusModel sm(emptySink(), defaultParams());
  FakeTui     tui;

  tick(sm, tui, 'D');
  ASSERT_EQ(sm.state(), StatusState::DISPLAY_MENU);
  EXPECT_EQ(tui.setup_display_menu_calls, 1);

  tui.display_menu_should_close = true;
  tick(sm, tui, static_cast<int>(mrs_uav_status::tui::Key::Enter));

  EXPECT_EQ(sm.state(), StatusState::STANDARD);
  EXPECT_EQ(tui.clear_menus_calls, 1);
  // Unlike MAIN_MENU, DISPLAY_MENU close must not call refreshAfterMenu().
  EXPECT_EQ(tui.refresh_after_menu_calls, 0);
}

TEST(StatusModel, UnrecognizedKeyInStandardFlushesInput) {
  StatusModel sm(emptySink(), defaultParams());
  FakeTui     tui;

  tick(sm, tui, 'z');

  EXPECT_EQ(sm.state(), StatusState::STANDARD);
  EXPECT_EQ(tui.flush_input_calls, 1);
}

TEST(StatusModel, NumberKeySelectsPaneByZeroBasedIndex) {
  StatusModel sm(emptySink(), defaultParams());
  FakeTui     tui;

  tick(sm, tui, '3');

  EXPECT_EQ(tui.select_pane_last_idx, 2);
}

TEST(StatusModel, RefreshesBottomWindowOnlyOutsideMenuStates) {
  StatusModel sm(emptySink(), defaultParams());
  FakeTui     tui;

  tick(sm, tui, 'h'); // STANDARD -> STANDARD
  EXPECT_EQ(tui.refresh_bottom_window_calls, 1);

  tick(sm, tui, 'm');                            // STANDARD -> MAIN_MENU
  EXPECT_EQ(tui.refresh_bottom_window_calls, 1); // unchanged: now in a menu state
}

TEST(StatusModel, SnapshotCarriesLatestDataFreshnessAndBorderStatus) {
  StatusModel sm(emptySink(), defaultParams());

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
  StatusModel sm(emptySink(), defaultParams());

  sm.onString(1.0, "-id gps -p hello world");

  const RenderSnapshot snap = sm.snapshot(1.0);
  ASSERT_EQ(snap.display_strings.size(), 1u);
  EXPECT_EQ(snap.display_strings[0], "hello world");
}

TEST(StatusModel, DisplayStringIsDedupedById) {
  StatusModel sm(emptySink(), defaultParams());

  sm.onString(1.0, "-id gps first");
  sm.onString(2.0, "-id gps second");

  const RenderSnapshot snap = sm.snapshot(2.0);
  ASSERT_EQ(snap.display_strings.size(), 1u);
  EXPECT_EQ(snap.display_strings[0], "second");
}

TEST(StatusModel, NonPersistentDisplayStringExpiresAfterTenSeconds) {
  StatusModel sm(emptySink(), defaultParams());
  FakeTui     tui;

  sm.onString(1.0, "-id gps transient");
  ASSERT_EQ(sm.snapshot(1.0).display_strings.size(), 1u);

  sm.setFreshness(Freshness{});
  sm.tick(12.0, -1, tui); // pruning runs at the top of tick()

  EXPECT_TRUE(sm.snapshot(12.0).display_strings.empty());
}

TEST(StatusModel, PersistentDisplayStringSurvivesExpiry) {
  StatusModel sm(emptySink(), defaultParams());
  FakeTui     tui;

  sm.onString(1.0, "-id gps -p forever");

  sm.setFreshness(Freshness{});
  sm.tick(120.0, -1, tui);

  ASSERT_EQ(sm.snapshot(120.0).display_strings.size(), 1u);
  EXPECT_EQ(sm.snapshot(120.0).display_strings[0], "forever");
}

} // namespace mrs_uav_status::status
