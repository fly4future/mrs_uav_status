#include <gtest/gtest.h>

#include <mrs_uav_status/status/status_model.hpp>
#include <mrs_uav_status/tui/constants.hpp>

namespace mrs_uav_status::status
{

namespace
{

class FakeTui : public tui::TuiActions {
public:
  bool main_menu_should_close    = false;
  bool goto_menu_should_close    = false;
  bool display_menu_should_close = false;

  int  remote_mode_calls           = 0;
  bool remote_mode_active          = false;
  int  setup_main_menu_calls       = 0;
  int  setup_goto_menu_calls       = 0;
  int  setup_display_menu_calls    = 0;
  int  clear_menus_calls           = 0;
  int  refresh_after_menu_calls    = 0;
  int  refresh_bottom_window_calls = 0;
  int  toggle_help_calls           = 0;
  int  cycle_panes_calls           = 0;
  int  select_pane_last_idx        = -1;
  int  toggle_mini_calls           = 0;
  int  flush_input_calls           = 0;

  void enterRemoteMode() override {
    remote_mode_calls++;
  }
  void setRemoteMode(bool in_remote_mode) override {
    remote_mode_active = in_remote_mode;
  }
  void remoteHandler(int) override {
  }

  void setupMainMenu() override {
    setup_main_menu_calls++;
  }
  bool mainMenuHandler(int) override {
    return main_menu_should_close;
  }
  void setupGotoMenu() override {
    setup_goto_menu_calls++;
  }
  bool gotoMenuHandler(int) override {
    return goto_menu_should_close;
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

void tick(StatusModel &model, FakeTui &tui, int key) {
  model.setFreshness(Freshness{});
  model.tick(0.0, key, tui);
}

} // namespace

TEST(StatusModel, StartsInStandardState) {
  StatusModel sm;
  EXPECT_EQ(sm.state(), StatusState::STANDARD);
}

TEST(StatusModel, EntersRemoteModeOnRWhenFlyingNormally) {
  StatusModel sm;
  FakeTui     tui;

  ControlInfoData ci;
  ci.flying_normally = true;
  sm.onControlInfo(ci);

  tick(sm, tui, 'R');

  EXPECT_EQ(sm.state(), StatusState::REMOTE);
  EXPECT_EQ(tui.remote_mode_calls, 1);
  EXPECT_TRUE(tui.remote_mode_active);
}

TEST(StatusModel, DoesNotEnterRemoteModeWhenNotFlyingNormally) {
  StatusModel sm;
  FakeTui     tui;

  tick(sm, tui, 'R'); // flying_normally defaults to false

  EXPECT_EQ(sm.state(), StatusState::STANDARD);
  EXPECT_EQ(tui.remote_mode_calls, 0);
}

TEST(StatusModel, ExitsRemoteModeOnSecondR) {
  StatusModel sm;
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
  StatusModel sm;
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
  StatusModel sm;
  FakeTui     tui;

  tick(sm, tui, 'm');
  ASSERT_EQ(sm.state(), StatusState::MAIN_MENU);
  EXPECT_EQ(tui.setup_main_menu_calls, 1);

  tui.main_menu_should_close = true;
  tick(sm, tui, static_cast<int>(mrs_uav_status::tui::Key::Enter));

  EXPECT_EQ(sm.state(), StatusState::STANDARD);
  EXPECT_EQ(tui.clear_menus_calls, 1);
  EXPECT_EQ(tui.refresh_after_menu_calls, 1);
}

TEST(StatusModel, GotoMenuOpensAndClosesOnHandlerReturningTrue) {
  StatusModel sm;
  FakeTui     tui;

  tick(sm, tui, 'g');
  ASSERT_EQ(sm.state(), StatusState::GOTO_MENU);
  EXPECT_EQ(tui.setup_goto_menu_calls, 1);

  tui.goto_menu_should_close = true;
  tick(sm, tui, static_cast<int>(mrs_uav_status::tui::Key::Enter));

  EXPECT_EQ(sm.state(), StatusState::STANDARD);
  EXPECT_EQ(tui.clear_menus_calls, 1);
  // Unlike MAIN_MENU, GOTO_MENU close must not call refreshAfterMenu().
  EXPECT_EQ(tui.refresh_after_menu_calls, 0);
}

TEST(StatusModel, DisplayMenuOpensAndClosesOnHandlerReturningTrue) {
  StatusModel sm;
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
  StatusModel sm;
  FakeTui     tui;

  tick(sm, tui, 'z');

  EXPECT_EQ(sm.state(), StatusState::STANDARD);
  EXPECT_EQ(tui.flush_input_calls, 1);
}

TEST(StatusModel, NumberKeySelectsPaneByZeroBasedIndex) {
  StatusModel sm;
  FakeTui     tui;

  tick(sm, tui, '3');

  EXPECT_EQ(tui.select_pane_last_idx, 2);
}

TEST(StatusModel, RefreshesBottomWindowOnlyOutsideMenuStates) {
  StatusModel sm;
  FakeTui     tui;

  tick(sm, tui, 'h'); // STANDARD -> STANDARD
  EXPECT_EQ(tui.refresh_bottom_window_calls, 1);

  tick(sm, tui, 'm');                            // STANDARD -> MAIN_MENU
  EXPECT_EQ(tui.refresh_bottom_window_calls, 1); // unchanged: now in a menu state
}

TEST(StatusModel, SnapshotCarriesLatestDataFreshnessAndBorderStatus) {
  StatusModel sm;

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
  StatusModel sm;

  sm.onString(1.0, "-id gps -p hello world");

  const RenderSnapshot snap = sm.snapshot(1.0);
  ASSERT_EQ(snap.display_strings.size(), 1u);
  EXPECT_EQ(snap.display_strings[0], "hello world");
}

TEST(StatusModel, DisplayStringIsDedupedById) {
  StatusModel sm;

  sm.onString(1.0, "-id gps first");
  sm.onString(2.0, "-id gps second");

  const RenderSnapshot snap = sm.snapshot(2.0);
  ASSERT_EQ(snap.display_strings.size(), 1u);
  EXPECT_EQ(snap.display_strings[0], "second");
}

TEST(StatusModel, NonPersistentDisplayStringExpiresAfterTenSeconds) {
  StatusModel sm;
  FakeTui     tui;

  sm.onString(1.0, "-id gps transient");
  ASSERT_EQ(sm.snapshot(1.0).display_strings.size(), 1u);

  sm.setFreshness(Freshness{});
  sm.tick(12.0, -1, tui); // pruning runs at the top of tick()

  EXPECT_TRUE(sm.snapshot(12.0).display_strings.empty());
}

TEST(StatusModel, PersistentDisplayStringSurvivesExpiry) {
  StatusModel sm;
  FakeTui     tui;

  sm.onString(1.0, "-id gps -p forever");

  sm.setFreshness(Freshness{});
  sm.tick(120.0, -1, tui);

  ASSERT_EQ(sm.snapshot(120.0).display_strings.size(), 1u);
  EXPECT_EQ(sm.snapshot(120.0).display_strings[0], "forever");
}

} // namespace mrs_uav_status::status
