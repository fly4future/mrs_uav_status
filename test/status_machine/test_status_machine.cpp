#include <gtest/gtest.h>

#include <mrs_uav_status/status/status_machine.hpp>
#include <mrs_uav_status/tui/constants.hpp>

namespace mrs_uav_status::status
{

namespace
{

class FakeTui : public tui::TuiActions {
public:
  bool flying_normally           = true;
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

  void setDataFreshness(bool, bool, bool, bool, bool) override {
  }
  bool isFlyingNormally() override {
    return flying_normally;
  }
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

TickInput keyTick(int key) {
  return TickInput{key, Freshness{}, rclcpp::Time(0, 0, RCL_ROS_TIME)};
}

} // namespace

TEST(StatusMachine, StartsInStandardState) {
  StatusMachine sm;
  EXPECT_EQ(sm.state(), StatusState::STANDARD);
}

TEST(StatusMachine, EntersRemoteModeOnRWhenFlyingNormally) {
  StatusMachine sm;
  FakeTui       tui;
  tui.flying_normally = true;

  sm.handleTick(keyTick('R'), tui);

  EXPECT_EQ(sm.state(), StatusState::REMOTE);
  EXPECT_EQ(tui.remote_mode_calls, 1);
  EXPECT_TRUE(tui.remote_mode_active);
}

TEST(StatusMachine, DoesNotEnterRemoteModeWhenNotFlyingNormally) {
  StatusMachine sm;
  FakeTui       tui;
  tui.flying_normally = false;

  sm.handleTick(keyTick('R'), tui);

  EXPECT_EQ(sm.state(), StatusState::STANDARD);
  EXPECT_EQ(tui.remote_mode_calls, 0);
}

TEST(StatusMachine, ExitsRemoteModeOnSecondR) {
  StatusMachine sm;
  FakeTui       tui;

  sm.handleTick(keyTick('R'), tui);
  ASSERT_EQ(sm.state(), StatusState::REMOTE);

  sm.handleTick(keyTick('R'), tui);

  EXPECT_EQ(sm.state(), StatusState::STANDARD);
  EXPECT_FALSE(tui.remote_mode_active);
}

TEST(StatusMachine, MainMenuOpensAndClosesOnHandlerReturningTrue) {
  StatusMachine sm;
  FakeTui       tui;

  sm.handleTick(keyTick('m'), tui);
  ASSERT_EQ(sm.state(), StatusState::MAIN_MENU);
  EXPECT_EQ(tui.setup_main_menu_calls, 1);

  tui.main_menu_should_close = true;
  sm.handleTick(keyTick(static_cast<int>(mrs_uav_status::tui::Key::Enter)), tui);

  EXPECT_EQ(sm.state(), StatusState::STANDARD);
  EXPECT_EQ(tui.clear_menus_calls, 1);
  EXPECT_EQ(tui.refresh_after_menu_calls, 1);
}

TEST(StatusMachine, GotoMenuOpensAndClosesOnHandlerReturningTrue) {
  StatusMachine sm;
  FakeTui       tui;

  sm.handleTick(keyTick('g'), tui);
  ASSERT_EQ(sm.state(), StatusState::GOTO_MENU);
  EXPECT_EQ(tui.setup_goto_menu_calls, 1);

  tui.goto_menu_should_close = true;
  sm.handleTick(keyTick(static_cast<int>(mrs_uav_status::tui::Key::Enter)), tui);

  EXPECT_EQ(sm.state(), StatusState::STANDARD);
  EXPECT_EQ(tui.clear_menus_calls, 1);
}

TEST(StatusMachine, DisplayMenuOpensAndClosesOnHandlerReturningTrue) {
  StatusMachine sm;
  FakeTui       tui;

  sm.handleTick(keyTick('D'), tui);
  ASSERT_EQ(sm.state(), StatusState::DISPLAY_MENU);
  EXPECT_EQ(tui.setup_display_menu_calls, 1);

  tui.display_menu_should_close = true;
  sm.handleTick(keyTick(static_cast<int>(mrs_uav_status::tui::Key::Enter)), tui);

  EXPECT_EQ(sm.state(), StatusState::STANDARD);
  EXPECT_EQ(tui.clear_menus_calls, 1);
}

TEST(StatusMachine, UnrecognizedKeyInStandardFlushesInput) {
  StatusMachine sm;
  FakeTui       tui;

  sm.handleTick(keyTick('z'), tui);

  EXPECT_EQ(sm.state(), StatusState::STANDARD);
  EXPECT_EQ(tui.flush_input_calls, 1);
}

TEST(StatusMachine, NumberKeySelectsPaneByZeroBasedIndex) {
  StatusMachine sm;
  FakeTui       tui;

  sm.handleTick(keyTick('3'), tui);

  EXPECT_EQ(tui.select_pane_last_idx, 2);
}

TEST(StatusMachine, RefreshesBottomWindowOnlyOutsideMenuStates) {
  StatusMachine sm;
  FakeTui       tui;

  sm.handleTick(keyTick('h'), tui); // STANDARD -> STANDARD
  EXPECT_EQ(tui.refresh_bottom_window_calls, 1);

  sm.handleTick(keyTick('m'), tui);              // STANDARD -> MAIN_MENU
  EXPECT_EQ(tui.refresh_bottom_window_calls, 1); // unchanged: now in a menu state
}

} // namespace mrs_uav_status::status
