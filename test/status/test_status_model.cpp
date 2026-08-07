#include <gtest/gtest.h>

#include <mrs_uav_status/status/status_model.hpp>
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
  tui.flying_normally = true;

  tick(sm, tui, 'R');

  EXPECT_EQ(sm.state(), StatusState::REMOTE);
  EXPECT_EQ(tui.remote_mode_calls, 1);
  EXPECT_TRUE(tui.remote_mode_active);
}

TEST(StatusModel, DoesNotEnterRemoteModeWhenNotFlyingNormally) {
  StatusModel sm;
  FakeTui     tui;
  tui.flying_normally = false;

  tick(sm, tui, 'R');

  EXPECT_EQ(sm.state(), StatusState::STANDARD);
  EXPECT_EQ(tui.remote_mode_calls, 0);
}

TEST(StatusModel, ExitsRemoteModeOnSecondR) {
  StatusModel sm;
  FakeTui     tui;

  tick(sm, tui, 'R');
  ASSERT_EQ(sm.state(), StatusState::REMOTE);

  tick(sm, tui, 'R');

  EXPECT_EQ(sm.state(), StatusState::STANDARD);
  EXPECT_FALSE(tui.remote_mode_active);
}

TEST(StatusModel, ExitsRemoteModeOnEscape) {
  StatusModel sm;
  FakeTui     tui;

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

} // namespace mrs_uav_status::status
