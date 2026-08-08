#pragma once

#include <cstddef>

namespace mrs_uav_status::tui
{

// The subset of TUI's public API StatusModel drives. TUI implements this; tests substitute a
// fake instead of constructing a real (ncurses-backed) TUI.
class TuiActions {
public:
  virtual ~TuiActions() = default;

  virtual void enterRemoteMode()                  = 0;
  virtual void setRemoteMode(bool in_remote_mode) = 0;
  virtual void remoteHandler(int key)             = 0;

  virtual void setupMainMenu()             = 0;
  virtual bool mainMenuHandler(int key)    = 0;
  virtual void setupGotoMenu()             = 0;
  virtual bool gotoMenuHandler(int key)    = 0;
  virtual void setupDisplayMenu()          = 0;
  virtual bool displayMenuHandler(int key) = 0;
  virtual void clearMenus()                = 0;
  virtual void refreshAfterMenu()          = 0;

  virtual void toggleHelp()                = 0;
  virtual void cyclePanes()                = 0;
  virtual void selectPane(std::size_t idx) = 0;
  virtual void toggleMini()                = 0;
  virtual void setupWindows()              = 0;
  virtual void renderFast()                = 0;
  virtual void renderSlow()                = 0;

  virtual void flushInput()          = 0;
  virtual void refreshBottomWindow() = 0;
};

} // namespace mrs_uav_status::tui
