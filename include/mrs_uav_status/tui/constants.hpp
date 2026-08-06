#pragma once

namespace mrs_uav_status::tui
{

// --- Keyboard Input Constants ---
// Represents specific key codes used for terminal navigation.
enum class Key : int
{
  Enter  = 10,
  Escape = 27,
  Delete = 330
};

// --- Terminal UI Color Pairs ---
// These refer to the logical pairs initialized via init_pair() in ncurses.
enum class ColorPair : int
{
  Normal = 100,
  Field  = 101,
  Green  = 102,
  Red    = 103,
  Yellow = 104
};

// --- Background Colors ---
// Useful for specifying transparency.
enum class BackgroundColor : int
{
  Default = -1
};

// --- Extended Terminal Colors ---
// Standard xterm-256 color codes.
enum class Color : int
{
  NiceRed    = 196,
  NiceGreen  = 82,
  DarkGreen  = 2,
  NiceBlue   = 33,
  DarkBlue   = 19,
  NiceYellow = 220,
  DarkYellow = 172
};

} // namespace mrs_uav_status::tui
