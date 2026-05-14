#include <commons.h>

class Menu {

public:
  Menu(int begin_y, int begin_x, std::vector<std::string> &text);
  Menu(int begin_y, int begin_x, std::vector<std::string> &text, int id);

  WINDOW *getWin();
  int     getLine();
  int     getId();

  struct Result
  {
    enum class Action
    {
      None,
      Select,
      Exit
    } action = Action::None; // default value

    int selected_line = -1; // default value, -1 indicates no line selected
    int pressed_key   = -1; // default value, -1 indicates no key pressed
  };

  Result iterate(std::vector<std::string> &text, int key, bool refresh);
  Result iterate(int key, bool refresh);

private:
  WINDOW                  *win_;
  int                      line = 0;
  int                      id_;
  int                      y;
  int                      x;
  int                      rows;
  int                      cols;
  std::vector<std::string> text_;
};
