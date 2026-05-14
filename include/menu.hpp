#include <commons.hpp>

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
    } action = Action::None;

    int selected_line = -1;
    int pressed_key   = -1;
  };

  Result iterate(std::vector<std::string> &text, int key, bool refresh);
  Result iterate(int key, bool refresh);

private:
  WINDOW                  *win_;
  int                      line_ = 0;
  int                      id_;
  int                      y_;
  int                      x_;
  int                      rows_;
  int                      cols_;
  std::vector<std::string> text_;
};
