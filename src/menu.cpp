#include <menu.hpp>

/* Menu() //{ */

Menu::Menu(int begin_y, int begin_x, std::vector<std::string> &text) {
  unsigned long longest_string = 0;

  for (unsigned long line = 0; line < text.size(); line++) {
    if (text[line].length() > longest_string) {
      longest_string = text[line].length();
    }
  }

  id_   = 0;
  text_ = text;
  win_  = newwin(text_.size() + 2, longest_string + 2, begin_y, begin_x);
}

Menu::Menu(int begin_y, int begin_x, std::vector<std::string> &text, int id) {

  unsigned long longest_string = 0;

  for (unsigned long line = 0; line < text.size(); line++) {
    if (text[line].length() > longest_string) {
      longest_string = text[line].length();
    }
  }
  id_   = id;
  text_ = text;
  win_  = newwin(text_.size() + 2, longest_string + 2, begin_y, begin_x);
}

//}

/* getWin() //{ */

WINDOW *Menu::getWin() {
  return win_;
}

//}

/* getId() //{ */

int Menu::getId() {
  return id_;
}

//}

/* getLine() //{ */

int Menu::getLine() {
  return line_;
}

//}

//}

/* iterate() //{ */

Menu::Result Menu::iterate(std::vector<std::string> &text, int key, bool refresh) {

  Result result;

  wattron(win_, A_BOLD);

  if (key == 'q' || key == KEY_ESC) {
    result.action = Result::Action::Exit;
    wattroff(win_, A_BOLD);
    return result;
  }

  wattron(win_, COLOR_PAIR(GREEN));
  box(win_, 0, 0);
  wattroff(win_, COLOR_PAIR(GREEN));

  for (unsigned long j = 0; j < text.size(); j++) {
    mvwaddstr(win_, j + 1, 1, text[j].c_str());
  }

  if (key == KEY_UP || key == 'k') {
    line_--;
    line_ = (line_ < 0) ? text.size() - 1 : line_;
  } else if (key == KEY_DOWN || key == 'j') {
    line_++;
    line_ = (line_ > int(text.size() - 1)) ? 0 : line_;
  } else {
    result.pressed_key   = key;
    result.selected_line = line_;
  }

  wattron(win_, A_STANDOUT);
  mvwaddstr(win_, line_ + 1, 1, text[line_].c_str());
  wattroff(win_, A_STANDOUT);

  if (refresh) {
    wrefresh(win_);
  }

  wattroff(win_, A_BOLD);
  return result;
}

/* iterate() //{ */

Menu::Result Menu::iterate(int key, bool refresh) {
  return iterate(text_, key, refresh);
}
//}
