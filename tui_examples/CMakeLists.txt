# TUI ncurses example (manual, interactive — not an automated test)

add_executable(MrsUavStatus_TuiExample)

target_sources(MrsUavStatus_TuiExample
  PRIVATE
    tui_example.cpp
)

target_link_libraries(MrsUavStatus_TuiExample
  PRIVATE
    mrs_lib::mrs_lib
    Curses::Curses
)

target_compile_definitions(MrsUavStatus_TuiExample PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

install(
  TARGETS MrsUavStatus_TuiExample
  DESTINATION lib/${PROJECT_NAME}
)
