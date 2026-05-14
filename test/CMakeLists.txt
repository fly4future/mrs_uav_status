# NCurses test

add_executable(MrsUavStatus_NcursesTest)

target_sources(MrsUavStatus_NcursesTest
  PRIVATE
    ncurses_test.cpp
)

target_link_libraries(MrsUavStatus_NcursesTest
  PRIVATE
    mrs_lib::mrs_lib
    Boost::filesystem
    Curses::Curses
)

target_compile_definitions(MrsUavStatus_NcursesTest PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

install(
  TARGETS MrsUavStatus_NcursesTest
  DESTINATION lib/${PROJECT_NAME}
)
