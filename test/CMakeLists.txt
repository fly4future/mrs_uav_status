find_package(ament_cmake_clang_format REQUIRED)
ament_clang_format(CONFIG_FILE "${PROJECT_SOURCE_DIR}/.clang-format")

find_package(ament_cmake_xmllint REQUIRED)
ament_xmllint()
