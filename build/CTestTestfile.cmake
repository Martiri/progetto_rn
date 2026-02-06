# CMake generated Testfile for 
# Source directory: /home/jojo/progetto_locale/gitpull/progetto_rn
# Build directory: /home/jojo/progetto_locale/gitpull/progetto_rn/build
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
if(CTEST_CONFIGURATION_TYPE MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
  add_test([=[progetto_test]=] "/home/jojo/progetto_locale/gitpull/progetto_rn/build/Debug/progetto_test")
  set_tests_properties([=[progetto_test]=] PROPERTIES  _BACKTRACE_TRIPLES "/home/jojo/progetto_locale/gitpull/progetto_rn/CMakeLists.txt;66;add_test;/home/jojo/progetto_locale/gitpull/progetto_rn/CMakeLists.txt;0;")
elseif(CTEST_CONFIGURATION_TYPE MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
  add_test([=[progetto_test]=] "/home/jojo/progetto_locale/gitpull/progetto_rn/build/Release/progetto_test")
  set_tests_properties([=[progetto_test]=] PROPERTIES  _BACKTRACE_TRIPLES "/home/jojo/progetto_locale/gitpull/progetto_rn/CMakeLists.txt;66;add_test;/home/jojo/progetto_locale/gitpull/progetto_rn/CMakeLists.txt;0;")
elseif(CTEST_CONFIGURATION_TYPE MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
  add_test([=[progetto_test]=] "/home/jojo/progetto_locale/gitpull/progetto_rn/build/RelWithDebInfo/progetto_test")
  set_tests_properties([=[progetto_test]=] PROPERTIES  _BACKTRACE_TRIPLES "/home/jojo/progetto_locale/gitpull/progetto_rn/CMakeLists.txt;66;add_test;/home/jojo/progetto_locale/gitpull/progetto_rn/CMakeLists.txt;0;")
else()
  add_test([=[progetto_test]=] NOT_AVAILABLE)
endif()
