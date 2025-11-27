cmake_minimum_required(VERSION 3.28)
project(progettopf VERSION 0.1.0 LANGUAGES CXX)

# supporto per i test
include(CTest)

# esporta il compilation database
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Warning flags (portabili)
string(APPEND CMAKE_CXX_FLAGS
      " -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion"
      " -Wshadow -Wimplicit-fallthrough -Wextra-semi -Wold-style-cast"
      " -fno-omit-frame-pointer")


# abilita asserzioni della standard library
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU")
  string(APPEND CMAKE_CXX_FLAGS " -D_GLIBCXX_ASSERTIONS")
elseif (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  string(APPEND CMAKE_CXX_FLAGS " -D_LIBCPP_HARDENING_MODE=_LIBCPP_HARDENING_MODE_EXTENSIVE")
endif()

# SFML è opzionale: se trovato, verrà collegato
find_package(SFML 2.6 COMPONENTS graphics QUIET)

# abilita address sanitizer e undefined-behaviour sanitizer in Debug mode
string(APPEND CMAKE_CXX_FLAGS_DEBUG " -fsanitize=address,undefined")
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU")
  string(APPEND CMAKE_CXX_FLAGS " -D_GLIBCXX_SANITIZE_STD_ALLOCATOR")
endif()
string(APPEND CMAKE_EXE_LINKER_FLAGS_DEBUG " -fsanitize=address,undefined")

# raccogli automaticamente tutti i sorgenti .cpp in cpp/ e quelli nella root
file(GLOB_RECURSE SOURCES_CPP CONFIGURE_DEPENDS "${CMAKE_SOURCE_DIR}/cpp/*.cpp")
file(GLOB ROOT_CPP CONFIGURE_DEPENDS "${CMAKE_SOURCE_DIR}/*.cpp")
set(PROJECT_SOURCES ${SOURCES_CPP} ${ROOT_CPP})

if (NOT PROJECT_SOURCES)
  message(FATAL_ERROR "Nessun file .cpp trovato in ./cpp o nella root del progetto. Aggiungi sorgenti prima di configurare.")
endif()

# eseguibile principale
add_executable(progetto ${PROJECT_SOURCES})
target_include_directories(progetto PRIVATE "${CMAKE_SOURCE_DIR}/include")
if (SFML_FOUND)
  target_link_libraries(progetto PRIVATE sfml-graphics)
endif()

# Targets per esecuzione: 'run' esegue l'eseguibile principale
add_custom_target(run
  COMMAND $<TARGET_FILE:progetto>
  DEPENDS progetto
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  COMMENT "Esegue l'eseguibile progetto"
)

# il testing e' abilitato di default
# per disabilitarlo, passare -DBUILD_TESTING=OFF a cmake durante la fase di configurazione
if (BUILD_TESTING)
  file(GLOB_RECURSE TEST_SOURCES CONFIGURE_DEPENDS "${CMAKE_SOURCE_DIR}/test/*.cpp")
  if (TEST_SOURCES)
    add_executable(progetto.t ${TEST_SOURCES})
    target_include_directories(progetto.t PRIVATE "${CMAKE_SOURCE_DIR}/include" "${CMAKE_SOURCE_DIR}/test")
    if (SFML_FOUND)
      target_link_libraries(progetto.t PRIVATE sfml-graphics)
    endif()
    add_test(NAME progetto.t COMMAND progetto.t)

    add_custom_target(run_tests
      COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
      DEPENDS progetto.t
      COMMENT "Esegue i test con ctest"
    )
  else()
    message(STATUS "Nessun file .cpp trovato in test/ — salto la creazione dell'eseguibile di test.")
  endif()
endif()