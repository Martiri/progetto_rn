cmake_minimum_required(VERSION 3.28)
project(progettopf VERSION 0.1.0)

# abilita il supporto per i test, tra cui l'opzione BUILD_TESTING usata sotto
include(CTest)

# esporta il compilation database, utile per altri strumenti, a partire dall'editor
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# richiedi l'uso di C++20 (volendo anche C++23), senza estensioni non-standard offerte dal compilatore usato
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# abilita warning
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

# abilita address sanitizer e undefined-behaviour sanitizer in Debug mode
string(APPEND CMAKE_CXX_FLAGS_DEBUG " -fsanitize=address,undefined")
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU")
  string(APPEND CMAKE_CXX_FLAGS " -D_GLIBCXX_SANITIZE_STD_ALLOCATOR")
endif()
string(APPEND CMAKE_EXE_LINKER_FLAGS_DEBUG " -fsanitize=address,undefined")

# se usato, richiedi il componente graphics della libreria SFML (versione 2.6 in Ubuntu 24.04)
find_package(SFML 2.6 COMPONENTS graphics)

# raccolgo automaticamente i sorgenti nella cartella cpp/
file(GLOB_RECURSE PROJECT_SOURCES CONFIGURE_DEPENDS "${CMAKE_SOURCE_DIR}/cpp/*.cpp")

if (NOT PROJECT_SOURCES)
  message(FATAL_ERROR "Nessun sorgente trovato in ${CMAKE_SOURCE_DIR}/cpp/. Assicurati che i file .cpp siano presenti.")
endif()

# crea l'eseguibile principale
add_executable(progetto ${PROJECT_SOURCES})

# includi la cartella include/ per gli header
target_include_directories(progetto PRIVATE "${CMAKE_SOURCE_DIR}/include")

# se SFML è stato trovato, collegalo al target
if (SFML_FOUND)
  target_link_libraries(progetto PRIVATE sfml-graphics)
else()
  message(STATUS "SFML non trovata: salto il linking con sfml-graphics.")
endif()

# Target di utilità per eseguire l'applicazione dalla build directory:
# usa: `cmake --build build --target run` per compilare (se necessario) e lanciare
add_custom_target(run
  COMMAND $<TARGET_FILE:progetto>
  DEPENDS progetto
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)

# Testing: cerca sorgenti di test in test/ e crea un eseguibile di test se presenti
if (BUILD_TESTING)
  file(GLOB_RECURSE TEST_SOURCES CONFIGURE_DEPENDS "${CMAKE_SOURCE_DIR}/test/*.cpp")
  if (TEST_SOURCES)
    add_executable(tests ${TEST_SOURCES})
    target_include_directories(tests PRIVATE "${CMAKE_SOURCE_DIR}/include" "${CMAKE_SOURCE_DIR}/test")
    add_test(NAME tests COMMAND tests)
  else()
    message(STATUS "BUILD_TESTING=ON ma non sono stati trovati file di test in test/.")
  endif()
endif()