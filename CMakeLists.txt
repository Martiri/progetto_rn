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

# copia il font nella directory di build
configure_file(${CMAKE_SOURCE_DIR}/arial.ttf ${CMAKE_BINARY_DIR}/arial.ttf COPYONLY)
# abilita warning
#string(APPEND CMAKE_CXX_FLAGS
 #     " -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion"
   # " -Wshadow -Wimplicit-fallthrough -Wextra-semi -Wold-style-cast"
  #  " -fno-omit-frame-pointer")

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
find_package(SFML 2.6 COMPONENTS graphics REQUIRED)
find_package(TBB REQUIRED)
#crea una libreria condivisa "boids_core" con i file sorgente della logica
add_library(boids_core flock.cpp boid.cpp slider.cpp predator.cpp simgraphics.cpp simvalues.cpp Vector2D.cpp)
target_link_libraries(boids_core PRIVATE TBB::tbb)


# crea l'eseguibile principale progetto_run
add_executable(progetto_run main.cpp)
target_link_libraries(progetto_run PRIVATE boids_core sfml-graphics)
# aggiungere eventuali altri eseguibili
# Aggiunge un comando custom "run"
add_custom_target(run
    COMMAND progetto_run
    DEPENDS progetto_run
    WORKING_DIRECTORY ${CMAKE_PROJECT_DIR}
    COMMENT "Avvio la simulazione..."
)
# il testing e' abilitato di default
# per disabilitarlo, passare -DBUILD_TESTING=OFF a cmake durante la fase di configurazione
if (BUILD_TESTING)

  # aggiungi l'eseguibile progetto_test
  add_executable(progetto_test flock.t.cpp)
  target_link_libraries(progetto_test PRIVATE boids_core sfml-graphics)
  # aggiungi l'eseguibile progetto_test alla lista dei test
  add_test(NAME progetto_test COMMAND progetto_test)

# Aggiunge un comando custom "check" per i test
add_custom_target(check
   COMMAND progetto_test
  DEPENDS progetto_test
  WORKING_DIRECTORY ${CMAKE_PROJECT_DIR}
  COMMENT "Esecuzione dei test..."
)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -flto -march=native -Wall -Wextra")
set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -flto")