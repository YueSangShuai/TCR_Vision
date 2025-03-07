cmake_minimum_required(VERSION 3.23)
project(AutoChange)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)


find_package(Qt5 COMPONENTS
        Core
        Gui
        Widgets
        REQUIRED)

add_executable(AutoChange main.cpp mianwindow.cpp mianwindow.h mianwindow.ui Tool.h)
target_link_libraries(AutoChange
        Qt5::Core
        Qt5::Gui
        Qt5::Widgets
        )

