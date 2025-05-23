cmake_minimum_required(VERSION 3.6)

project(E32DC_RemoteController)

set(Qt5_DIR C:\\Qt\\5.15.2\\msvc2019_64\\lib\\cmake\\Qt5)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

add_executable(E32DC_RemoteController

${CMAKE_SOURCE_DIR}/3rdparty/QJoysticks/Joysticks/JoysticksCommon.h

${CMAKE_SOURCE_DIR}/3rdparty/QJoysticks/Joysticks/Android_Joystick.h
${CMAKE_SOURCE_DIR}/3rdparty/QJoysticks/Joysticks/Android_Joystick.cpp

${CMAKE_SOURCE_DIR}/3rdparty/QJoysticks/Joysticks/SDL_Joysticks.h
${CMAKE_SOURCE_DIR}/3rdparty/QJoysticks/Joysticks/SDL_Joysticks.cpp

${CMAKE_SOURCE_DIR}/3rdparty/QJoysticks/Joysticks/VirtualJoystick.h
${CMAKE_SOURCE_DIR}/3rdparty/QJoysticks/Joysticks/VirtualJoystick.cpp

${CMAKE_SOURCE_DIR}/3rdparty/QJoysticks/QJoysticks.cpp
${CMAKE_SOURCE_DIR}/3rdparty/QJoysticks/QJoysticks.h

${CMAKE_SOURCE_DIR}/src/ControllerGui.ui
${CMAKE_SOURCE_DIR}/src/AutoscanComPortsGui.ui

${CMAKE_SOURCE_DIR}/src/ControllerWindow.h
${CMAKE_SOURCE_DIR}/src/ControllerWindow.cpp

${CMAKE_SOURCE_DIR}/src/SerialComm.h
${CMAKE_SOURCE_DIR}/src/SerialComm.cpp

${CMAKE_SOURCE_DIR}/src/UdpComm.h
${CMAKE_SOURCE_DIR}/src/UdpComm.cpp

${CMAKE_SOURCE_DIR}/src/PlotWidget.h
${CMAKE_SOURCE_DIR}/src/PlotWidget.cpp

${CMAKE_SOURCE_DIR}/src/RollPitchDisplay.h
${CMAKE_SOURCE_DIR}/src/RollPitchDisplay.cpp

${CMAKE_SOURCE_DIR}/src/CommandLineEdit.h
${CMAKE_SOURCE_DIR}/src/CommandLineEdit.cpp
${CMAKE_SOURCE_DIR}/src/SerialTerminalWidget.h
${CMAKE_SOURCE_DIR}/src/SerialTerminalWidget.cpp

${CMAKE_SOURCE_DIR}/src/HeadingDisplay.h
${CMAKE_SOURCE_DIR}/src/HeadingDisplay.cpp

${CMAKE_SOURCE_DIR}/src/JoystickBridge.h
${CMAKE_SOURCE_DIR}/src/JoystickBridge.cpp
 
${CMAKE_SOURCE_DIR}/src/main.cpp
)

find_package(Threads REQUIRED)
find_package(Qt5 COMPONENTS Core Gui Widgets SerialPort Network Concurrent REQUIRED)

find_package(SDL2)

if (SDL2_FOUND)
    target_include_directories(E32DC_RemoteController PUBLIC ${SDL2_INCLUDE_DIRS})
    target_link_libraries(E32DC_RemoteController ${SDL2_LIBRARIES})
    add_compile_definitions(SDL_SUPPORTED)
endif()

target_include_directories(E32DC_RemoteController PUBLIC ${CMAKE_SOURCE_DIR}/src/ ${CMAKE_SOURCE_DIR}/3rdparty/QJoysticks/)
target_link_libraries(E32DC_RemoteController Qt5::Core Qt5::Gui Qt5::Widgets Qt5::SerialPort Qt5::Network Qt5::Concurrent)
