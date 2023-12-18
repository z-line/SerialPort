add_library(serialport OBJECT SerialPort.cpp)
target_link_libraries(serialport PUBLIC utils)
target_link_directories(serialport PUBLIC .)
