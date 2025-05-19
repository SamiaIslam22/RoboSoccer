# Webots Makefile
CXX_SOURCES = referee.cpp
INCLUDE = -I"$(WEBOTS_HOME)/include/controller/cpp"
LIBRARIES = -L"$(WEBOTS_HOME)/lib/controller" -lController -lCppController
TARGET = referee
LDFLAGS = -mconsole

### Generic Makefile.include
WEBOTS_HOME_PATH=C:/PROGRA~1/Webots
include $(WEBOTS_HOME_PATH)/resources/Makefile.include