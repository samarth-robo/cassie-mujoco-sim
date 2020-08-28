# Build for linux by default
PLATFORM := LINUX

# Compilation settings
INC     := -Iinclude -I../include -I$(HOME)/.mujoco/mujoco200_linux/include
CFLAGS  := -std=gnu11 -Wall -Wextra -O3 -march=sandybridge -flto
LDFLAGS := -L. -L..
LIBS    := -lcassiemujoco

# Platform-specific settings
ifeq ($(PLATFORM), WIN)
CC      := x86_64-w64-mingw32-gcc
TESTOUT := cassietest.exe
SIMOUT  := cassiesim.exe
CTRLOUT := cassiectrl.exe
IKOUT   := cassieik.exe
else
CC      := gcc
LDFLAGS += -Wl,-rpath,'$$ORIGIN'
TESTOUT := cassietest
SIMOUT  := cassiesim
CTRLOUT := cassiectrl
IKOUT   := cassieik
endif

# Default target
all: $(TESTOUT) $(SIMOUT) $(CTRLOUT) $(IKOUT)

# Normal targets
clean:
	rm -f $(TESTOUT)
	rm -f $(SIMOUT)
	rm -f $(CTRLOUT)
	rm -f $(IKOUT)

$(TESTOUT): cassietest.c
	$(CC) cassietest.c $(INC) $(CFLAGS) $(LDFLAGS) $(LIBS) -o $(TESTOUT)

$(SIMOUT): cassiesim.c
	$(CC) cassiesim.c $(INC) $(CFLAGS) $(LDFLAGS) $(LIBS) -o $(SIMOUT)

$(CTRLOUT): cassiectrl.c
	$(CC) cassiectrl.c $(INC) $(CFLAGS) $(LDFLAGS) $(LIBS) -o $(CTRLOUT)

$(IKOUT): cassieik.cpp
	$(CC) cassieik.cpp $(INC) `pkg-config --cflags eigen3` $(CFLAGS) $(LDFLAGS) $(LIBS) -lstdc++ -L$(HOME)/.mujoco/mujoco200_linux/bin -lmujoco200 -lGL -lglew $(HOME)/.mujoco/mujoco200_linux/bin/libglfw.so.3 -o $(IKOUT)
# Virtual targets
.PHONY: all clean
