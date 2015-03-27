CC:=g++
CC_FLAGS:=-g -O2 -std=c++11
LD_FLAGS:=
CPP_FILES := $(wildcard *.cpp)
OBJ_FILES := $(addprefix obj/,$(notdir $(CPP_FILES:.cpp=.o)))

all: AMODBasicTest

AMODBasicTest: $(OBJ_FILES)
	$(CC) $(LD_FLAGS) -o $@ $^

obj/%.o: %.cpp
	$(CC) $(CC_FLAGS) -c -o $@ $<
