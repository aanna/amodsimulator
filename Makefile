CC:=g++
CC_FLAGS:=-g -O2 -std=c++11 -Isrc/
LD_FLAGS:=
CPP_FILES := $(wildcard src/*.cpp)
OBJ_FILES := $(addprefix obj/,$(notdir $(CPP_FILES:.cpp=.o)))

all: AMODBasicTest

AMODBasicTest: $(OBJ_FILES) examples/main.cpp
	$(CC) $(CC_FLAGS) $(LD_FLAGS) -o $@ $^ 

obj/%.o: src/%.cpp
	$(CC) $(CC_FLAGS) -c -o $@ $<
	
clean: 
	rm obj/*.o AMODBasicTest

