CC:=g++
CC_FLAGS:=-g -std=c++11 -Isrc/ 
LD_FLAGS:=
#-L/Library/gurobi602/mac64/include/ 
CPP_FILES := $(wildcard src/*.cpp)
OBJ_FILES := $(addprefix obj/,$(notdir $(CPP_FILES:.cpp=.o)))

all: AMODBase

AMODBase: $(OBJ_FILES) examples/main.cpp
	$(CC) $(CC_FLAGS) $(LD_FLAGS) -o $@ $^ -lglpk

obj/%.o: src/%.cpp
	$(CC) $(CC_FLAGS) -c -o $@ $<
	
clean: 
	rm obj/*.o AMODBase


