CC=clang++
DEPS=$(wildcard include/*.h) $(wildcard include/*.hpp)

CFLAGS=@compile_flags.txt
rwildcard=$(foreach d,$(wildcard $(1:=/*)),$(call rwildcard,$d,$2) $(filter $(subst *,%,$2),$d))

SRC_DIR := ./src
OBJ_DIR := ./obj
SRC_FILES := $(call rwildcard,$(SRC_DIR),*.cpp)
OBJ_FILES := $(patsubst $(SRC_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(SRC_FILES))

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS) -framework openGL

SFML_OBJ=vendor/imgui/imguilib.a

main: $(OBJ_FILES) $(SFML_OBJ)
	$(CC) -o main.exe main.cpp $^ $(CFLAGS) -framework openGL

collision.a: $(OBJ_FILES) $(SFML_OBJ)
	ar -rcs collision.a $(OBJ_FILES)


clean:
	rm -f imgui.ini
	rm -f $(OBJ_FILES) 
	rm -f collision.a
	rm -f main.exe
