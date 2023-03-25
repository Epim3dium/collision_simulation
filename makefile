CC=clang++
DEPS=$(wildcard include/*.h) $(wildcard include/*.hpp)

CFLAGS=@compile_flags.txt

SRC_DIR := ./src
OBJ_DIR := ./obj
SRC_FILES := $(wildcard $(SRC_DIR)/*.cpp)
OBJ_FILES := $(patsubst $(SRC_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(SRC_FILES))

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS) -framework openGL

SFML_OBJ=vendor/imgui/imguilib.a

main: $(OBJ_FILES) $(SFML_OBJ)
	$(CC) -o main.exe $^ $(CFLAGS) -framework openGL

collision.a: $(OBJ_FILES) $(SFML_OBJ)
	ar -rcs collision.a *.o


clean:
	rm -f imgui.ini
	rm -f obj/*
	rm -f collision.a
	rm -f main.exe
