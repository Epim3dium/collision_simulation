CC=clang++
DEPS=$(wildcard include/*.h) $(wildcard include/*.hpp)

CFLAGS=@compile_flags.txt

OBJ=sim.o utils.o col_utils.o collision.o node.o physics_manager.o rigidbody.o
EXECUTABLE=main.o 

%.o: %.cpp $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS) -framework openGL

SFML_OBJ=vendor/imgui/imguilib.a

main.exe: $(OBJ) $(EXECUTABLE) $(SFML_OBJ)
	$(CC) -o $@ $^ $(CFLAGS) -framework openGL
collision.a: $(OBJ) $(SFML_OBJ)
	ar -rcs collision.a *.o


clean:
	rm -f imgui.ini
	rm -f $(wildcard *.o)
	rm -f collision.a
	rm -f main.exe
