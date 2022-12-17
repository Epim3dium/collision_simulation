CC=clang++
DEPS=$(wildcard include/*.h) $(wildcard include/*.hpp)

CFLAGS=@compile_flags.txt

OBJ=main.o sim.o utils.o col_utils.o collision.o node.o soft_body.o physics_manager.o

%.o: %.cpp $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS) -framework openGL

SFML_OBJ=vendor/imgui/imguilib.a

main.exe: $(OBJ) $(SFML_OBJ)
	$(CC) -o $@ $^ $(CFLAGS) -framework openGL

clean:
	rm -f imgui.ini
	rm -f $(wildcard *.o)
	rm -f main.exe
