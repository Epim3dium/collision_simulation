CC=clang++
DEPS=$(wildcard include/*.h) $(wildcard include/*.hpp)

CFLAGS=@compile_flags.txt

OBJ=types.o col_utils.o solver.o physics_manager.o rigidbody.o restraint.o particle.o particle_manager.o crumbler.o
DEMO=main.o sim.o

%.o: %.cpp $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS) -framework openGL

SFML_OBJ=vendor/imgui/imguilib.a

main: $(OBJ) $(DEMO) $(SFML_OBJ)
	$(CC) -o main.exe $^ $(CFLAGS) -framework openGL

perf: $(OBJ) $(PERF) $(SFML_OBJ)
	$(CC) -o main.exe $^ $(CFLAGS) -framework openGL

collision.a: $(OBJ) $(SFML_OBJ)
	ar -rcs collision.a *.o


clean:
	rm -f imgui.ini
	rm -f $(wildcard *.o)
	rm -f collision.a
	rm -f main.exe
