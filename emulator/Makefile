all:    riscv-emulator

riscv-emulator: riscv-emulator.o
	g++ $^ -o $@

%.o:    %.cpp
	g++ -Wall -O3 -c $< -o $@

clean:
	rm -rf *.o riscv-emulator

