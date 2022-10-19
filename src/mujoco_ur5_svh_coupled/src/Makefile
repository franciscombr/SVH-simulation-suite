COMMON=-O2 -I /etc/mujoco-2.2.1/include -L /etc/mujoco-2.2.1/lib -std=c++17 -pthread -Wl,-no-as-needed -Wl,-rpath,'$$ORIGIN'/etc/mujoco-2.2.1/lib

all:
	$(CC) -c -O2 -I /etc/mujoco-2.2.1/include /etc/mujoco-2.2.1/sample/uitools.c
	$(CXX) $(COMMON) uitools.o simulate.cpp -lmujoco -lglfw  -o simulate
	rm *.o