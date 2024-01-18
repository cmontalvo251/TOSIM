OBJ=	source/TOSIM.o
CC = gfortran #Define compiler (CC) to be gfortran compiler
FFLAGS= -c -fno-automatic -O3
MAINFLAGS = -c -w -ffree-form -ffree-line-length-none -fdollar-ok -fcray-pointer -fdefault-real-8 -fdefault-double-8 -O3 #-fcheck=all
INCLUDE = #-I./libs/Linux -I./libs/Linux/modfiles - Archaic from AREA-I days
LIBS = #-L./libs/Linux -lWingsX - Archaic from AREA-I days
EXECUTABLE=Simulation.exe

all: $(OBJ) $(EXECUTABLE)

$(EXECUTABLE): $(OBJ)
	$(CC) -O3 -o $(EXECUTABLE) $(OBJ)
source/TOSIM.o: source/TOSIM.f90
#	mkdir Output_Files
	rm -rf *.o *.exe
	$(CC) $(MAINFLAGS) source/TOSIM.f90
	mv TOSIM.o source/TOSIM.o
clean:
	rm -rf *.o *.exe *.mod
	rm -rf source/*.o source/*.mod
rebuild:
	make clean
	make
