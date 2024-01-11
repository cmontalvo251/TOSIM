OBJ=	source/TOSIM.o \
	source/quadcopter.o
CC = gfortran #Define compiler (CC) to be gfortran compiler
FFLAGS= -c -fno-automatic -O3
MAINFLAGS = -c -w -ffree-form -ffree-line-length-none -fdollar-ok -fcray-pointer -fdefault-real-8 -fdefault-double-8 -O3 #-fcheck=all
INCLUDE = #-I./libs/Linux -I./libs/Linux/modfiles - Archaic from AREA-I days
LIBS = #-L./libs/Linux -lWingsX - Archaic from AREA-I days
EXECUTABLE=LinuxRun.exe

all: $(OBJ) $(EXECUTABLE)

$(EXECUTABLE): $(OBJ)
	$(CC) -O3 -o $(EXECUTABLE) $(OBJ)
source/quadcopter.o: source/quadcopter.f90
	$(CC) $(MAINFLAGS) source/quadcopter.f90 -o source/quadcopter.o
source/TOSIM.o: source/TOSIM.f90
#	mkdir Output_Files
	rm -rf *.o LinuxRun.exe
	$(CC) $(MAINFLAGS) source/TOSIM.f90
	mv TOSIM.o source/TOSIM.o
	mv tosimdatatypes.mod source/tosimdatatypes.mod
clean:
	rm -rf *.o LinuxRun.exe *.mod
	rm -rf source/*.o source/*.mod
rebuild:
	make clean
	make
