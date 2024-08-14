# Makefile : builds libbreezyslam.so C++ library
#
# Copyright (C) Simon D. Levy 2014
#
# This code is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as 
# published by the Free Software Foundation, either version 3 of the 
# License, or (at your option) any later version.
#
# This code is distributed in the hope that it will be useful,     
# but WITHOUT ANY WARRANTY without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public License 
#  along with this code.  If not, see <http:#www.gnu.org/licenses/>.

# Where you want to put the library
LIBDIR = /usr/local/lib

# Set library extension based on OS
ifeq ("$(shell uname)","Darwin")
  LIBEXT = dylib
else ifeq ("$(shell uname)","Linux")
  CFLAGS = -fPIC
  LIBEXT = so
else
  LIBEXT = dll
endif

ARCH = $(shell uname -m)

# Set SIMD compile params based on architecture
ifeq ("$(ARCH)","armv7l")
  SIMD_FLAGS = -mfpu=neon
else ifeq ("$(ARCH)","i686")
  SIMD_FLAGS = -msse3
else
  ARCH = sisd
endif

# Build directory
BUILDDIR = slambuild

# Ensure build directory exists
$(shell mkdir -p $(BUILDDIR))

all: $(BUILDDIR)/libbreezyslam.$(LIBEXT)

test: $(BUILDDIR)/breezytest
	./$(BUILDDIR)/breezytest

$(BUILDDIR)/libbreezyslam.$(LIBEXT): $(BUILDDIR)/algorithms.o $(BUILDDIR)/Scan.o $(BUILDDIR)/Map.o \
                                    $(BUILDDIR)/WheeledRobot.o $(BUILDDIR)/coreslam.o \
                                    $(BUILDDIR)/coreslam_$(ARCH).o $(BUILDDIR)/random.o $(BUILDDIR)/ziggurat.o
	g++ -O3 -shared $(BUILDDIR)/algorithms.o $(BUILDDIR)/Scan.o $(BUILDDIR)/Map.o \
                        $(BUILDDIR)/WheeledRobot.o $(BUILDDIR)/coreslam.o $(BUILDDIR)/coreslam_$(ARCH).o \
                        $(BUILDDIR)/random.o $(BUILDDIR)/ziggurat.o \
          -o $(BUILDDIR)/libbreezyslam.$(LIBEXT) -lm

$(BUILDDIR)/algorithms.o: algorithms.cpp algorithms.hpp Laser.hpp Position.hpp Map.hpp Scan.hpp PoseChange.hpp \
                          WheeledRobot.hpp ./c/coreslam.h 
	g++ -O3 -I./c -c -Wall $(CFLAGS) algorithms.cpp -o $(BUILDDIR)/algorithms.o

$(BUILDDIR)/Scan.o: Scan.cpp Scan.hpp PoseChange.hpp Laser.hpp ./c/coreslam.h
	g++ -O3 -I./c -c -Wall $(CFLAGS) Scan.cpp -o $(BUILDDIR)/Scan.o

$(BUILDDIR)/Map.o: Map.cpp Map.hpp Position.hpp Scan.hpp ./c/coreslam.h
	g++ -O3 -I./c -c -Wall $(CFLAGS) Map.cpp -o $(BUILDDIR)/Map.o

$(BUILDDIR)/WheeledRobot.o: WheeledRobot.cpp WheeledRobot.hpp 
	g++ -O3 -I./c -c -Wall $(CFLAGS) WheeledRobot.cpp -o $(BUILDDIR)/WheeledRobot.o

$(BUILDDIR)/coreslam.o: ./c/coreslam.c ./c/coreslam.h
	gcc -O3 -c -Wall $(CFLAGS) ./c/coreslam.c -o $(BUILDDIR)/coreslam.o

$(BUILDDIR)/coreslam_$(ARCH).o: ./c/coreslam_$(ARCH).c ./c/coreslam.h
	gcc -O3 -c -Wall $(CFLAGS) $(SIMD_FLAGS) ./c/coreslam_$(ARCH).c -o $(BUILDDIR)/coreslam_$(ARCH).o

$(BUILDDIR)/random.o: ./c/random.c
	gcc -O3 -c -Wall $(CFLAGS) ./c/random.c -o $(BUILDDIR)/random.o
    
$(BUILDDIR)/ziggurat.o: ./c/ziggurat.c
	gcc -O3 -c -Wall $(CFLAGS) ./c/ziggurat.c -o $(BUILDDIR)/ziggurat.o
    
install: $(BUILDDIR)/libbreezyslam.$(LIBEXT)
		mkdir -p build
		g++ -O3 -c -I ./slambuild log2pgm.cc -o build/log2pgm.o
		g++ -O3 -o build/log2pgm build/log2pgm.o -L/$(BUILDDIR) -lbreezyslam
run:./build/log2pgm
	./build/log2pgm laser_data 0 9999 eog laser_data.pgm
# cp $(BUILDDIR)/libbreezyslam.$(LIBEXT) $(LIBDIR)
    
doc:
	doxygen

clean:
	rm -rf $(BUILDDIR) libbreezyslam.$(LIBEXT) Documentation \#* *~
	rm -rf build/log2pgm build/log2pgm.o