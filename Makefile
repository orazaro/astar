PROGRAMS = game15
TESTS =

OBJECTS     = g15_state.o
HFILES      = $(OBJECTS:%.o=%.h)
LIBRARIES	= cppunit

CC			= gcc
CXX			= g++
CFLAGS		+= -Wall -O3
CPPFLAGS		+= -Wall -O2
LDFLAGS		= $(LIBRARIES:%=-l%)

.PHONY: all clean test

all: $(PROGRAMS)

game15: game15.o $(OBJECTS) $(HFILES)
	$(CXX) $(OBJECTS) $< $(LDFLAGS) -o $@ 

%.o: %.cpp %.h
	$(CXX) -c $(CPPFLAGS) $<

%: %.cpp
	$(CXX) $(CPPFLAGS) $^ -o $@

%: %.c
	$(CC) $(CFLAGS) $^ -o $@

clean:
	rm -f *.o *.a $(PROGRAMS)

