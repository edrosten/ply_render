CXX=g++ -std=c++0x -I rply-1.1.1
CC=gcc

debug=0

CXXFLAGS=-g -O2 -Wall -Wextra -W -ansi -pedantic -std=c++11 -std=c++0x
LDFLAGS= -lcvd

ifeq ($(debug),1)
CXXFLAGS+= -DTOON_NDEBUG -DNDEBUG -DTOON_CHECK_BOUNDS -D_GLIBCXX_DEBUG
else
endif

default:render scanline_render

render: render.o model_loader.o rply-1.1.1/rply.o runner.o
	$(CXX) -o $@  $^ $(LDFLAGS)

scanline_render: scanline_render.o model_loader.o rply-1.1.1/rply.o  scanline_render_test.o
	$(CXX) -o $@  $^ $(LDFLAGS)

clean:
	rm -f *.o render scanline_render
