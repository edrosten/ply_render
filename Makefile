CXX=g++-4.7 -std=c++0x -I rply-1.1.1
CC=gcc

debug=0

ifeq ($(debug),0)
CXXFLAGS=-Wall -Wextra -O3 -g -ggdb -std=c++0x -DTOON_NDEBUG -DNDEBUG
CXXFLAGS=-Wall -Wextra -O3 -g -ggdb -std=c++0x 
LDFLAGS=-lGVars3 -lcvd 
else
CXXFLAGS=-Wall -Wextra -O0 -g -ggdb -std=c++0x -DTOON_CHECK_BOUNDS -D_GLIBCXX_DEBUG
LDFLAGS=-lGVars3_debug -lcvd_debug 
endif

default:render

render: render.o model_loader.o rply-1.1.1/rply.o simple_vertex_io.o
	$(CXX) -o $@  $^ $(LDFLAGS)

clean:
	rm *.o
