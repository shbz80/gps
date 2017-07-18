COMMON=-O2 -I../include -L../bin -std=c++11

default:
	g++ $(COMMON) test.cpp -lmujoco140nogl -o ../bin/test
	g++ $(COMMON) compile.cpp -lmujoco140nogl -o ../bin/compile
	g++ $(COMMON) derivative.cpp -lmujoco140nogl -fopenmp -o ../bin/derivative
	g++ $(COMMON) simulate.cpp -lmujoco140 -lGL -lglew ../bin/libglfw.so.3 -o ../bin/simulate
	g++ $(COMMON) record.cpp -lmujoco140 -lGL -lglew ../bin/libglfw.so.3 -o ../bin/record

egl:	
	g++ $(COMMON) -DMJ_EGL record.cpp -lmujoco140 -lOpenGL -lEGL -lglewegl -o ../bin/recordegl

osmesa:	
	g++ $(COMMON) -DMJ_OSMESA record.cpp -lmujoco140 -lOSMesa -lglewosmesa -o ../bin/recordosmesa

all: default egl osmesa
