SRC = src/Vis3d.cpp src/starter.cpp src/launcherimpl.cpp src/pr2_gui.cpp 
OUT = bin/pr2_gui
PKG = pr2_gui

CFLAGS = -g `wx-config --cxxflags` $(shell rospack export/cpp/cflags pr2_gui)
LFLAGS = `wx-config --libs` $(shell rospack export/cpp/lflags pr2_gui)

all:	$(SRC)
		g++ $(SRC) $(CFLAGS) $(LFLAGS) -o $(OUT)

