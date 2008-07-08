SRC = starter.cpp launcherimpl.cpp pr2_gui.cpp
OUT = pr2_gui
PKG = pr2_gui

CFLAGS = `wx-config --cxxflags` $(shell rospack export/cpp/cflags pr2_gui)
LFLAGS = `wx-config --libs` $(shell rospack export/cpp/lflags pr2_gui)

all:	$(SRC)
		g++ $(SRC) $(CFLAGS) $(LFLAGS) -o $(OUT)

