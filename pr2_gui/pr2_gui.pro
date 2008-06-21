TEMPLATE = app

QMAKE_LIBS = $$quote($$quote($$quote($(shell rospack export/cpp/lflags pr2_gui))))

QMAKE_CXXFLAGS_RELEASE = $$quote($$quote($$quote($(shell rospack export/cpp/cflags pr2_gui))))

QMAKE_CXXFLAGS_DEBUG = $$quote($$quote($$quote($(shell rospack export/cpp/cflags pr2_gui))))

QMAKE_CFLAGS_RELEASE = $$quote($$quote($$quote($(shell rospack export/cpp/cflags pr2_gui))))

QMAKE_CFLAGS_DEBUG = $$quote($$quote($$quote($(shell rospack export/cpp/cflags pr2_gui))))

QT = gui \
	core

CONFIG += qt \
	release \
	warn_on \
	console

DESTDIR = bin

OBJECTS_DIR = build

MOC_DIR = build

UI_DIR = build

FORMS = ui/launcher.ui

HEADERS = src/launcherimpl.h

SOURCES = src/launcherimpl.cpp \
	src/main.cpp \
	src/Vis3d.hh \
	src/ILModel.cpp \
	../irrlicht_viewer/ILRender.cc \
	../irrlicht_viewer/ILClient.cc
