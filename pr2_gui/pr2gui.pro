TEMPLATE	= app
LANGUAGE	= C++

CONFIG	+= qt warn_on release

LIBS	+= $(shell rospack export/cpp/lflags pr2_gui)

SOURCES	+= main.cpp

FORMS	= Launcher.ui \
	Visualization.ui \
	PTZ.ui \
	Stereo.ui \
	Wrist.ui \
	Topdown.ui \
	Status.ui

IMAGES	= images/filenew \
	images/fileopen \
	images/filesave \
	images/print \
	images/undo \
	images/redo \
	images/editcut \
	images/editcopy \
	images/editpaste \
	images/searchfind \
	images/filenew_1 \
	images/fileopen_1 \
	images/filesave_1 \
	images/print_1 \
	images/undo_1 \
	images/redo_1 \
	images/editcut_1 \
	images/editcopy_1 \
	images/editpaste_1 \
	images/searchfind_1 \
	images/filenew_2 \
	images/fileopen_2 \
	images/filesave_2 \
	images/print_2 \
	images/undo_2 \
	images/redo_2 \
	images/editcut_2 \
	images/editcopy_2 \
	images/editpaste_2 \
	images/searchfind_2 \
	images/filenew_3 \
	images/fileopen_3 \
	images/filesave_3 \
	images/print_3 \
	images/undo_3 \
	images/redo_3 \
	images/editcut_3 \
	images/editcopy_3 \
	images/editpaste_3 \
	images/searchfind_3 \
	images/filenew_4 \
	images/fileopen_4 \
	images/filesave_4 \
	images/print_4 \
	images/undo_4 \
	images/redo_4 \
	images/editcut_4 \
	images/editcopy_4 \
	images/editpaste_4 \
	images/searchfind_4 \
	images/filenew_5 \
	images/fileopen_5 \
	images/filesave_5 \
	images/print_5 \
	images/undo_5 \
	images/redo_5 \
	images/editcut_5 \
	images/editcopy_5 \
	images/editpaste_5 \
	images/searchfind_5 \
	images/filenew_6 \
	images/fileopen_6 \
	images/filesave_6 \
	images/print_6 \
	images/undo_6 \
	images/redo_6 \
	images/editcut_6 \
	images/editcopy_6 \
	images/editpaste_6 \
	images/searchfind_6 \
	images/filenew_7 \
	images/fileopen_7 \
	images/filesave_7 \
	images/print_7 \
	images/undo_7 \
	images/redo_7 \
	images/editcut_7 \
	images/editcopy_7 \
	images/editpaste_7 \
	images/searchfind_7

CFLAGS += $(shell rospack export/cpp/cflags pr2gui)





unix {
  UI_DIR = .ui
  MOC_DIR = .moc
  OBJECTS_DIR = .obj
}

