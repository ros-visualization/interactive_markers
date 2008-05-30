.PHONY: install_lib all clean

CPP = g++

# Default Target
all: build_lib

headers = $(filter %.hh %.h, $(1))
noheaders = $(filter-out %.hh %.h, $(1))
nosources = $(filter-out %.cc %.c, $(1))
objects = $(filter %.o, $(1:%.c=%.o) $(1:%.cc=%.o)) $(call noheaders,$(call nosources,$(1)))



CPPFLAGS = $(shell rospack export/cpp/cflags irrlicht_viewer)
LDFLAGS =  $(shell rospack export/cpp/lflags irrlicht_viewer)

CUSTOM_NODES = $(shell ls CustomNodes/*.cc)

ILRENDER_LIB = libilrender.a

# Generic Targets
%.o: %.cc
	$(CPP) $(CPPFLAGS) -O2 -c $< -o $@

# Library Targets


$(ILRENDER_LIB):  $(call objects,$(CUSTOM_NODES))
	ar -rcs $(ILRENDER_LIB) $(call objects,$(CUSTOM_NODES))

#g++ $(CPPFLAGS) -o $@  $(call objects,$(CUSTOM_NODES))

# Executable Targets
test: ILRenderTest.cc ILRender.o ILClient.o $(ILRENDER_LIB) 
	$(CPP) $(CPPFLAGS) -o $@ $^  $(LDFLAGS)

build_lib: $(ILRENDER_LIB)

# Special Targets

clean:
	$(CLEANCOMMAND)
	rm -rf test
	cd $(ILRENDER_PATH)/CustomNodes && $(CLEANCOMMAND)


