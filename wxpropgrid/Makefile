all: installed

VERSION 	= wxpropgrid-1.4.6
FILENAME	= $(VERSION)-src.tar.gz
TARBALL		= build/$(VERSION)-src.tar.gz
TARBALL_URL	= http://pr.willowgarage.com/downloads/$(FILENAME)
UNPACK_CMD	= tar xzf
SOURCE_DIR	= build/propgrid
#MD5SUM_FILE 	= $(FILENAME).md5sum
WXPROPGRID_ROOT = $(shell rospack find wxpropgrid)

include $(shell rospack find mk)/download_unpack_build.mk

installed: $(SOURCE_DIR)/unpacked
	cd $(SOURCE_DIR) && \
		export CFLAGS='-g -D__NOTWXPYTHON__ -g' && \
		export CXXFLAGS='-g -D__NOTWXPYTHON__ -g' && \
		./configure --prefix=$(WXPROPGRID_ROOT)/propgrid_install && \
		make && \
		make install
	touch installed

clean:
	-cd $(SOURCE_DIR) && \
		make clean && \
		rm -f Makefile
	-rm -rf propgrid_install
	-rm -f installed

wipe: clean
	-rm -rf build
	-rm -rf $(TARBALL)


.PHONY: clean
