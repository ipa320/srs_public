all: installed

TARBALL		= build/pellet-2.3.0.zip
TARBALL_URL 	= http://dl.clarkparsia.com/downloads/f2539fb8811e4de449a2e801e77d91f5/pellet-2.3.0.zip
UNPACK_CMD  	= unzip -uo
SOURCE_DIR 	= build/pellet-2.3.0

include $(shell rospack find mk)/download_unpack_build.mk

installed: $(SOURCE_DIR)/unpacked
	mkdir -p common/lib
	cp -r $(SOURCE_DIR)/lib common
clean:
	rm -rf common $(SOURCE_DIR) $(TARBALL)
wipe: clean
	-rm -rf build
