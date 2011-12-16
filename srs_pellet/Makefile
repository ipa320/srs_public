all: installed

TARBALL		= build/pellet-2.3.0.zip
TARBALL_URL 	= http://dl.dropbox.com/u/29720262/pellet-2.3.0.zip
UNPACK_CMD  	= unzip -uo
SOURCE_DIR 	= build/pellet-2.3.0
MD5SUM_FILE	= pellet-2.3.0.md5sum

include $(shell rospack find mk)/download_unpack_build.mk

installed: $(SOURCE_DIR)/unpacked
	mkdir -p common/lib
	cp -r $(SOURCE_DIR)/lib common
clean:
	rm -rf common $(SOURCE_DIR) $(TARBALL)
wipe: clean
	-rm -rf build
