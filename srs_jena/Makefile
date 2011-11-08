all: installed

TARBALL		= build/jena-2.6.4.zip
TARBALL_URL 	= http://sourceforge.net/projects/jena/files/Jena/Jena-2.6.4/jena-2.6.4.zip/download
UNPACK_CMD  	= unzip
SOURCE_DIR 	= build/jena-2.6.4
MD5SUM_FILE 	= jena-2.6.4.zip.md5sum

include $(shell rospack find mk)/download_unpack_build.mk

installed:
	ls
clean:
	rm -rf $(SOURCE_DIR) $(TARBALL)
wipe: clean
	-rm -rf build
