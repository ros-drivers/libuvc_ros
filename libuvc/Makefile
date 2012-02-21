#include $(shell rospack find mk)/cmake.mk
.PHONY: clean download

BUILD_TYPE?=Release
CMAKE=cmake
CMAKE_ARGS=-DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
	   -DCMAKE_INSTALL_PREFIX=`rospack find libuvc`/libuvc

all:
	mkdir -p build
	@git clone git://github.com/ktossell/libuvc.git build/git || (echo "INFO: skipping clone"; true)
	cd build/git && git pull
	mkdir -p build/build
	cd build/build && $(CMAKE) ../git $(CMAKE_ARGS)
	cd build/build && make && make install
	
	# you shouldn't have to do this, but it works for the Mac
	if [ `uname` = Darwin ]; then \
		install_name_tool -id `rospack find libuvc`/libuvc/lib/libuvc.dylib `rospack find libuvc`/libuvc/lib/libuvc.dylib; \
	fi

clean:
	rm -rf build/build libuvc

