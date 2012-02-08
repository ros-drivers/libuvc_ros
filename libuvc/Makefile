#include $(shell rospack find mk)/cmake.mk
.PHONY: clean download

CMAKE=cmake
CMAKE_ARGS=-DCMAKE_BUILD_TYPE=Release \
	   -DCMAKE_INSTALL_PREFIX=`rospack find libuvc`/libuvc

all:
	mkdir -p build
	@git clone git://github.com/ktossell/libuvc.git build/git || (echo "INFO: skipping clone"; true)
	cd build/git && git pull
	mkdir -p build/build
	cd build/build && $(CMAKE) ../git $(CMAKE_ARGS)
	cd build/build && make && make install

clean:
	rm -rf build libuvc

