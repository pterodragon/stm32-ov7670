git submodule update --init --recursive
cd openocd
./bootstrap
./configure
make
popd
make

