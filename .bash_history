ls -lah
ls -lah sit-aw-anchoring/
exit
ls -lah
exit
cd sit-aw-anchoring/humble/src/
cd anchoring-typedb/
rm -rf build && mkdir build && cd build/
cmake ..
exit
cd sit-aw-anchoring/humble/src/anchoring-typedb/build/
cmake ..
make
./anchoring test
cd ..
./build/anchoring test
./build/anchoring test2
typedb-studio 
exit
