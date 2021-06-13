echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j8

echo "Configuring and building Thirdparty/g2o ..."

cd ../../g2o
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j8


echo "Uncompress vocabulary ..."

cd ../../../
cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..