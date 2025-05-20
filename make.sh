cd InputMesh
if [ ! -f 3DoutUcomp.xml ]; then
  bash extrude.sh
fi
cd ..
cd build
cmake ..
make
cd ..
./build/mesh
if [ "0" = "$?" ]; then
    rm testcomp.xml
    NekMesh test.xml testcomp.xml
    echo "compressed testcomp.xml"
fi
