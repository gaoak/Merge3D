cd InputMesh
if [ ! -f 3DoutUcomp.xml ]; then
  bash extrude.sh
fi
cd ..
c++ -std=c++11 *.cpp CAD3D/*.cpp -c -Og -g
c++ -std=c++11 -o mesh-g *.o  -g
./mesh-g
if [ "0" = "$?" ]; then
    rm testcomp.xml
    NekMesh test.xml testcomp.xml
    echo "compressed testcomp.xml"
fi
