c++ -std=c++11 *.cpp CAD3D/*.cpp -c -Og -g
c++ -std=c++11 -o mesh-g *.o  -g
./mesh-g
if [ "0" = "$?" ]; then
    NekMesh test.xml testcomp.xml
    echo "compressed testcomp.xml"
fi
