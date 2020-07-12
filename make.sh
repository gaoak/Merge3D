c++ -DFOILAOA=15 -DFOILM=0 -DFOILP=0 -DFOILT=12 -std=c++11 -I./CAD3D/ -I../Mesh2D/ *.cpp CAD3D/*.cpp -c -Og -g
c++ -std=c++11 -o mesh-g *.o  -g
./mesh-g
NekMesh test.xml testcomp.xml
