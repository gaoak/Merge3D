c++ -std=c++11 -I./CAD3D/ *.cpp CAD3D/*.cpp -c -Og -g
c++ -std=c++11 -o mesh-g *.o  -g
./mesh-g
