c++ -std=c++11 *.cpp CAD3D/*.cpp -c -Og -g
c++ -std=c++11 *.o -Og -o mesh-g -ltinyxml2 -g
./mesh-g
