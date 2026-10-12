#include "PipeMesh.h"
#include "params3D.h"
#include <iostream>
#include <string>

int main(int argc, char *argv[]) {
  try {
    const BentPipe pipe(pipeParameters());
    if (argc == 2 && std::string(argv[1]) == "--layers") {
      std::cout << pipe.TotalLayers() << '\n';
      return 0;
    }
    if (argc == 2 && std::string(argv[1]) == "--help") {
      std::cout << "Usage: mesh [--layers]\n"
                   "Configure the circular bent pipe in params3D.h; "
                   "run bash make.sh.\n";
      return 0;
    }
    if (argc != 1)
      throw std::invalid_argument("Unknown arguments; use --help");
    GeneratePipeMesh(pipe, straightMesh, outputMesh);
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "Pipe mesh generation failed: " << error.what() << '\n';
    return 1;
  }
}
