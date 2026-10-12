#ifndef PIPE_MESH_H
#define PIPE_MESH_H

#include "BentPipe.h"
#include <string>

void GeneratePipeMesh(const BentPipe &pipe, const std::string &input,
                      const std::string &output);

#endif
