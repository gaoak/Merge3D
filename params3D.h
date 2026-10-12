#ifndef PIPE_PARAMETERS_H
#define PIPE_PARAMETERS_H
#include "BentPipe.h"

// NekMesh extrusion layers = inletLayers + bendLayers + outletLayers.
constexpr const char *straightMesh = "InputMesh/pipeStraight.xml";
constexpr const char *outputMesh = "pipe.xml";

// Bend entry is (0,0,0), inlet tangent is +x; bend turns towards +z.
// Cross-sections are circles; their radius is read from the input mesh.
inline PipeParameters pipeParameters() {
  PipeParameters p;
  p.bendAngleDegrees = 90.0; // Total turning angle, 0 < angle <= 180 degrees.
  p.bendRadius = 2.0;       // Must exceed the actual pipe radius.
  p.inletExtension = 10.0;   // Straight tangent length before the arc.
  p.outletExtension = 20.0;  // Straight tangent length after the arc.
  p.inletLayers = 25;        // Specified inlet straight cell count.
  p.bendLayers = 16;         // Uniform bend cells; spacing = R * angle / count.
  p.outletLayers = 50;       // Specified outlet straight cell count.
  p.inletGrowthRate = 1.06;  // Growth from bend entry towards inlet.
  p.outletGrowthRate = 1.06; // Growth from bend exit towards outlet.
  p.geometryPoints = 6;     // At least the input circle's curve point count.
  return p;
}
#endif
