#ifndef PIPE_PARAMETERS_H
#define PIPE_PARAMETERS_H
#include "BentPipe.h"

// NekMesh extrude:layers; the plane count is extrudeLayers + 1.
constexpr int extrudeLayers = 128;
constexpr const char *straightMesh = "InputMesh/pipeStraight.xml";
constexpr const char *outputMesh = "pipe.xml";

// Arc centered at s=0, bent in the x-z plane; the tangent at s=0 is +z.
// Cross-sections are circles; their radius is read from the input mesh.
inline PipeParameters pipeParameters() {
  PipeParameters p;
  p.bendAngleDegrees = 90.0; // Total turning angle, 0..180 degrees.
  p.bendRadius = 2.0;       // Must exceed the actual pipe radius.
  p.inletExtension = 2.0;   // Straight tangent length before the arc.
  p.outletExtension = 2.0;  // Straight tangent length after the arc.
  p.centerSpacing = 0.02;   // First cell length on BOTH sides of s=0.
  p.growthRate = 1.08;      // Exponential growth, then an equal-spacing tail.
  p.inletLayers = 64;       // Outlet layers = actual source planes - 1 - this.
  p.geometryPoints = 6;     // At least the input circle's curve point count.
  return p;
}
#endif
