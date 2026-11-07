/*
merge 3D nektar mesh at Re 400
*/
#include "CAD3D/LineEdge.h"
#include "CAD3D/MeshRegion.h"
#include "CAD3D/Nektarpp.h"
#include "CAD3D/Util.h"
#include "TrapezoidWing.h"
#include "params3D.h"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
using namespace std;
static double anckerz[4][2];
double offset = tipoffset;
static void init() {
  if (!std::isfinite(tiphzgrowth) || tiphzgrowth < 1.)
    throw invalid_argument("tiphzgrowth must be finite and >= 1");
  anckerz[0][0] = 0.;
  anckerz[0][1] = 0.;
  anckerz[1][0] = spanlength;
  anckerz[1][1] = 0.;
  anckerz[2][0] = spanlength + offset;
  anckerz[2][1] = 0.;
  anckerz[3][0] = domainz;
  anckerz[3][1] = 0.;
}

static void setzscale(vector<double> &targz1, vector<double> &targz2,
                      vector<double> &targz3, bool confirm) {
  int N0 = wingn;
  int N1 = tipn;
  targz1.clear();
  targz2.clear();
  targz3.clear();
  LineEdge line0(anckerz[0], anckerz[1], N0, BOUNDARYLAYER1, 0, 0, 0, hFirstLayer,
                 2., 5);
  LineEdge line1(anckerz[2], anckerz[3], N1, BOUNDARYLAYER0,
                 tiphzgrowth * offset, tiphzgrowth, 7,
                 0, 0, 0);
  vector<double> p;
  for (int i = 0; i <= line0.m_N; ++i) {
    double s = i * 2. / line0.m_N - 1.;
    p = line0.Evaluate(s);
    targz1.push_back(p[0]);
  }
  targz3.push_back(anckerz[1][0]);
  for (int i = 0; i <= line1.m_N; ++i) {
    double s = i * 2. / line1.m_N - 1.;
    p = line1.Evaluate(s);
    targz2.push_back(p[0]);
    targz3.push_back(p[0]);
  }
  for (int i = 0; i < targz1.size(); ++i)
    cout << targz1[i] << ", ";
  cout << endl;
  for (int i = 0; i < targz2.size(); ++i)
    cout << targz2[i] << ", ";
  cout << endl;
  for (int i = 0; i < targz3.size(); ++i)
    cout << targz3[i] << ", ";
  cout << endl;
  if (!confirm)
    return;
  cout << "input c for continue " << endl;
  char c = 0;
  cin >> c;
  if (c != 'c') {
    cout << "end generating." << endl;
    exit(-1);
  }
}

static double neawallRegion(double x, double y, double z) { return 1; }

static double detectSingular(double x, double y, double z) {
  if (z > spanlength + 0.1 || z < spanlength - 0.1)
    return -1.;
  return 10 - x * x - y * y;
}

static const TrapezoidWing &wingTransform() {
  static const TrapezoidWing mapping(
      sourceChord, rootChord, tipChord, tipLEOffset, spanlength, foilAngleDegrees,
      leadingEdgeRadius, trailingEdgeRadius, taperNearNormal, taperFarNormal,
      taperNearUpstream, taperFarUpstream, taperNearDownstream, taperFarDownstream);
  return mapping;
}

static void taperWing(double *p) { wingTransform().Apply(p); }

static int generateLshape(const string &output, bool confirm) {
  const int layers = 32, maxBoundaryLayers = 5;
  const double length = 0.4, firstHeight = 0.003, growth = 1.6;
  // Stop before a geometric layer would exceed the remaining uniform spacing.
  int boundaryLayers = 0;
  double boundaryWidth = 0., height = firstHeight;
  while (boundaryLayers < maxBoundaryLayers) {
    const int coreLayers = layers - 2 * (boundaryLayers + 1);
    const double coreWidth = length - 2 * (boundaryWidth + height);
    if (coreLayers <= 0 || coreWidth <= 0. || height > coreWidth / coreLayers)
      break;
    boundaryWidth += height;
    height *= growth;
    ++boundaryLayers;
  }
  vector<double> target(layers + 1, 0.);
  height = firstHeight;
  for (int i = 1; i <= boundaryLayers; ++i) {
    target[i] = target[i - 1] + height;
    target[layers - i] = length - target[i];
    height *= growth;
  }
  const double coreHeight =
      (length - 2 * boundaryWidth) / (layers - 2 * boundaryLayers);
  for (int i = boundaryLayers + 1; i < layers - boundaryLayers; ++i)
    target[i] = boundaryWidth + (i - boundaryLayers) * coreHeight;
  target[layers] = length;
  cout << "L-shape: " << layers << " layers, z=[0, " << length << "], "
       << boundaryLayers << " boundary layers at each end, core spacing="
       << setprecision(12) << coreHeight << "\nz coordinates:\n";
  for (double z : target)
    cout << z << ' ';
  cout << endl;
  if (confirm) {
    cout << "input c for continue " << endl;
    char c = 0;
    cin >> c;
    if (c != 'c')
      throw runtime_error("Mesh generation cancelled");
  }

  NektarppXml mesh("InputMesh/lshape_3d_uniform.xml", "lshape_");
  auto geometry = mesh.m_doc.FirstChildElement("NEKTAR");
  geometry = geometry ? geometry->FirstChildElement("GEOMETRY") : nullptr;
  auto vertices = geometry ? geometry->FirstChildElement("VERTEX") : nullptr;
  if (!vertices || geometry->IntAttribute("DIM") != 3)
    throw runtime_error("Expected a 3D extruded mesh; run InputMesh/extrude.sh");
  set<int> levels;
  for (auto v = vertices->FirstChildElement(); v; v = v->NextSiblingElement()) {
    double x, y, z;
    istringstream coords(v->GetText() ? v->GetText() : "");
    if (!(coords >> x >> y >> z) || !isfinite(z) || z < 0. || z > 1. ||
        fabs(z * layers - round(z * layers)) > 1e-8)
      throw runtime_error("Expected 32 uniform unit-length layers");
    levels.insert(static_cast<int>(round(z * layers)));
  }
  if (levels.size() != layers + 1)
    throw runtime_error("Expected 33 z planes; rerun InputMesh/extrude.sh");
  // LoadXml remaps vertices and curved points directly in the XML document.
  // No topology/composite/domain reorganization is needed for this extrusion.
  mesh.LoadXml(layers, target);
  mesh.CheckMesh();
  mesh.OutXml(output);
  return 0;
}

static int generateMesh(int argc, char *argv[]) {
  bool confirm = true, deformed = false, lshape = false;
  string output;
  for (int i = 1; i < argc; ++i) {
    const string arg = argv[i];
    if (arg == "--yes")
      confirm = false;
    else if (arg == "--lshape")
      lshape = true;
    else if (arg == "--deformed")
      deformed = true;
    else if (arg == "--rectangular")
      deformed = false;
    else if (arg == "--output" && i + 1 < argc)
      output = argv[++i];
    else if (arg == "--help") {
      cout << "Usage: mesh [--yes] [--lshape | --deformed | --rectangular] "
              "[--output mesh.xml]\n"
              "Default: rectangular mesh (test.xml). "
              "--deformed: trapezoid mesh (deformed.xml).\n"
              "--lshape: 32-layer L-shape mesh (lshape_3d.xml).\n";
      return 0;
    } else
      throw invalid_argument("Unknown or incomplete option: " + arg);
  }
  if (output.empty())
    output = lshape ? "lshape_3d.xml" : (deformed ? "deformed.xml" : "test.xml");
  if (lshape)
    return generateLshape(output, confirm);
  if (deformed)
    cout << "Trapezoid chord: root=" << rootChord << ", tip=" << tipChord
         << ", tip leading-edge offset=" << tipLEOffset
         << ", continuous mapping Jacobian >= "
         << wingTransform().JacobianLowerBound() << endl;
  init();
  vector<double> targz1, targz2, targz3;
  setzscale(targz1, targz2, targz3, confirm);
  vector<void *> condition;
  condition.push_back((void *)detectSingular);
  condition.push_back((void *)neawallRegion);

  NektarppXml baseMesh("InputMesh/3DoutUcomp.xml", "outmesh_", 1E-6);
  baseMesh.LoadWallmapping("InputMesh/wallmapping.dat");
  baseMesh.LoadXml(targz1.size() - 1, targz1, 0., offset);
  // baseMesh.OutXml("testouter1.xml");

  NektarppXml baseMesh2("InputMesh/3DoutUcomp2.xml", "outmesh_", 1E-6);
  baseMesh2.LoadWallmapping("InputMesh/wallmapping.dat");
  baseMesh2.LoadXml(targz2.size() - 1, targz2, 0., 0.);
  baseMesh2.OutXml(deformed ? "deformedouter2.xml" : "testouter2.xml");

  NektarppXml innerMesh("InputMesh/3DinUcomp.xml", "outmesh_", 1E-6);
  innerMesh.LoadWallmapping("InputMesh/wallmapping.dat");
  innerMesh.LoadXml(targz3.size() - 1, targz3, 0., 0., true, 0);
  // innerMesh.OutXml("testinner.xml");

  baseMesh.AddMeshRegion(baseMesh2);
  baseMesh.AddMeshRegion(innerMesh);
  while (baseMesh.ReorgBoundary(M_PI * 0.499))
    ;
  baseMesh.ReorgDomain(condition, true);
  // Deform the assembled mesh once, including all CURVED interpolation points.
  // Organize boundaries beforehand to retain the existing boundary IDs.
  if (deformed)
    baseMesh.DeformPts((void *)taperWing);
  baseMesh.UpdateXml();
  baseMesh.CheckMesh();
  baseMesh.OutXml(output);

  return 0;
}

int main(int argc, char *argv[]) {
  try {
    return generateMesh(argc, argv);
  } catch (const exception &error) {
    cerr << "Mesh generation failed: " << error.what() << endl;
    return 1;
  }
}
