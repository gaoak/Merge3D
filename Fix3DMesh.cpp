/*
merge 3D nektar mesh at Re 400
*/
#include "CAD3D/LineEdge.h"
#include "CAD3D/MeshRegion.h"
#include "CAD3D/Nektarpp.h"
#include "CAD3D/Util.h"
#include "params3D.h"
#include <algorithm>
#include <iostream>
using namespace std;
static double anckerz[4][2];
double offset = 0.1;
static void init() {
  anckerz[0][0] = 0.;
  anckerz[0][1] = 0.;
  anckerz[1][0] = spanlength;
  anckerz[1][1] = 0.;
}

static void setzscale(vector<double> &targz1) {
  int N0 = wingn;
  targz1.clear();
  LineEdge line0(anckerz[0], anckerz[1], N0, BOUNDARYLAYER1, 0, 0, 0, offset,
                 2., 5);
  vector<double> p;
  for (int i = 0; i <= line0.m_N; ++i) {
    double s = i * 2. / line0.m_N - 1.;
    p = line0.Evaluate(s);
    targz1.push_back(p[0]);
  }
  for (int i = 0; i < targz1.size(); ++i)
    cout << targz1[i] << ", ";
  cout << endl;
  cout << "input c for continue " << endl;
  char c = 0;
  cin >> c;
  if (c != 'c') {
    cout << "end generating." << endl;
    exit(-1);
  }
}

static double neawallRegion(double x, double y, double z) { return 1; }

void movetip(double *p) { p[2] += spanlength; }

int main() {
  init();
  vector<double> targz1;
  setzscale(targz1);

  vector<double> targz0;
  NektarppXml tipMesh("tip.xml", "Tip:", 1E-6);
  tipMesh.LoadWallmapping("wallmapping.dat");
  tipMesh.LoadXml(1, targz0, 0., 0., true, -1);
  tipMesh.DeformPts((void *)movetip);

  NektarppXml rootMesh("root3D.xml", "Root", 1E-6);
  rootMesh.LoadXml(targz1.size() - 1, targz1);

  tipMesh.AddMeshRegion(rootMesh);
  while (tipMesh.ReorgBoundary(M_PI * 0.499))
    ;
  vector<void *> condition;
  tipMesh.ReorgDomain(condition, false);
  tipMesh.UpdateXml();
  tipMesh.CheckMesh();
  tipMesh.OutXml("test.xml");
  return 0;
}
