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
bool silent = false;
static double anckerz[4][2];
static void init() {
  anckerz[0][0] = 0.;
  anckerz[0][1] = 0.;
  anckerz[1][0] = spanlength;
  anckerz[1][1] = 0.;
}

static void setzscale(vector<double> &targz1) {
  int N0 = wingn;
  targz1.clear();
  LineEdge line0(anckerz[0], anckerz[1], N0, BOUNDARYLAYER1, 0, 0, 0, tipoffset,
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
  if (!silent) {
    cout << "input c for continue " << endl;
    char c = 0;
    cin >> c;
    if (c != 'c') {
      cout << "end generating." << endl;
      exit(-1);
    }
  }
}

static double neawallRegion(double x, double y, double z) { return 1; }

void movetip(double *p) { p[2] += spanlength; }

/* usage
1, Extract root surface, root.xml:xml:uncompress
2, Fix root surface, ConvertXml root.xml root.xml
3, Extrude root surface following params3D.h -> wingn, as
root3D.xml:xml:uncompress 4, Save wing tip mesh as tip.xml:xml:uncompress 5,
Concatenate tip.xml and root3D.xml to gmsh, test.msh 6, Convert gmsh file to
Nektar xml, NekMesh test.msh test2.xml 7, Fix concatenated mesh, Fix3DMesh gmsh
test2.xml 8, The final mesh fixed.xml is correct
*/

int main(int argc, char *argv[]) {
  bool gmshmap = false;
  std::string sortedfile;
  for (int i = 1; i < argc; ++i) {
    if (strcmp(argv[i], "gmsh") == 0) {
      gmshmap = true;
      sortedfile = argv[i + 1];
    }
    if (strcmp(argv[i], "silent") == 0) {
      silent = true;
    }
  }
  init();
  vector<double> targz1;
  setzscale(targz1);

  vector<double> targz0;
  NektarppXml tipMesh("tip.xml", "Tip:", 1E-5);
  tipMesh.LoadWallmapping("wallmapping.dat");
  tipMesh.LoadXml(1, targz0, 0., 0., true, -1);
  tipMesh.DeformPts((void *)movetip);
  tipMesh.UpdateXml();

  NektarppXml rootMesh("root3D.xml", "Root", 1E-4);
  rootMesh.LoadXml(targz1.size() - 1, targz1);

  rootMesh.AddMeshRegion(tipMesh);

  if (!gmshmap) {
    rootMesh.OutGmsh("test.msh");
  } else {
    rootMesh.RemapMesh(sortedfile);
    while (rootMesh.ReorgBoundary(M_PI * 0.499))
      ;
    vector<void *> condition;
    condition.push_back((void *)neawallRegion);
    rootMesh.ReorgDomain(condition, true);
    rootMesh.UpdateXml();
    rootMesh.CheckMesh();
    rootMesh.OutXml("fixed.xml");
  }
  return 0;
}
