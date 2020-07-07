#include"MeshRegion.h"
#include"Nektarpp.h"
#include"LineEdge.h"
#include<iostream>
using namespace std;
int N1 = 22;
int N2 = 8;
double p[3][2];

LineEdge line1(p[0], p[1], N1, QUDREFINE1, 0., 0.02);
LineEdge line2(p[1], p[2], N2, EXPREFINE0, 0.02, 0.);

int main() {
    NektarppXml baseMesh("outmesh_", 1E-6);
    NektarppXml innerMesh("innermesh_", 1E-6);
    vector<double> targz1;
    vector<double> targz2;
    p[0][0] = 0.; p[0][1] = 0.;
    p[1][0] = 5.; p[1][1] = 0.;
    p[2][0] = 7.; p[2][1] = 0.;
    vector<double> p;
    for(int i=0; i<N1; ++i) {
        double s = i*2./N1-1.;
        p = line1.Evaluate(s);
        targz1.push_back(p[0]);
    }
    for(int i=0; i<=N2; ++i) {
        double s = i*2./N2-1.;
        p = line2.Evaluate(s);
        targz1.push_back(p[0]);
        targz2.push_back(p[0]);
    }
    for(int i=0; i<targz1.size(); ++i) cout << targz1[i] << ", ";
    cout << endl;
    for(int i=0; i<targz2.size(); ++i) cout << targz2[i] << ", ";
    cout << endl;
    baseMesh.LoadXml("3DoutUcomp.xml", N1+N2, targz1);
    innerMesh.LoadXml("3DinUcomp.xml", N2, targz2);
    baseMesh.AddMeshRegion(innerMesh);
    baseMesh.OutXml("test.xml");
}
