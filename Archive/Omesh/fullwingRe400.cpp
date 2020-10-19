/*
merge 3D nektar mesh at Re 400
*/
#include"MeshRegion.h"
#include"Nektarpp.h"
#include"LineEdge.h"
#include"params.h"
#include"Util.h"
#include<iostream>
#include<algorithm>
using namespace std;
double spanlength = 5.;
double domainz = 7.;
static double anckerz[4][2];
static double anckerz_neg[4][2];
double offset = hFirstLayer;
static void init() {
    anckerz_neg[0][0] = 0.;                   anckerz_neg[0][1] = 0.;
    anckerz_neg[1][0] = -spanlength;          anckerz_neg[1][1] = 0.;
    anckerz_neg[2][0] = -spanlength - offset; anckerz_neg[2][1] = 0.;
    anckerz_neg[3][0] = -domainz;             anckerz_neg[3][1] = 0.;
    anckerz[0][0] = 0.;                  anckerz[0][1] = 0.;
    anckerz[1][0] = spanlength;          anckerz[1][1] = 0.;
    anckerz[2][0] = spanlength + offset; anckerz[2][1] = 0.;
    anckerz[3][0] = domainz;             anckerz[3][1] = 0.;
}

static void setzscale(vector<double> &targz1, vector<double> &targz2, vector<double> &targz3) {
    int N0 = 40;
    int N1 = 9;
    targz1.clear();
    targz2.clear();
    targz3.clear();
    LineEdge line0(anckerz[0], anckerz[1], N0, BOUNDARYLAYER1, 0, 0, 0, offset, 2., 5);
    LineEdge line1(anckerz[2], anckerz[3], N1, BOUNDARYLAYER0, 2.*offset, 2., 7, 0, 0, 0);
    vector<double> p;
    for(int i=0; i<=line0.m_N; ++i) {
        double s = i*2./line0.m_N-1.;
        p = line0.Evaluate(s);
        targz1.push_back(p[0]);
    }
    targz3.push_back(anckerz[1][0]);
    for(int i=0; i<=line1.m_N; ++i) {
        double s = i*2./line1.m_N-1.;
        p = line1.Evaluate(s);
        targz2.push_back(p[0]);
        targz3.push_back(p[0]);
    }
    for(int i=0; i<targz1.size(); ++i) cout << targz1[i] << ", ";
    cout << endl;
    for(int i=0; i<targz2.size(); ++i) cout << targz2[i] << ", ";
    cout << endl;
    for(int i=0; i<targz3.size(); ++i) cout << targz3[i] << ", ";
    cout << endl;
    cout << "input c for continue " << endl;
    char c = 0;
    cin >> c;
    if(c!='c') {
        cout << "end generating." << endl;
        exit(-1);
    }
}

static double neawallRegion(double x, double y, double z) {
    return 1;
}

static double detectSingular(double x, double y, double z) {
    if(z>spanlength+0.1 || z<spanlength-0.1) return -1.;
    return 10 - x*x - y*y;
}

int main() {
    init();
    vector<double> targz1, targz2, targz3;
    vector<double> targz1_n, targz2_n, targz3_n;
    setzscale(targz1, targz2, targz3);
    for(int i=targz1.size()-1; i>=0; --i) {
        targz1_n.push_back(-targz1[i]);
    }
    for(int i=targz2.size()-1; i>=0; --i) {
        targz2_n.push_back(-targz2[i]);
    }
    for(int i=targz3.size()-1; i>=0; --i) {
        targz3_n.push_back(-targz3[i]);
    }
    vector<void*> condition;
    condition.push_back((void *)detectSingular);
    condition.push_back((void *)neawallRegion);

    NektarppXml mesh2D("../Mesh2D/outerRegion.xml", "2Doutmesh_", 1E-6);
    mesh2D.LoadXml(targz2.size()-1, targz2);
    mesh2D.ReorgDomain(condition, false);
    mesh2D.UpdateXml();
    //mesh2D.OutXml("test2D.xml");

    NektarppXml baseMesh("../Mesh2D/3DoutUcomp.xml", "outmesh_", 1E-6);
    baseMesh.LoadWallmapping("wallmapping.dat");
    baseMesh.LoadXml(targz1.size()-1, targz1, 0., offset);
    NektarppXml baseMesh_negative("../Mesh2D/3DoutUcomp.xml", "outmesh_neg", 1E-6);
    baseMesh_negative.LoadWallmapping("wallmapping.dat");
    baseMesh_negative.LoadXml(targz1_n.size()-1, targz1_n, offset, 0.);
    //baseMesh.OutXml("testouter1.xml");

    NektarppXml baseMesh2("../Mesh2D/3DoutUcomp2.xml", "outmesh_", 1E-6);
    baseMesh2.LoadWallmapping("wallmapping.dat");
    baseMesh2.LoadXml(targz2.size()-1, targz2, 0., 0.);
    baseMesh2.OutXml("testouter2.xml");
    NektarppXml baseMesh2_negative("../Mesh2D/3DoutUcomp2.xml", "outmesh_", 1E-6);
    baseMesh2_negative.LoadWallmapping("wallmapping.dat");
    baseMesh2_negative.LoadXml(targz2_n.size()-1, targz2_n, 0., 0.);
    baseMesh2_negative.OutXml("testouter2.xml");

    NektarppXml innerMesh("../Mesh2D/3DinUcomp.xml", "outmesh_", 1E-6);
    innerMesh.LoadWallmapping("wallmapping.dat");
    innerMesh.LoadXml(targz3.size()-1, targz3, 0., 0., true, 0);
    NektarppXml innerMesh_negative("../Mesh2D/3DinUcomp.xml", "outmesh_", 1E-6);
    innerMesh_negative.LoadWallmapping("wallmapping.dat");
    innerMesh_negative.LoadXml(targz3_n.size()-1, targz3_n, 0., 0., true, targz3_n.size()-1);
    //innerMesh.OutXml("testinner.xml");

    baseMesh.AddMeshRegion(baseMesh_negative);
    baseMesh.AddMeshRegion(baseMesh2_negative);
    baseMesh.AddMeshRegion(baseMesh2);
    baseMesh.AddMeshRegion(innerMesh);
    baseMesh.AddMeshRegion(innerMesh_negative);
    baseMesh.ReorgBoundary();
    baseMesh.ReorgDomain(condition, true);
    baseMesh.UpdateXml();
    baseMesh.CheckMesh();
    baseMesh.OutXml("test.xml");

    return 0;
}
