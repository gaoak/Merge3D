#include"MeshRegion.h"
#include"Nektarpp.h"
#include"LineEdge.h"
#include"params.h"
#include"Util.h"
#include<iostream>
using namespace std;
double spanlength = 5.;
double domainz = 7.;
static double anckerz[4][2];
static void init() {
    anckerz[0][0] = 0.;         anckerz[0][1] = 0.;
    anckerz[1][0] = spanlength - 2.*chordLen; anckerz[1][1] = 0.;
    anckerz[2][0] = spanlength; anckerz[2][1] = 0.;
    anckerz[3][0] = domainz;    anckerz[3][1] = 0.;
}

static void setzscale(vector<double> &targz1, vector<double> &targz2) {
    int N0 = 10;
    int N1 = 15;
    int N2 = 5;
    LineEdge line0(anckerz[0], anckerz[1], N0, UNIFORM,    0., 0.);
    LineEdge line1(anckerz[1], anckerz[2], N1, QUDREFINE1, 0., hFirstLayer);
    LineEdge line2(anckerz[2], anckerz[3], N2, EXPREFINE0, hFirstLayer, 0.);
    vector<double> p;
    for(int i=0; i<line0.m_N; ++i) {
        double s = i*2./line0.m_N-1.;
        p = line0.Evaluate(s);
        targz1.push_back(p[0]);
    }
    for(int i=0; i<line1.m_N; ++i) {
        double s = i*2./line1.m_N-1.;
        p = line1.Evaluate(s);
        targz1.push_back(p[0]);
    }
    for(int i=0; i<=line2.m_N; ++i) {
        double s = i*2./line2.m_N-1.;
        p = line2.Evaluate(s);
        targz1.push_back(p[0]);
        targz2.push_back(p[0]);
    }
    for(int i=0; i<targz1.size(); ++i) cout << targz1[i] << ", ";
    cout << endl;
    for(int i=0; i<targz2.size(); ++i) cout << targz2[i] << ", ";
    cout << endl;
}

static double neawallRegion(double x, double y, double z) {
    vector<double> p(3);
    p[0] = x; p[1] = y; p[2] = z;
    transform(p, -AoA);
    double radius = rBoundaryLayer + Thickness*0.5;
    double endz = spanlength + rBoundaryLayer;
    double res = radius*radius - (x*x + y*y);
    if(res>=0 && z <= endz && z>=-endz) return 1.;
    double xend = chordLen + wakeLen;
    if(p[0]>=0. && p[0]<=xend && p[1]>=-radius && p[1]<=radius && z <= endz && z>=-endz) return 1.;
    else return -1;
}

static double wakeRegion(double x, double y, double z) {
    double endz = spanlength + rBoundaryLayer;
    double wakexs = chordLen + wakeLen;
    double ytop = farWakeUp   + (x-wakexs)*tan(wakeDiffuseAngle-farWakeAoA);
    double ybot = farWakeDown - (x-wakexs)*tan(wakeDiffuseAngle+farWakeAoA);
    if(x>=0. && x<=farWakeRight && y>=ybot && y<=ytop && z <= endz && z>=-endz) return 1.;
    else return -1;
}

int main() {
    init();
    NektarppXml baseMesh("../Mesh2D/3DoutUcomp.xml", "outmesh_", 1E-6);
    NektarppXml innerMesh("../Mesh2D/3DinUcomp.xml", "innermesh_", 1E-6);
    vector<double> targz1;
    vector<double> targz2;
    setzscale(targz1, targz2);
    baseMesh.LoadXml(targz1.size()-1, targz1);
    innerMesh.LoadXml(targz2.size()-1, targz2);
    
    baseMesh.AddMeshRegion(innerMesh);
    vector<void*> condition;
    condition.push_back((void *)neawallRegion);
    condition.push_back((void *)wakeRegion);
    baseMesh.ReorgDomain(condition);

    baseMesh.UpdateXml();
    baseMesh.OutXml("test.xml");

    //vector<double> center={0., 0., 0.};
    //vector<vector<double> > centers;
    //centers.push_back(center);
    //vector<double> radius = {100.};
    //innerMesh.OutCompAsGeo("test.geo", centers, radius);
}
