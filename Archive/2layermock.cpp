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
double spanlength = 0.1;
double domainz = 0.2;

static void setzscale(vector<double> &targz1, vector<double> &targz2) {
    targz1.push_back(0);
    targz1.push_back(spanlength);
    targz2.push_back(spanlength);
    targz1.push_back(domainz);
    targz2.push_back(domainz);
    for(int i=0; i<targz1.size(); ++i) cout << targz1[i] << ", ";
    cout << endl;
    for(int i=0; i<targz2.size(); ++i) cout << targz2[i] << ", ";
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

int main() {
    vector<double> targz1;
    vector<double> targz2;
    setzscale(targz1, targz2);
    vector<void*> condition;
    condition.push_back((void *)neawallRegion);

    NektarppXml mesh2D("../Mesh2D/outerRegion.xml", "2Doutmesh_", 1E-6);
    mesh2D.LoadXml(targz2.size()-1, targz2);
    mesh2D.ReorgDomain(condition);
    mesh2D.UpdateXml();
    mesh2D.OutXml("test2D.xml");

    NektarppXml baseMesh("../Mesh2D/3DoutUcomp.xml", "outmesh_", 1E-6);
    NektarppXml innerMesh("../Mesh2D/3DinUcomp.xml", "innermesh_", 1E-6);
    baseMesh.LoadXml(targz1.size()-1, targz1);
    innerMesh.LoadXml(targz2.size()-1, targz2);
    baseMesh.AddMeshRegion(innerMesh);
    baseMesh.ReorgDomain(condition);
    baseMesh.UpdateXml();
    baseMesh.OutXml("test.xml");

    return 0;
}
