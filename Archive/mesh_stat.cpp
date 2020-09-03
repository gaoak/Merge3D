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

static double neawallRegion(double x, double y, double z) {
    return 1.;
}

int main(int argc, char* argv[]) {
    vector<double> targz1;
    vector<void*> condition;
    if(argc>1)
    {
        NektarppXml baseMesh(argv[1], "outmesh_", 1E-6);
        baseMesh.LoadXml(0, targz1);
        baseMesh.ReorgDomain(condition);
        baseMesh.UpdateXml();
    }
    return -1;
}
