#include "CAD3D/MeshRegion.h"
#include "CAD3D/Nektarpp.h"

using namespace std;

int main() {
    NektarppXml baseMesh("outmesh_", 1E-6);
    NektarppXml innerMesh("innermesh_", 1E-6);
    vector<double> targz1 = {0., 1., 2.};
    vector<double> targz2 = {1., 2.};
    baseMesh.LoadXml("3DoutUcomp.xml", 3, targz1);
    innerMesh.LoadXml("3DinUcomp.xml", 1, targz2);
    baseMesh.AddXml(innerMesh);
    baseMesh.OutXml("test.xml");
}
