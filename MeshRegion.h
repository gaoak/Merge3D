#ifndef MESHREGION_H
#define MESHREGION_H
#include<vector>
#include<set>
#include<map>
#include<string>
#define TRIGELEMENT 2
#define QUADELEMENT 3
#define LINEBUFFERSIZE 1000
//only topology structures
//no physics
class MeshRegion {
public:
    MeshRegion(std::string name, double tolerance = 1.E-6);
    double m_tolerance;
    std::string m_name;
    std::vector<std::vector<double> > m_pts;
    std::set<int> m_bndPts;
    std::vector<std::vector<int> > m_edges;
    std::vector<std::vector<int> > m_faces;
    std::vector<std::vector<int> > m_cells;
    void sortCells();
    std::vector<std::vector<int>> extractBoundary();
    int pointIsExist(std::vector<double> p, int &pId);
    void rebuildEdgesIndex();
    void rebuildFacesIndex();
    
    std::map<std::set<int>, int> m_edgesIndex;
    std::map<std::set<int>, int> m_facesIndex;
private:
};
#endif // MESHREGION_H
