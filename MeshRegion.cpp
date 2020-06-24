#include"MeshRegion.h"
#include<cmath>
#include<fstream>
#include<iostream>
#include<string>
#include<vector>
#include<map>
#include<set>
MeshRegion::MeshRegion(std::string name, double tolerance)
{
    m_name = name;
    m_tolerance = tolerance;
}

void MeshRegion::sortCellFromQtoT(){
    std::vector<std::vector<int> > tmpcells;
    for(int i=0; i<m_cells.size(); ++i) {
        if(m_cells[i].size()==4) tmpcells.push_back(m_cells[i]);
    }
    for(int i=0; i<m_cells.size(); ++i) {
        if(m_cells[i].size()==3) tmpcells.push_back(m_cells[i]);
    }
    m_cells = tmpcells;
}

void MeshRegion::rebuildEdgesIndex(){
    m_edgesIndex.clear();
    for(int i=0; i<m_edges.size(); ++i) {
        std::set<int> p;
        p.insert(m_edges[i][0]);
        p.insert(m_edges[i][1]);
        m_edgesIndex[p] = i;
    }
}

std::vector<std::vector<int>> MeshRegion::extractBoundary() {
    m_unSharedPts.clear();
    m_unSharedPtsSet.clear();
    std::vector<int> edgeCount(m_edges.size(), 0);
    for(int i=0; i<m_cells.size(); ++i) {
        for(int k=0; k<m_cells[i].size(); ++k) {
            edgeCount[m_cells[i][k]] += 1;
        }
    }
    std::map<int, std::vector<int>> ptsToUnSharedEdges;
    std::set<int> pts;
    for(int i=0; i<edgeCount.size(); ++i) {
        if(1 == edgeCount[i]) {
            ptsToUnSharedEdges[m_edges[i][0]].push_back(i);
            ptsToUnSharedEdges[m_edges[i][1]].push_back(i);
            pts.insert(m_edges[i][0]);
            pts.insert(m_edges[i][1]);
        }else {
            if(edgeCount[i]!=2) std::cout << "Warning: in region " << m_name <<  ", edge " << i << ", times used by cells " << edgeCount[i] << std::endl;
        }
    }
    for(auto it=ptsToUnSharedEdges.begin(); it!=ptsToUnSharedEdges.end(); ++it) {
        if(it->second.size()!=2) {
            std::cout << "Warning: in region" << m_name <<  ", pts " << (it->first) << ", = (" << m_pts[(it->first)][0] << ", " << m_pts[(it->first)][1] << "), on edge " << it->second[0] << "only used once." << std::endl;
        }
    }
    std::cout << "Extract " << pts.size() << " points on the boundary in region " << m_name << std::endl;
    int ptsIndex = -1;
    int nbnds = -1;
    while(pts.size()>0) {
        if(ptsIndex==-1) {
            ptsIndex = *(pts.begin());
            std::vector<int> b0;
            std::set<int>    bs0;
            b0.push_back(ptsIndex);
            bs0.insert(ptsIndex);
            m_unSharedPts.push_back(b0);
            m_unSharedPtsSet.push_back(bs0);
            nbnds++;
            pts.erase(ptsIndex);
        }else {
            int tmpindex;
            if(ptsIndex != m_edges[ptsToUnSharedEdges[ptsIndex][0]][0]) {
                tmpindex = m_edges[ptsToUnSharedEdges[ptsIndex][0]][0];
            }else {
                tmpindex = m_edges[ptsToUnSharedEdges[ptsIndex][0]][1];
            }
            if(m_unSharedPtsSet[nbnds].find(tmpindex)==m_unSharedPtsSet[nbnds].end()) {
                m_unSharedPts[nbnds].push_back(tmpindex);
                m_unSharedPtsSet[nbnds].insert(tmpindex);
                pts.erase(tmpindex);
                ptsIndex = tmpindex;
                continue;
            }
            if(ptsIndex != m_edges[ptsToUnSharedEdges[ptsIndex][1]][0]) {
                tmpindex = m_edges[ptsToUnSharedEdges[ptsIndex][1]][0];
            }else {
                tmpindex = m_edges[ptsToUnSharedEdges[ptsIndex][1]][1];
            }
            if(m_unSharedPtsSet[nbnds].find(tmpindex)==m_unSharedPtsSet[nbnds].end()) {
                m_unSharedPts[nbnds].push_back(tmpindex);
                m_unSharedPtsSet[nbnds].insert(tmpindex);
                pts.erase(tmpindex);
                ptsIndex = tmpindex;
                continue;
            }
            ptsIndex = -1;
        }
    }
    std::cout << "Extract " << m_unSharedPts.size() << " unconnected boundaries in region " << m_name << std::endl;
    return m_unSharedPts;
}

int MeshRegion::pointIsExist(std::vector<double> p, int &pId) {
    for(auto it = m_bndPts.begin(); it!=m_bndPts.end(); ++it) {
        pId = *it;
        if(0.5*(fabs(m_pts[pId][0] - p[0]) + fabs(m_pts[pId][1] - p[1]))<m_tolerance) {
            return 1;
        }
    }
    return 0;
}
