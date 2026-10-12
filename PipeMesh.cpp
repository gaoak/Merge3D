#include "PipeMesh.h"
#include "CAD3D/Nektarpp.h"
#include "CAD3D/Util.h"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <sstream>

using Point = BentPipe::Point;
using namespace tinyxml2;

namespace {
struct SourceSection {
  std::vector<double> planes;
  double x, y, radius;
};

std::vector<int> FaceVertices(NektarppXml &mesh, int id) {
  std::vector<int> result;
  char type;
  mesh.GetFacePts(id, result, type);
  return result;
}

SourceSection PrepareSource(NektarppXml &mesh) {
  if (mesh.m_dim != 3)
    throw std::runtime_error("Input must be an extruded 3D mesh; run make.sh first");
  std::set<double> z;
  for (const auto &point : mesh.m_pts)
    z.insert(point.second[2]);
  SourceSection source;
  source.planes.assign(z.begin(), z.end());
  if (source.planes.size() < 3)
    throw std::runtime_error("Need at least two extrusion layers");
  const double tolerance = 1e-7;
  std::map<int, int> usage;
  for (const auto &cell : mesh.m_cells)
    for (int face : cell.second)
      ++usage[face];
  mesh.m_bndComposite.clear();
  std::set<int> wallVertices;
  for (const auto &face : mesh.m_faces) {
    if (usage[face.first] == 2)
      continue;
    if (usage[face.first] != 1)
      throw std::runtime_error("Source has an unused or non-manifold face");
    const auto vertices = FaceVertices(mesh, face.first);
    bool inlet = true, outlet = true;
    for (int id : vertices) {
      const double pz = mesh.m_pts.at(id)[2];
      inlet &= std::fabs(pz - source.planes.front()) < tolerance;
      outlet &= std::fabs(pz - source.planes.back()) < tolerance;
    }
    const int boundary = inlet ? 1 : outlet ? 2 : 0;
    mesh.m_bndComposite[boundary].insert(face.first);
    if (boundary == 0)
      wallVertices.insert(vertices.begin(), vertices.end());
  }
  if (mesh.m_bndComposite.size() != 3 || wallVertices.empty())
    throw std::runtime_error("Expected wall, inlet and outlet in the source cylinder");
  double xmin = 1e100, xmax = -1e100, ymin = 1e100, ymax = -1e100;
  for (int id : wallVertices) {
    const auto &p = mesh.m_pts.at(id);
    xmin = std::min(xmin, p[0]); xmax = std::max(xmax, p[0]);
    ymin = std::min(ymin, p[1]); ymax = std::max(ymax, p[1]);
  }
  source.x = (xmin + xmax) / 2.;
  source.y = (ymin + ymax) / 2.;
  // A complete circle is required. Curve nodes refine the radius estimate
  // when the polygon does not contain its extrema.
  double radiusSum = 0.;
  std::size_t radiusCount = 0;
  auto geometry = mesh.m_doc.FirstChildElement("NEKTAR")->FirstChildElement("GEOMETRY");
  auto curves = geometry->FirstChildElement("CURVED");
  if (curves)
    for (auto node = curves->FirstChildElement(); node;
         node = node->NextSiblingElement()) {
      if (std::string(node->Name()) != "E")
        throw std::runtime_error("Expected the straight extrusion to contain edge curves only");
      const int eid = node->IntAttribute("EDGEID");
      const auto &edge = mesh.m_edges.at(eid);
      if (!wallVertices.count(edge[0]) || !wallVertices.count(edge[1]))
        throw std::runtime_error("Only circular wall curves are supported in the source section");
      std::vector<double> values;
      parserDouble(node->GetText(), values);
      for (std::size_t i = 0; i < values.size(); i += 3) {
        radiusSum += std::hypot(values[i] - source.x, values[i + 1] - source.y);
        ++radiusCount;
      }
    }
  if (radiusCount)
    source.radius = radiusSum / radiusCount;
  else
    source.radius = (xmax - xmin + ymax - ymin) / 4.;
  for (int id : wallVertices) {
    const auto &p = mesh.m_pts.at(id);
    if (std::fabs(std::hypot(p[0] - source.x, p[1] - source.y) - source.radius) >
        tolerance * std::max(1., source.radius))
      throw std::runtime_error("Source wall is not a circle centered on the extrusion axis");
  }

  // Every plane must contain exactly the same two-dimensional section.
  std::vector<std::vector<std::pair<double, double>>> sections(source.planes.size());
  for (const auto &point : mesh.m_pts) {
    const auto &p = point.second;
    if (std::hypot(p[0] - source.x, p[1] - source.y) >
        source.radius + tolerance * std::max(1., source.radius))
      throw std::runtime_error("Source vertex lies outside the circular section");
    const auto index = std::lower_bound(source.planes.begin(), source.planes.end(), p[2])
                       - source.planes.begin();
    sections[index].emplace_back(p[0], p[1]);
  }
  for (auto &section : sections)
    std::sort(section.begin(), section.end());
  for (const auto &section : sections) {
    if (section.size() != sections.front().size())
      throw std::runtime_error("Source planes contain different vertex counts");
    for (std::size_t i = 0; i < section.size(); ++i)
      if (std::fabs(section[i].first - sections.front()[i].first) > tolerance ||
          std::fabs(section[i].second - sections.front()[i].second) > tolerance)
        throw std::runtime_error("Source sections do not match along the extrusion axis");
  }

  // Keep boundary composites separate from the volume composites.
  mesh.m_domain.clear();
  mesh.m_domainType.clear();
  std::map<char, int> domains;
  for (const auto &cell : mesh.m_cellsType) {
    if (cell.second != 'H' && cell.second != 'R')
      throw std::runtime_error("Only extruded quadrilateral/triangle cells are supported");
    if (!domains.count(cell.second)) {
      const int id = 100 + static_cast<int>(domains.size());
      domains[cell.second] = id;
      mesh.m_domainType[id] = cell.second;
    }
    mesh.m_domain[domains[cell.second]].insert(cell.first);
  }
  return source;
}

void AppendCurve(NektarppXml &mesh, XMLElement *root, int &curveId, char tag,
                 int entityId, const std::vector<Point> &points) {
  const std::string name(1, tag);
  auto node = mesh.m_doc.NewElement(name.c_str());
  node->SetAttribute("ID", curveId++);
  node->SetAttribute(tag == 'E' ? "EDGEID" : "FACEID", entityId);
  node->SetAttribute("NUMPOINTS", static_cast<int>(points.size()));
  node->SetAttribute("TYPE", "PolyEvenlySpaced");
  std::ostringstream text;
  text << std::scientific << std::setprecision(17);
  for (const auto &p : points)
    for (double value : p)
      text << value << ' ';
  node->SetText(text.str().c_str());
  root->InsertEndChild(node);
}

void PrepareCurvedGeometry(NektarppXml &mesh, const BentPipe &pipe,
                          const SourceSection &source, const PipeStations &axis) {
  auto geometry = mesh.m_doc.FirstChildElement("NEKTAR")->FirstChildElement("GEOMETRY");
  auto curves = geometry->FirstChildElement("CURVED");
  int n = pipe.Parameters().geometryPoints;
  if (curves)
    for (auto node = curves->FirstChildElement(); node;
         node = node->NextSiblingElement())
      n = std::max(n, node->IntAttribute("NUMPOINTS"));
  else {
    curves = mesh.m_doc.NewElement("CURVED");
    geometry->InsertEndChild(curves);
  }
  if (n > 12)
    throw std::runtime_error("Source wall polynomial order exceeds supported geometryPoints");
  // Rebuild all circular edges with one common points distribution, then add
  // axial curvature and tensor-product wall faces. Interior cross-section
  // edges remain straight; their lateral faces are ruled by the axial curves.
  curves->DeleteChildren();
  std::set<int> circularEdges;
  for (int fid : mesh.m_bndComposite.at(0))
    for (int eid : mesh.m_faces.at(fid)) {
      const auto &e = mesh.m_edges.at(eid);
      if (std::fabs(mesh.m_pts.at(e[0])[2] - mesh.m_pts.at(e[1])[2]) < 1e-8)
        circularEdges.insert(eid);
    }
  int curveId = 0;
  const double pi = std::acos(-1.);
  for (const auto &edge : mesh.m_edges) {
    const auto &a = mesh.m_pts.at(edge.second[0]);
    const auto &b = mesh.m_pts.at(edge.second[1]);
    const bool circular = circularEdges.count(edge.first) != 0;
    const bool axial = std::fabs(a[2] - b[2]) > 1e-8;
    if (!circular && !(axial && pipe.CurvesLayer(axis.At(a[2]), axis.At(b[2]))))
      continue;
    if (axial && (std::fabs(a[0] - b[0]) + std::fabs(a[1] - b[1]) > 1e-7))
      throw std::runtime_error("Axial source edge connects different section vertices");
    const double angle = std::atan2(a[1] - source.y, a[0] - source.x);
    const double delta = std::remainder(std::atan2(b[1] - source.y, b[0] - source.x)
                                       - angle, 2. * pi);
    std::vector<Point> points;
    for (int i = 0; i < n; ++i) {
      const double t = static_cast<double>(i) / (n - 1);
      Point p;
      for (int j = 0; j < 3; ++j)
        p[j] = a[j] * (1. - t) + b[j] * t;
      if (circular) {
        p[0] = source.x + source.radius * std::cos(angle + t * delta);
        p[1] = source.y + source.radius * std::sin(angle + t * delta);
      }
      points.push_back(p);
    }
    AppendCurve(mesh, curves, curveId, 'E', edge.first, points);
  }
  for (int fid : mesh.m_bndComposite.at(0)) {
    const auto ids = FaceVertices(mesh, fid);
    if (ids.size() != 4)
      throw std::runtime_error("Extruded circular wall faces must be quadrilaterals");
    const auto &a = mesh.m_pts.at(ids[0]);
    const auto &b = mesh.m_pts.at(ids[1]);
    const auto &c = mesh.m_pts.at(ids[2]);
    const auto &d = mesh.m_pts.at(ids[3]);
    const double z0 = std::min({a[2], b[2], c[2], d[2]});
    const double z1 = std::max({a[2], b[2], c[2], d[2]});
    if (!pipe.CurvesLayer(axis.At(z0), axis.At(z1)))
      continue;
    const bool circleAlongU = std::fabs(a[2] - b[2]) < 1e-8;
    const auto &next = circleAlongU ? b : d;
    const double angle = std::atan2(a[1] - source.y, a[0] - source.x);
    const double delta = std::remainder(std::atan2(next[1] - source.y, next[0] - source.x)
                                       - angle, 2. * pi);
    std::vector<Point> points;
    // Nektar quadrilateral ordering: u varies fastest, corners are 0,1,2,3.
    for (int j = 0; j < n; ++j)
      for (int i = 0; i < n; ++i) {
        const double u = static_cast<double>(i) / (n - 1);
        const double v = static_cast<double>(j) / (n - 1);
        const double phi = angle + delta * (circleAlongU ? u : v);
        const double z = (1. - u) * (1. - v) * a[2] + u * (1. - v) * b[2]
                         + u * v * c[2] + (1. - u) * v * d[2];
        points.push_back({source.x + source.radius * std::cos(phi),
                          source.y + source.radius * std::sin(phi), z});
      }
    AppendCurve(mesh, curves, curveId, 'F', fid, points);
  }
}

void WriteAxis(const BentPipe &pipe, const SourceSection &source,
               const std::vector<double> &s) {
  std::ofstream out("axis.csv");
  if (!out)
    throw std::runtime_error("Cannot write axis.csv");
  out << "plane,source_z,s,ds,center_x,center_y,center_z,tangent_x,tangent_y,tangent_z,normal_x,normal_y,normal_z\n";
  out << std::setprecision(17);
  for (std::size_t i = 0; i < s.size(); ++i) {
    const auto frame = pipe.At(s[i]);
    out << i << ',' << source.planes[i] << ',' << s[i] << ','
        << (i ? s[i] - s[i - 1] : 0.);
    for (const auto &point : {frame.center, frame.tangent, frame.normal})
      for (double value : point)
        out << ',' << value;
    out << '\n';
  }
}
} // namespace

void GeneratePipeMesh(const BentPipe &pipe, const std::string &input,
                      const std::string &output) {
  NektarppXml mesh(input, "pipe_", 1e-6);
  mesh.LoadXml(0, {}); // Preserve raw z; interpolate every curve point later.
  const auto source = PrepareSource(mesh);
  const double factor = pipe.CheckRadius(source.radius);
  const int layers = static_cast<int>(source.planes.size()) - 1;
  if (layers != pipe.TotalLayers())
    throw std::invalid_argument("Source extrusion layer count differs from the three-segment "
                                "distribution; run bash make.sh to re-extrude");
  const auto stations = pipe.Stations();
  const PipeStations axis(source.planes, stations);
  std::cout << "Input section radius: " << source.radius
            << "; bend radius: " << pipe.Parameters().bendRadius
            << "; inner radius: " << pipe.Parameters().bendRadius - source.radius
            << "; continuous Jacobian factor >= " << factor << '\n';
  std::cout << "Layers: " << layers << " = " << pipe.InletLayers()
            << " inlet + " << pipe.Parameters().bendLayers << " bend + "
            << pipe.OutletLayers() << " outlet; bend spacing: " << pipe.BendSpacing() << '\n';
  PrepareCurvedGeometry(mesh, pipe, source, axis);
  mesh.DeformPts(std::function<void(double *)>([&](double *p) {
    const auto mapped = pipe.Map(p[0] - source.x, p[1] - source.y, axis.At(p[2]));
    std::copy(mapped.begin(), mapped.end(), p);
  }));
  mesh.UpdateXml();
  mesh.CheckMesh();
  mesh.OutXml(output);
  WriteAxis(pipe, source, stations);

  std::ofstream summary("pipe_summary.json");
  if (!summary)
    throw std::runtime_error("Cannot write pipe_summary.json");
  const auto &p = pipe.Parameters();
  summary << std::setprecision(17)
          << "{\n  \"bend_angle_degrees\": " << p.bendAngleDegrees
          << ",\n  \"bend_radius\": " << p.bendRadius
          << ",\n  \"pipe_radius\": " << source.radius
          << ",\n  \"source_center_x\": " << source.x
          << ",\n  \"source_center_y\": " << source.y
          << ",\n  \"inlet_extension\": " << p.inletExtension
          << ",\n  \"outlet_extension\": " << p.outletExtension
          << ",\n  \"bend_spacing\": " << pipe.BendSpacing()
          << ",\n  \"bend_layers\": " << p.bendLayers
          << ",\n  \"inlet_growth_rate\": " << p.inletGrowthRate
          << ",\n  \"outlet_growth_rate\": " << p.outletGrowthRate
          << ",\n  \"layers\": " << layers
          << ",\n  \"inlet_layers\": " << pipe.InletLayers()
          << ",\n  \"outlet_layers\": " << pipe.OutletLayers()
          << ",\n  \"minimum_continuous_jacobian_factor\": " << factor
          << ",\n  \"vertices\": " << mesh.m_pts.size()
          << ",\n  \"elements\": " << mesh.m_cells.size() << "\n}\n";
}
