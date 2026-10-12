#include "CAD3D/Nektarpp.h"
#include "CAD3D/Util.h"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

static void translate(double *p) { p[0] += .125; p[1] -= .25; }
static void require(bool ok, const char *message) {
  if (!ok) throw std::runtime_error(message);
}
static std::map<std::string, std::vector<double>> curves(NektarppXml &mesh) {
  std::map<std::string, std::vector<double>> result;
  auto root = mesh.m_doc.FirstChildElement("NEKTAR")
                  ->FirstChildElement("GEOMETRY")->FirstChildElement("CURVED");
  for (auto node = root->FirstChildElement(); node; node = node->NextSiblingElement()) {
    const bool edge = std::string(node->Name()) == "E";
    const auto key = std::string(node->Name()) +
                     std::to_string(node->IntAttribute(edge ? "EDGEID" : "FACEID"));
    parserDouble(node->GetText(), result[key]);
  }
  return result;
}
// Serialization stress record: a face with more text than the old fixed
// 10,000-byte buffer. No NekMesh geometry evaluation is performed on this record.
static void addLargeFace(NektarppXml &mesh) {
  auto root = mesh.m_doc.FirstChildElement("NEKTAR")
                  ->FirstChildElement("GEOMETRY")->FirstChildElement("CURVED");
  auto face = mesh.m_doc.NewElement("F");
  face->SetAttribute("ID", 100000);
  face->SetAttribute("FACEID", 0);
  face->SetAttribute("NUMPOINTS", 144);
  face->SetAttribute("TYPE", "PolyEvenlySpaced");
  std::ostringstream text;
  text << std::scientific << std::setprecision(18);
  for (int j = 0; j < 12; ++j)
    for (int i = 0; i < 12; ++i)
      text << i / 11. << ' ' << j / 11. << ' ' << 0. << ' ';
  require(text.str().size() > 10000, "Test face does not exceed the old buffer");
  face->SetText(text.str().c_str());
  root->InsertEndChild(face);
}

int main(int argc, char *argv[]) {
  try {
    require(argc == 3, "Provide source section and roundtrip output filenames");
    NektarppXml legacy(argv[1]), modern(argv[1]);
    legacy.LoadXml(0, {});
    modern.LoadXml(0, {});
    legacy.DeformPts((void *)translate);
    const double dx = .125, dy = -.25;
    modern.DeformPts(std::function<void(double *)>([&](double *p) {
      p[0] += dx; p[1] += dy;
    }));
    require(legacy.m_pts == modern.m_pts, "Old/new callbacks changed vertex results");
    require(curves(legacy) == curves(modern), "Old/new callbacks changed edge curves");
    addLargeFace(modern);
    const auto before = curves(modern);
    modern.UpdateXml();
    modern.OutXml(argv[2]);
    NektarppXml replay(argv[2]);
    replay.LoadXml(0, {});
    require(curves(replay) == before, "Curve points changed order or precision on reload");
    for (const auto &p : modern.m_pts)
      for (int i = 0; i < 3; ++i)
        require(std::fabs(p.second[i]-replay.m_pts.at(p.first)[i]) < 1e-12,
                "Vertex roundtrip changed geometry");
    bool rejected = false;
    try { NektarppXml missing(std::string(argv[2])+".missing"); }
    catch (const std::runtime_error &) { rejected = true; }
    require(rejected, "Missing XML does not report an error");
    std::cout << "CAD3D old callback and large curved-face roundtrip checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
