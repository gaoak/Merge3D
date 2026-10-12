#ifndef BENT_PIPE_H
#define BENT_PIPE_H

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

struct PipeParameters {
  double bendAngleDegrees = 90.;
  double bendRadius = 2.;
  double inletExtension = 2.;
  double outletExtension = 2.;
  int inletLayers = 14;
  int bendLayers = 32;
  int outletLayers = 14;
  double inletGrowthRate = 1.06;
  double outletGrowthRate = 1.06;
  int geometryPoints = 6;
};

class BentPipe {
public:
  using Point = std::array<double, 3>;
  struct Frame {
    Point center, tangent, normal;
  };

  explicit BentPipe(const PipeParameters &p) : m_parameters(p) {
    const double values[] = {p.bendAngleDegrees, p.bendRadius, p.inletExtension,
                             p.outletExtension, p.inletGrowthRate, p.outletGrowthRate};
    for (double value : values)
      if (!std::isfinite(value))
        throw std::invalid_argument("Pipe parameters must be finite");
    if (p.bendAngleDegrees <= 0. || p.bendAngleDegrees > 180. ||
        p.bendRadius <= 0. || p.inletExtension < 0. ||
        p.outletExtension < 0. || p.bendLayers < 1 ||
        p.inletLayers < 0 || p.outletLayers < 0 ||
        p.inletGrowthRate < 1. || p.outletGrowthRate < 1. ||
        p.geometryPoints < 2 || p.geometryPoints > 12)
      throw std::invalid_argument("Invalid bend, spacing or geometry parameters");
    m_arcLength = p.bendRadius * (p.bendAngleDegrees * std::acos(-1.) / 180.);
    if (!std::isfinite(m_arcLength + p.outletExtension) ||
        !std::isfinite(BendSpacing()) || BendSpacing() <= 0.)
      throw std::invalid_argument("Invalid arc length or bend spacing");
    if (p.bendLayers > 1000000 || p.inletLayers > 1000000 - p.bendLayers ||
        p.outletLayers > 1000000 - p.bendLayers - p.inletLayers || TotalLayers() < 2)
      throw std::invalid_argument("Total layer count must be between 2 and 1000000");
    m_inlet = StraightStations(p.inletExtension, p.inletLayers, p.inletGrowthRate);
    m_outlet = StraightStations(p.outletExtension, p.outletLayers, p.outletGrowthRate);
  }

  double ArcLength() const { return m_arcLength; }
  double BendSpacing() const { return m_arcLength / m_parameters.bendLayers; }
  int InletLayers() const { return m_parameters.inletLayers; }
  int OutletLayers() const { return m_parameters.outletLayers; }
  int TotalLayers() const { return InletLayers() + m_parameters.bendLayers + OutletLayers(); }
  const PipeParameters &Parameters() const { return m_parameters; }

  Frame At(double s) const {
    const double arcS = std::max(0., std::min(m_arcLength, s));
    const double theta = arcS / m_parameters.bendRadius;
    const double sine = std::sin(theta), cosine = std::cos(theta);
    // Bend entry is the origin; inlet tangent is +x, bend turns towards +z.
    // N points outward; N x (0,1,0) = T preserves source orientation.
    const double h = std::sin(theta / 2.);
    Frame f{{m_parameters.bendRadius * sine, 0.,
              2. * m_parameters.bendRadius * h * h},
             {cosine, 0., sine}, {sine, 0., -cosine}};
    for (int i = 0; i < 3; ++i)
      f.center[i] += (s - arcS) * f.tangent[i];
    return f;
  }

  Point Map(double x, double y, double s) const {
    const Frame f = At(s);
    return {f.center[0] + x * f.normal[0], y,
            f.center[2] + x * f.normal[2]};
  }

  double CheckRadius(double sectionRadius) const {
    if (!std::isfinite(sectionRadius) || sectionRadius <= 0.)
      throw std::invalid_argument("Invalid input cross-section radius");
    if (m_parameters.bendRadius <= sectionRadius * (1. + 1e-8))
      throw std::invalid_argument("Bend radius must exceed the actual pipe radius; "
                                  "otherwise inner-side sections fold/overlap");
    return 1. - sectionRadius / m_parameters.bendRadius;
  }

  bool CurvesLayer(double s0, double s1) const {
    const double tol = 1e-12 * std::max(1., m_arcLength);
    return std::min(s0, s1) < m_arcLength - tol && std::max(s0, s1) > tol;
  }

  std::vector<double> Stations() const {
    std::vector<double> result;
    result.reserve(TotalLayers() + 1);
    for (int i = InletLayers(); i > 0; --i)
      result.push_back(-m_inlet[i]);
    for (int i = 0; i <= m_parameters.bendLayers; ++i)
      result.push_back(m_arcLength * (static_cast<double>(i) / m_parameters.bendLayers));
    for (int i = 1; i <= OutletLayers(); ++i)
      result.push_back(m_arcLength + m_outlet[i]);
    return result;
  }

private:
  std::vector<double> StraightStations(double length, int count, double growth) const {
    if (length == 0. && count == 0)
      return {0.};
    if (length == 0. || count == 0)
      throw std::invalid_argument("Straight length and its layer count must both be zero "
                                  "or both positive");
    const double first = BendSpacing();
    if (first > length / count * (1. + 1e-12))
      throw std::invalid_argument("Bend spacing times straight layer count exceeds "
                                  "straight length; reduce that count or increase bendLayers");
    if (count == 1 && std::fabs(length - first) > first * 1e-12)
      throw std::invalid_argument("A one-layer straight must have exactly the bend spacing");
    // Fixed count and exact first spacing. The specified rate bounds growth;
    // cap it with a uniform tail to reach the prescribed straight endpoint.
    double possible = 0., step = first;
    for (int i = 0; i < count && possible < length; ++i) {
      possible += step;
      step *= growth;
    }
    if (possible < length * (1. - 1e-12))
      throw std::invalid_argument("Straight layer count cannot reach its length with "
                                  "the specified bend spacing/growth rate; increase count or rate");

    std::vector<double> nodes(1, 0.);
    double remaining = length;
    step = first;
    bool uniform = false;
    for (int i = 0; i < count; ++i) {
      const int remainingCount = count - i;
      const double ds = remainingCount == 1 ? remaining : step;
      nodes.push_back(nodes.back() + ds);
      remaining -= ds;
      if (remainingCount > 1 && !uniform) {
        const double average = remaining / (remainingCount - 1);
        if (step * growth > average) {
          step = average;
          uniform = true;
        } else
          step *= growth;
      }
    }
    nodes.back() = length;
    return nodes;
  }

  PipeParameters m_parameters;
  double m_arcLength;
  std::vector<double> m_inlet, m_outlet;
};

// Linear reparameterization between the actual source extrusion planes.
// Arc lengths, rather than raw extrusion z, set the spacing of each section.
class PipeStations {
public:
  PipeStations(std::vector<double> planes, std::vector<double> stations)
      : m_planes(std::move(planes)), m_stations(std::move(stations)) {
    if (m_planes.size() < 2 || m_planes.size() != m_stations.size())
      throw std::invalid_argument("Invalid source/target plane counts");
    for (std::size_t i = 1; i < m_planes.size(); ++i)
      if (m_planes[i] <= m_planes[i - 1] || m_stations[i] <= m_stations[i - 1])
        throw std::invalid_argument("Source and target planes must be ordered");
  }

  double At(double z) const {
    const double tol = 1e-8 * std::max(1., m_planes.back() - m_planes.front());
    if (z < m_planes.front() - tol || z > m_planes.back() + tol)
      throw std::invalid_argument("Curved point lies outside the source extrusion");
    auto it = std::lower_bound(m_planes.begin(), m_planes.end(), z);
    if (it != m_planes.end() && std::fabs(*it - z) < tol)
      return m_stations[it - m_planes.begin()];
    if (it == m_planes.begin())
      return m_stations.front();
    if (it == m_planes.end())
      return m_stations.back();
    const std::size_t upper = it - m_planes.begin();
    if (std::fabs(z - m_planes[upper - 1]) < tol)
      return m_stations[upper - 1];
    const double t = (z - m_planes[upper - 1]) /
                     (m_planes[upper] - m_planes[upper - 1]);
    return m_stations[upper - 1] * (1. - t) + m_stations[upper] * t;
  }

private:
  std::vector<double> m_planes, m_stations;
};

#endif
