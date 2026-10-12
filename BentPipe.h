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
  double centerSpacing = .02;
  double growthRate = 1.08;
  int inletLayers = 64;
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
                             p.outletExtension, p.centerSpacing, p.growthRate};
    for (double value : values)
      if (!std::isfinite(value))
        throw std::invalid_argument("Pipe parameters must be finite");
    if (p.bendAngleDegrees < 0. || p.bendAngleDegrees > 180. ||
        p.bendRadius <= 0. || p.inletExtension < 0. ||
        p.outletExtension < 0. || p.centerSpacing <= 0. ||
        p.growthRate < 1. || p.inletLayers < 1 ||
        p.geometryPoints < 2 || p.geometryPoints > 12)
      throw std::invalid_argument("Invalid bend, spacing or geometry parameters");
    m_halfArc = p.bendRadius * p.bendAngleDegrees * std::acos(-1.) / 360.;
    if (!std::isfinite(InletLength()) || !std::isfinite(OutletLength()) ||
        InletLength() <= 0. || OutletLength() <= 0.)
      throw std::invalid_argument("Both sides of the bend center need positive length");
  }

  double InletLength() const { return m_halfArc + m_parameters.inletExtension; }
  double OutletLength() const { return m_halfArc + m_parameters.outletExtension; }
  double HalfArc() const { return m_halfArc; }
  const PipeParameters &Parameters() const { return m_parameters; }

  Frame At(double s) const {
    const double arcS = std::max(-m_halfArc, std::min(m_halfArc, s));
    const double theta = arcS / m_parameters.bendRadius;
    const double sine = std::sin(theta), cosine = std::cos(theta);
    // Center of the bend lies at (R,0,0); s=0 is the middle of the arc.
    const double h = std::sin(theta / 2.);
    Frame f{{2. * m_parameters.bendRadius * h * h, 0.,
              m_parameters.bendRadius * sine},
             {sine, 0., cosine}, {cosine, 0., -sine}};
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
    if (m_halfArc > 0. && m_parameters.bendRadius <= sectionRadius * (1. + 1e-8))
      throw std::invalid_argument("Bend radius must exceed the actual pipe radius; "
                                  "otherwise inner-side sections fold/overlap");
    return m_halfArc == 0. ? 1. : 1. - sectionRadius / m_parameters.bendRadius;
  }

  bool CurvesLayer(double s0, double s1) const {
    return m_halfArc > 0. && std::min(s0, s1) < m_halfArc &&
           std::max(s0, s1) > -m_halfArc;
  }

  std::vector<double> Stations(int totalLayers) const {
    const int rightCount = totalLayers - m_parameters.inletLayers;
    if (rightCount <= 0)
      throw std::invalid_argument("inletLayers must be smaller than the actual "
                                  "number of extruded layers");
    const auto left = HalfStations(InletLength(), m_parameters.inletLayers);
    const auto right = HalfStations(OutletLength(), rightCount);
    std::vector<double> result;
    result.reserve(totalLayers + 1);
    for (int i = m_parameters.inletLayers; i > 0; --i)
      result.push_back(-left[i]);
    result.insert(result.end(), right.begin(), right.end());
    return result;
  }

private:
  std::vector<double> HalfStations(double length, int count) const {
    const double first = m_parameters.centerSpacing;
    const double growth = m_parameters.growthRate;
    if (first > length / count * (1. + 1e-12))
      throw std::invalid_argument("centerSpacing * side layer count exceeds "
                                  "side length; reduce spacing or layer count");
    // All spacings must grow by at most growth. If even the full geometric
    // series is too short, these independently chosen constraints cannot fit.
    double possible = 0., step = first;
    for (int i = 0; i < count && possible < length; ++i) {
      possible += step;
      step *= growth;
    }
    if (possible < length * (1. - 1e-12))
      throw std::invalid_argument("Too few side layers to reach its end with "
                                  "the specified centerSpacing/growthRate");

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
  double m_halfArc;
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
