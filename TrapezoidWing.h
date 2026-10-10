#ifndef TRAPEZOID_WING_H
#define TRAPEZOID_WING_H

#include <algorithm>
#include <cmath>
#include <stdexcept>

// Coordinates follow Mesh2D: x = u*cos(AoA) + v*sin(AoA),
// y = -u*sin(AoA) + v*cos(AoA). The source leading edge is u=0.
class TrapezoidWing {
public:
  TrapezoidWing(double sourceChord, double rootChord, double tipChord,
                double span, double angleDegrees, double leadingRadius,
                double trailingRadius, double nearNormal, double farNormal,
                double nearUpstream, double farUpstream)
      : m_sourceChord(sourceChord), m_rootChord(rootChord),
        m_tipChord(tipChord), m_span(span), m_leadingRadius(leadingRadius),
        m_trailingRadius(trailingRadius), m_nearNormal(nearNormal),
        m_farNormal(farNormal), m_nearUpstream(nearUpstream),
        m_farUpstream(farUpstream) {
    const double values[] = {sourceChord, rootChord, tipChord, span,
                             angleDegrees, leadingRadius, trailingRadius,
                             nearNormal, farNormal, nearUpstream, farUpstream};
    for (double value : values)
      if (!std::isfinite(value))
        throw std::invalid_argument("Non-finite trapezoid parameter");
    if (span <= 0. || leadingRadius <= 0. || trailingRadius <= 0. ||
        std::min({sourceChord, rootChord, tipChord}) <=
            leadingRadius + trailingRadius ||
        nearNormal <= std::max(leadingRadius, trailingRadius) ||
        farNormal <= nearNormal || nearUpstream >= 0. ||
        farUpstream >= nearUpstream)
      throw std::invalid_argument("Invalid trapezoid dimensions or blend extent");
    const double angle = angleDegrees * std::acos(-1.) / 180.;
    m_cos = std::cos(angle);
    m_sin = std::sin(angle);
    // The maximum derivative of smoothstep is 1.5. In the orthonormal
    // (u,v,z) frame only u changes, so det(J) = 1 + d(z)*dw/du.
    if (JacobianLowerBound() <= 0.)
      throw std::invalid_argument("Taper would fold the continuous mapping; "
                                  "increase the source chord/blend extent");
  }

  double ChordAt(double z) const {
    const double t = std::max(0., std::min(1., z / m_span));
    return m_rootChord + t * (m_tipChord - m_rootChord);
  }

  double JacobianLowerBound() const {
    const double shrink = std::max(0., m_sourceChord -
                                          std::min(m_rootChord, m_tipChord));
    const double grow = std::max(0., std::max(m_rootChord, m_tipChord) -
                                        m_sourceChord);
    return std::min(1. - 1.5 * shrink /
                            (m_sourceChord - m_leadingRadius - m_trailingRadius),
                    1. - 1.5 * grow / (m_nearUpstream - m_farUpstream));
  }

  void Apply(double *p) const {
    const double u = p[0] * m_cos - p[1] * m_sin;
    const double v = p[0] * m_sin + p[1] * m_cos;
    const double displacement = m_sourceChord - ChordAt(p[2]);
    if (displacement == 0. || u <= m_farUpstream ||
        u >= m_sourceChord - m_trailingRadius ||
        std::fabs(v) >= m_farNormal)
      return;

    double weight;
    if (u < m_nearUpstream) {
      weight = Smoothstep((u - m_farUpstream) /
                          (m_nearUpstream - m_farUpstream));
    } else if (u <= m_leadingRadius) {
      // Translate the complete leading-edge semicircle without scaling it.
      weight = 1.;
    } else {
      // Change only the length of the straight section between the two caps.
      weight = 1. - Smoothstep((u - m_leadingRadius) /
                              (m_sourceChord - m_leadingRadius -
                               m_trailingRadius));
    }
    if (std::fabs(v) > m_nearNormal)
      weight *= 1. - Smoothstep((std::fabs(v) - m_nearNormal) /
                               (m_farNormal - m_nearNormal));
    const double du = displacement * weight;
    // Keep v and z unchanged. Return early above to preserve the far field
    // and trailing cap exactly, including their original roundoff.
    p[0] += du * m_cos;
    p[1] -= du * m_sin;
  }

private:
  static double Smoothstep(double t) { return t * t * (3. - 2. * t); }
  double m_sourceChord, m_rootChord, m_tipChord, m_span;
  double m_leadingRadius, m_trailingRadius;
  double m_nearNormal, m_farNormal, m_nearUpstream, m_farUpstream;
  double m_cos, m_sin;
};

#endif
