#ifndef TRAPEZOID_WING_H
#define TRAPEZOID_WING_H

#include <algorithm>
#include <cmath>
#include <stdexcept>

// Mesh2D coordinates: x = u*cos(AoA) + v*sin(AoA),
// y = -u*sin(AoA) + v*cos(AoA). The source leading edge is u=0.
class TrapezoidWing {
public:
  TrapezoidWing(double sourceChord, double rootChord, double tipChord,
                double tipLEOffset, double span, double angleDegrees,
                double leadingRadius, double trailingRadius,
                double nearNormal, double farNormal,
                double nearUpstream, double farUpstream,
                double nearDownstream, double farDownstream)
      : m_sourceChord(sourceChord), m_rootChord(rootChord),
        m_tipChord(tipChord), m_tipLEOffset(tipLEOffset), m_span(span),
        m_leadingRadius(leadingRadius), m_trailingRadius(trailingRadius),
        m_nearNormal(nearNormal), m_farNormal(farNormal),
        m_nearUpstream(nearUpstream), m_farUpstream(farUpstream),
        m_nearDownstream(nearDownstream), m_farDownstream(farDownstream) {
    const double values[] = {sourceChord, rootChord, tipChord, tipLEOffset, span,
                             angleDegrees, leadingRadius, trailingRadius,
                             nearNormal, farNormal, nearUpstream, farUpstream,
                             nearDownstream, farDownstream,
                             tipLEOffset + tipChord - sourceChord};
    for (double value : values)
      if (!std::isfinite(value))
        throw std::invalid_argument("Non-finite trapezoid parameter");
    if (span <= 0. || leadingRadius <= 0. || trailingRadius <= 0. ||
        std::min({sourceChord, rootChord, tipChord}) <=
            leadingRadius + trailingRadius ||
        nearNormal <= std::max(leadingRadius, trailingRadius) ||
        farNormal <= nearNormal || nearUpstream >= 0. ||
        farUpstream >= nearUpstream || nearDownstream <= sourceChord ||
        farDownstream <= nearDownstream)
      throw std::invalid_argument("Invalid trapezoid dimensions or blend extent");
    const double angle = angleDegrees * std::acos(-1.) / 180.;
    m_cos = std::cos(angle);
    m_sin = std::sin(angle);
    if (JacobianLowerBound() <= 0.)
      throw std::invalid_argument("Taper would fold the continuous mapping; "
                                  "increase the source chord/blend extent");
  }

  double ChordAt(double z) const {
    return m_rootChord + SpanFraction(z) * (m_tipChord - m_rootChord);
  }
  double LeadingEdgeAt(double z) const { return m_tipLEOffset * SpanFraction(z); }
  double TrailingEdgeAt(double z) const { return LeadingEdgeAt(z) + ChordAt(z); }

  double JacobianLowerBound() const {
    // Only u changes: det(J) = 1 + d(displacement)/du. Smoothstep's
    // maximum derivative is 1.5; normal blending lies between zero and one.
    const double shrink = std::max(0., m_sourceChord -
                                          std::min(m_rootChord, m_tipChord));
    const double upstreamCompression = std::max(0., -m_tipLEOffset);
    const double downstreamCompression = std::max({
        0., m_rootChord - m_sourceChord,
        m_tipLEOffset + m_tipChord - m_sourceChord});
    return std::min({
        1. - 1.5 * shrink / (m_sourceChord - m_leadingRadius - m_trailingRadius),
        1. - 1.5 * upstreamCompression / (m_nearUpstream - m_farUpstream),
        1. - 1.5 * downstreamCompression / (m_farDownstream - m_nearDownstream)});
  }

  void Apply(double *p) const {
    const double u = p[0] * m_cos - p[1] * m_sin;
    const double v = p[0] * m_sin + p[1] * m_cos;
    const double leadingShift = LeadingEdgeAt(p[2]);
    const double trailingShift = TrailingEdgeAt(p[2]) - m_sourceChord;
    if ((leadingShift == 0. && trailingShift == 0.) || u <= m_farUpstream ||
        u >= m_farDownstream || std::fabs(v) >= m_farNormal)
      return;

    double du;
    if (u < m_nearUpstream) {
      du = leadingShift * Smoothstep((u - m_farUpstream) /
                                    (m_nearUpstream - m_farUpstream));
    } else if (u <= m_leadingRadius) {
      du = leadingShift; // Translate the complete leading semicircle.
    } else if (u < m_sourceChord - m_trailingRadius) {
      const double t = Smoothstep((u - m_leadingRadius) /
                                 (m_sourceChord - m_leadingRadius - m_trailingRadius));
      du = leadingShift * (1. - t) + trailingShift * t;
    } else if (u <= m_nearDownstream) {
      du = trailingShift; // Translate the complete trailing semicircle.
    } else {
      du = trailingShift * (1. - Smoothstep((u - m_nearDownstream) /
                                          (m_farDownstream - m_nearDownstream)));
    }
    if (std::fabs(v) > m_nearNormal)
      du *= 1. - Smoothstep((std::fabs(v) - m_nearNormal) /
                            (m_farNormal - m_nearNormal));
    if (du == 0.)
      return;
    // Preserve normal/span coordinates, cap shapes and unchanged point bytes.
    p[0] += du * m_cos;
    p[1] -= du * m_sin;
  }

private:
  double SpanFraction(double z) const { return std::max(0., std::min(1., z / m_span)); }
  static double Smoothstep(double t) { return t * t * (3. - 2. * t); }
  double m_sourceChord, m_rootChord, m_tipChord, m_tipLEOffset, m_span;
  double m_leadingRadius, m_trailingRadius, m_nearNormal, m_farNormal;
  double m_nearUpstream, m_farUpstream, m_nearDownstream, m_farDownstream;
  double m_cos, m_sin;
};

#endif
