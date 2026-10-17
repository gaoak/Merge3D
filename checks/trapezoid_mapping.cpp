#include "TrapezoidWing.h"
#include <iostream>
#include <limits>
#include <string>
#include <utility>

static void require(bool ok, const std::string &message) {
  if (!ok) throw std::runtime_error(message);
}
static void close(double a, double b, const std::string &message,
                  double tolerance = 1e-11) {
  require(std::fabs(a - b) < tolerance, message);
}
template <class F> static void rejects(F action, const std::string &message) {
  bool rejected = false;
  try { action(); } catch (const std::invalid_argument &) { rejected = true; }
  require(rejected, message);
}

int main() {
  try {
    const double pi = std::acos(-1.), radius = 1. / 60.;
    for (double degrees : {0., 15., -30.})
      for (double offset : {0., .5, 1., -.2})
        for (const auto chords : {std::pair<double, double>{1.5, .5},
                                  {1., 1.}, {1.2, 1.2}}) {
          const double root = chords.first, tip = chords.second;
          TrapezoidWing map(1., root, tip, offset, 5., degrees, radius, radius,
                            .1, 5., -1., -5., 2., 6.);
          const double co = std::cos(degrees * pi / 180.);
          const double si = std::sin(degrees * pi / 180.);
          const auto checkPoint = [&](double u, double v, double z, double expectedU) {
            double p[] = {u * co + v * si, -u * si + v * co, z};
            map.Apply(p);
            close(p[0] * co - p[1] * si, expectedU, "Chordwise geometry");
            close(p[0] * si + p[1] * co, v, "Thickness changed");
            close(p[2], z, "Span coordinate changed");
          };
          double origin[] = {0., 0., 0.};
          map.Apply(origin);
          require(origin[0] == 0. && origin[1] == 0. && origin[2] == 0.,
                  "Root leading edge moved from origin");
          for (double z : {-1., 0., 1.25, 2.5, 3.75, 5., 7., 9.}) {
            const double t = std::max(0., std::min(1., z / 5.));
            const double chord = root + t * (tip - root), le = offset * t;
            close(map.ChordAt(z), chord, "Linear spanwise chord");
            close(map.LeadingEdgeAt(z), le, "Linear leading edge");
            close(map.TrailingEdgeAt(z), le + chord, "Trailing edge location");
            checkPoint(0., 0., z, le);
            checkPoint(1., 0., z, le + chord);
            for (int i = 0; i <= 40; ++i) {
              const double theta = pi / 2. + pi * i / 40.;
              checkPoint(radius + radius * std::cos(theta), radius * std::sin(theta),
                         z, radius + radius * std::cos(theta) + le);
              const double teTheta = -pi / 2. + pi * i / 40.;
              const double teU = 1. - radius + radius * std::cos(teTheta);
              checkPoint(teU, radius * std::sin(teTheta), z, teU + le + chord - 1.);
            }
          }
          // Include upstream, inter-cap, downstream and normal fluid blends.
          for (int i = 0; i <= 160; ++i)
            for (double v : {-6., -4., -.1, -radius, 0., radius, .1, 4., 6.})
              for (double z : {0., 1., 4., 5., 9.}) {
                const double u = -6. + i * .0875, eps = 1e-6;
                double a[] = {(u-eps)*co+v*si, -(u-eps)*si+v*co, z};
                double b[] = {(u+eps)*co+v*si, -(u+eps)*si+v*co, z};
                map.Apply(a); map.Apply(b);
                const double jac = ((b[0]-a[0])*co-(b[1]-a[1])*si)/(2.*eps);
                require(jac >= map.JacobianLowerBound()-1e-8,
                        "Mapping folds or violates Jacobian bound");
              }
          for (double x : {-40., 60.})
            for (double y : {-40., 0., 40.}) {
              double p[] = {x, y, 5.};
              map.Apply(p);
              require(p[0] == x && p[1] == y && p[2] == 5., "Far field moved");
            }
        }
    TrapezoidWing identity(1., 1., 1., 0., 5., 15., radius, radius,
                           .1, 5., -1., -5., 2., 6.);
    for (double z : {0., 5., 9.}) {
      double p[] = {.3, -.2, z};
      identity.Apply(p);
      require(p[0] == .3 && p[1] == -.2 && p[2] == z, "Identity mapping");
    }
    rejects([&] { TrapezoidWing bad(1., 1.5, .1, .5, 5., 15., radius, radius,
                                     .1, 5., -1., -5., 2., 6.); }, "Inter-cap folding accepted");
    rejects([&] { TrapezoidWing bad(1., 1.5, .5, -3., 5., 15., radius, radius,
                                     .1, 5., -1., -5., 2., 6.); }, "Upstream folding accepted");
    rejects([&] { TrapezoidWing bad(1., 1.5, .5, 4., 5., 15., radius, radius,
                                     .1, 5., -1., -5., 2., 6.); }, "Downstream folding accepted");
    rejects([&] { TrapezoidWing bad(1., 1.5, .5, .5, 5., 15., radius, radius,
                                     .1, 5., -1., -5., 1., 6.); }, "Bad downstream blend accepted");
    rejects([&] { TrapezoidWing bad(1., 1.5, .5,
                                     std::numeric_limits<double>::quiet_NaN(),
                                     5., 15., radius, radius, .1, 5., -1., -5., 2., 6.); },
            "NaN offset accepted");
    std::cout << "Root-anchored trapezoid mapping checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
