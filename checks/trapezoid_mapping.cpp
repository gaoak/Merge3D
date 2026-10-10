#include "TrapezoidWing.h"
#include <iostream>
#include <string>

static void require(bool ok, const std::string &message) {
  if (!ok)
    throw std::runtime_error(message);
}

static void close(double a, double b, const std::string &message,
                  double tolerance = 1e-11) {
  require(std::fabs(a - b) < tolerance, message);
}

int main() {
  try {
    const double pi = std::acos(-1.);
    const double radius = 1. / 60.;
    const double trailingCenter = 1. - radius;
    for (double degrees : {0., 15., -30.}) {
      TrapezoidWing map(1., 1.5, .5, 5., degrees, radius, radius,
                        .1, 5., -1., -5.);
      const double co = std::cos(degrees * pi / 180.);
      const double si = std::sin(degrees * pi / 180.);
      const auto checkPoint = [&](double u, double v, double z,
                                  double expectedU) {
        double p[] = {u * co + v * si, -u * si + v * co, z};
        map.Apply(p);
        close(p[0] * co - p[1] * si, expectedU, "Chordwise geometry");
        close(p[0] * si + p[1] * co, v, "Thickness changed");
        close(p[2], z, "Span coordinate changed");
      };
      for (double z : {0., 1.25, 2.5, 3.75, 5., 7., 9.}) {
        const double chord = z > 5. ? .5 : 1.5 - z / 5.;
        close(map.ChordAt(z), chord, "Linear spanwise taper");
        for (int i = 0; i <= 40; ++i) {
          const double theta = pi / 2. + pi * i / 40.;
          const double u = radius + radius * std::cos(theta);
          const double v = radius * std::sin(theta);
          checkPoint(u, v, z, u + 1. - chord);
          const double teTheta = -pi / 2. + pi * i / 40.;
          const double teU = trailingCenter + radius * std::cos(teTheta);
          checkPoint(teU, radius * std::sin(teTheta), z, teU);
        }
      }
      // Sample the actual map, including both fluid blending regions, rather
      // than relying solely on the analytical positivity bound.
      for (int i = 0; i <= 100; ++i)
        for (double v : {-6., -4., -.1, -radius, 0., radius, .1, 4., 6.})
          for (double z : {0., 1., 4., 5., 9.}) {
            const double u = -6. + i * .08;
            const double eps = 1e-6;
            double a[] = {(u - eps) * co + v * si,
                           -(u - eps) * si + v * co, z};
            double b[] = {(u + eps) * co + v * si,
                           -(u + eps) * si + v * co, z};
            map.Apply(a);
            map.Apply(b);
            const double jac = ((b[0] - a[0]) * co -
                                (b[1] - a[1]) * si) / (2. * eps);
            require(jac >= map.JacobianLowerBound() - 1e-8,
                    "Mapping folds or violates Jacobian bound");
          }
      for (double x : {-40., 60.})
        for (double y : {-40., 0., 40.}) {
          double p[] = {x, y, 5.};
          map.Apply(p);
          require(p[0] == x && p[1] == y && p[2] == 5.,
                  "Far field moved");
        }
    }
    TrapezoidWing identity(1., 1., 1., 5., 15., radius, radius,
                           .1, 5., -1., -5.);
    double p[] = {.3, -.2, 4.};
    identity.Apply(p);
    require(p[0] == .3 && p[1] == -.2 && p[2] == 4., "Identity mapping");
    bool rejected = false;
    try {
      TrapezoidWing invalid(1., 1.5, .1, 5., 15., radius, radius,
                            .1, 5., -1., -5.);
    } catch (const std::invalid_argument &) {
      rejected = true;
    }
    require(rejected, "Non-positive Jacobian configuration accepted");
    std::cout << "Trapezoid mapping checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
