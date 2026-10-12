#include "BentPipe.h"
#include <iostream>
#include <limits>
#include <string>

static void require(bool condition, const std::string &message) {
  if (!condition)
    throw std::runtime_error(message);
}
static double dot(const BentPipe::Point &a, const BentPipe::Point &b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
static double determinant(const BentPipe::Point &a, const BentPipe::Point &b,
                          const BentPipe::Point &c) {
  return a[0] * (b[1] * c[2] - b[2] * c[1])
         - b[0] * (a[1] * c[2] - a[2] * c[1])
         + c[0] * (a[1] * b[2] - a[2] * b[1]);
}
static void close(double a, double b, const std::string &message, double tol = 1e-10) {
  require(std::fabs(a - b) < tol, message);
}
template <class F> static void rejects(F action, const std::string &message) {
  bool rejected = false;
  try { action(); } catch (const std::invalid_argument &) { rejected = true; }
  require(rejected, message);
}

int main() {
  try {
    PipeParameters p;
    BentPipe pipe(p);
    const auto stations = pipe.Stations(128);
    require(stations.size() == 129, "Incorrect plane count");
    close(stations[64], 0., "Missing exact bend-center plane");
    close(stations[65], .02, "Incorrect right center spacing");
    close(stations[63], -.02, "Incorrect left center spacing");
    close(stations.front(), -pipe.InletLength(), "Incorrect inlet position");
    close(stations.back(), pipe.OutletLength(), "Incorrect outlet position");
    for (int i = 0; i < 129; ++i)
      close(stations[i], -stations[128-i], "Symmetric case lost symmetry");
    for (int side : {-1, 1}) {
      double previous = .02;
      for (int i = 2; i <= 64; ++i) {
        const double ds = side * (stations[64+side*i] - stations[64+side*(i-1)]);
        require(ds >= previous - 1e-12 && ds <= previous * p.growthRate + 1e-12,
                "Spacing shrinks or exceeds the requested growth rate");
        previous = ds;
      }
    }
    close(pipe.CheckRadius(.5), .75, "Wrong minimum inner-side Jacobian factor");
    const auto inlet = pipe.At(-pipe.InletLength());
    const auto outlet = pipe.At(pipe.OutletLength());
    close(inlet.center[0], 2., "Inlet x");
    close(outlet.center[0], 2., "Outlet x");
    close(inlet.center[2], -2.*std::sqrt(2.), "Inlet z");
    close(outlet.center[2], 2.*std::sqrt(2.), "Outlet z");
    close(dot(inlet.tangent, outlet.tangent), 0., "Total turning angle");

    for (double s : {-pipe.InletLength(), -pipe.HalfArc(), -.5, 0., .5,
                      pipe.HalfArc(), pipe.OutletLength()}) {
      const auto frame = pipe.At(s);
      close(dot(frame.normal, frame.tangent), 0., "Non-orthogonal frame");
      close(dot(frame.normal, frame.normal), 1., "Normal is not a unit vector");
      close(dot(frame.tangent, frame.tangent), 1., "Tangent is not a unit vector");
      const double eps = 1e-6;
      const auto before = pipe.At(s-eps), after = pipe.At(s+eps);
      for (int k = 0; k < 3; ++k)
        close((after.center[k]-before.center[k])/(2.*eps), frame.tangent[k],
              "Centerline tangent/join is discontinuous", 3e-7);
      for (int i = 0; i < 36; ++i) {
        const double angle = i * std::acos(-1.) / 18.;
        const auto point = pipe.Map(.5*std::cos(angle), .5*std::sin(angle), s);
        BentPipe::Point offset;
        for (int k = 0; k < 3; ++k) offset[k] = point[k]-frame.center[k];
        close(dot(offset, frame.tangent), 0., "Section is not perpendicular");
        close(dot(offset, offset), .25, "Section is not a circle of radius .5");
      }
    }
    for (double s : {-3., -.5, 0., .5, 3.})
      for (double x : {-.5, 0., .5}) {
        const double eps = 1e-6;
        const auto origin = pipe.Map(x, .1, s);
        const auto ax = pipe.Map(x+eps, .1, s);
        const auto ay = pipe.Map(x, .1+eps, s);
        const auto az = pipe.Map(x, .1, s+eps);
        BentPipe::Point dx, dy, dz;
        for (int k = 0; k < 3; ++k) {
          dx[k] = (ax[k]-origin[k])/eps;
          dy[k] = (ay[k]-origin[k])/eps;
          dz[k] = (az[k]-origin[k])/eps;
        }
        const double jac = determinant(dx, dy, dz);
        close(jac, std::fabs(s)<pipe.HalfArc() ? 1.-x/p.bendRadius : 1.,
              "Incorrect sweep Jacobian", 1e-6);
      }
    p.outletExtension = 4.;
    BentPipe asymmetric(p);
    const auto different = asymmetric.Stations(130);
    require(different.size() == 131, "Outlet count is not derived from total planes");
    close(different[64], 0., "Asymmetric center");
    close(different.back(), asymmetric.OutletLength(), "Asymmetric outlet length");

    p = PipeParameters(); p.bendAngleDegrees = 0.;
    BentPipe straight(p);
    const auto straightPoint = straight.Map(.2, -.3, 1.);
    close(straightPoint[0], .2, "Straight-pipe x");
    close(straightPoint[1], -.3, "Straight-pipe y");
    close(straightPoint[2], 1., "Straight-pipe z");
    p.bendAngleDegrees = 180.;
    BentPipe hairpin(p);
    close(hairpin.At(-hairpin.InletLength()).center[2], -2., "180 degree inlet");
    close(hairpin.At(hairpin.OutletLength()).center[2], 2., "180 degree outlet");
    p.bendRadius = .5;
    rejects([&] { BentPipe(p).CheckRadius(.5); }, "Collapsed inner radius accepted");
    p.bendRadius = .4;
    rejects([&] { BentPipe(p).CheckRadius(.5); }, "Overlapping sections accepted");
    rejects([&] { pipe.Stations(64); }, "Invalid half-layer split accepted");
    p = PipeParameters(); p.growthRate = 1.;
    rejects([&] { BentPipe(p).Stations(128); }, "Impossible fixed-spacing length accepted");
    p = PipeParameters(); p.centerSpacing = .5;
    rejects([&] { BentPipe(p).Stations(128); }, "Too-large center spacing accepted");
    p = PipeParameters(); p.bendRadius = std::numeric_limits<double>::quiet_NaN();
    rejects([&] { BentPipe invalid(p); }, "NaN radius accepted");
    const PipeStations remap({2., 3., 4.}, {-2., 0., 3.});
    close(remap.At(2.5), -1., "Source z interpolation");
    close(remap.At(3.5), 1.5, "Curved axial point interpolation");
    std::cout << "Bent pipe geometry and spacing checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
