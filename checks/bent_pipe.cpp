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

static void checkStations(const BentPipe &pipe) {
  const auto s = pipe.Stations();
  const auto &p = pipe.Parameters();
  const int entry = pipe.InletLayers(), exit = entry + p.bendLayers;
  const double tol = 1e-12*std::max({1., std::fabs(s.front()), std::fabs(s.back())});
  require(s.size() == static_cast<std::size_t>(pipe.TotalLayers()+1), "Wrong plane count");
  close(s.front(), -p.inletExtension, "Inlet length");
  close(s[entry], 0., "Missing exact bend entry");
  close(s[exit], pipe.ArcLength(), "Missing exact bend exit");
  close(s.back(), pipe.ArcLength()+p.outletExtension, "Outlet length");
  for (std::size_t i = 1; i < s.size(); ++i)
    require(s[i] > s[i-1], "Unordered planes");
  for (int i = entry; i < exit; ++i) {
    close(s[i+1]-s[i], pipe.BendSpacing(), "Nonuniform bend spacing");
    require(pipe.CurvesLayer(s[i], s[i+1]), "Bend layer lost curved geometry");
  }
  for (int side : {-1, 1}) {
    const int start = side < 0 ? entry : exit;
    const int count = side < 0 ? pipe.InletLayers() : pipe.OutletLayers();
    const double growth = side < 0 ? p.inletGrowthRate : p.outletGrowthRate;
    double previous = pipe.BendSpacing(), possible = 0., step = previous;
    for (int i = 1; i <= count; ++i) {
      const double ds = side*(s[start+side*i]-s[start+side*(i-1)]);
      if (i == 1) close(ds, pipe.BendSpacing(), "Straight/bend spacing mismatch", tol);
      require(ds >= previous-tol && ds <= previous*growth+tol,
              "Straight spacing shrinks or exceeds its growth rate: rate=" +
              std::to_string(growth) + ", count=" + std::to_string(count) +
              ", layer=" + std::to_string(i));
      require(!pipe.CurvesLayer(s[start+side*i], s[start+side*(i-1)]),
              "Straight cell classified as a bend cell");
      previous = ds;
      possible += step;
      step *= growth;
    }
    require(possible >= (side < 0 ? p.inletExtension : p.outletExtension)-tol,
            "Straight layers cannot reach endpoint");
  }
}

int main() {
  try {
    PipeParameters p;
    BentPipe pipe(p);
    checkStations(pipe);
    const auto stations = pipe.Stations();
    for (int i = 0; i <= pipe.TotalLayers(); ++i)
      close(stations[i]+stations[pipe.TotalLayers()-i], pipe.ArcLength(),
            "Equal sides lost symmetry");
    close(pipe.CheckRadius(.5), .75, "Wrong minimum inner-side Jacobian factor");
    const auto inlet = pipe.At(-p.inletExtension);
    const auto outlet = pipe.At(pipe.ArcLength()+p.outletExtension);
    close(inlet.center[0], -2., "Inlet x");
    close(inlet.center[2], 0., "Inlet z");
    close(inlet.tangent[0], 1., "Inlet tangent must be +x");
    close(inlet.tangent[2], 0., "Inlet tangent z");
    close(outlet.center[0], 2., "Outlet x");
    close(outlet.center[2], 4., "Outlet z");
    close(outlet.tangent[2], 1., "90-degree outlet tangent must be +z");
    close(dot(inlet.tangent, outlet.tangent), 0., "Total turning angle");

    for (double s : {-p.inletExtension, 0., .5, pipe.ArcLength()/2.,
                      pipe.ArcLength(), pipe.ArcLength()+p.outletExtension}) {
      const auto frame = pipe.At(s);
      close(dot(frame.normal, frame.tangent), 0., "Non-orthogonal frame");
      close(dot(frame.normal, frame.normal), 1., "Normal is not a unit vector");
      close(dot(frame.tangent, frame.tangent), 1., "Tangent is not a unit vector");
      close(determinant(frame.normal, {0., 1., 0.}, frame.tangent), 1.,
            "Frame flips source cell orientation");
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
    for (double s : {-1., .5, 1., 4.})
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
        close(determinant(dx, dy, dz), s>0. && s<pipe.ArcLength() ? 1.+x/p.bendRadius : 1.,
              "Incorrect sweep Jacobian", 1e-6);
      }

    p.inletExtension = 10.; p.outletExtension = 20.;
    p.inletLayers = 48; p.outletLayers = 45;
    p.inletGrowthRate = 1.04; p.outletGrowthRate = 1.12;
    BentPipe asymmetric(p);
    checkStations(asymmetric);
    const int inletCount = asymmetric.InletLayers();
    p.outletGrowthRate = 1.2;
    BentPipe changedOutlet(p);
    checkStations(changedOutlet);
    require(changedOutlet.InletLayers() == inletCount &&
            changedOutlet.OutletLayers() == p.outletLayers &&
            changedOutlet.TotalLayers() == p.inletLayers+p.bendLayers+p.outletLayers,
            "Growth rate changed specified segment counts");
    const auto a = asymmetric.Stations(), b = changedOutlet.Stations();
    for (int i = 0; i <= inletCount+p.bendLayers; ++i)
      close(a[i], b[i], "Outlet rate changed inlet/bend distribution");
    require(std::fabs(a[inletCount+p.bendLayers+3]-b[inletCount+p.bendLayers+3]) > 1e-6,
            "Independent outlet rate did not change outlet spacing");

    // Exact geometric lengths must keep the specified multiplier everywhere.
    p = PipeParameters();
    const double h = BentPipe(p).BendSpacing();
    p.inletLayers = 10; p.outletLayers = 8;
    p.inletExtension = h*(std::pow(p.inletGrowthRate, 10)-1)/(p.inletGrowthRate-1);
    p.outletExtension = h*(std::pow(p.outletGrowthRate, 8)-1)/(p.outletGrowthRate-1);
    BentPipe geometric(p);
    checkStations(geometric);
    require(geometric.InletLayers() == 10 && geometric.OutletLayers() == 8,
            "Exact geometric layer count");
    const auto g = geometric.Stations();
    for (int i = 1; i <= 10; ++i)
      close(g[11-i]-g[10-i], h*std::pow(p.inletGrowthRate, i-1), "Exact growth multiplier");

    p = PipeParameters(); p.inletGrowthRate = p.outletGrowthRate = 1.;
    p.inletLayers = 10; p.outletLayers = 20;
    p.inletExtension = 10*h; p.outletExtension = 20*h;
    checkStations(BentPipe(p));
    p.outletExtension = 20.5*h;
    rejects([&] { BentPipe invalid(p); }, "Impossible fixed-spacing tail accepted");
    p = PipeParameters(); p.inletExtension = p.outletExtension = 0.;
    p.inletLayers = p.outletLayers = 0;
    BentPipe arcOnly(p);
    checkStations(arcOnly);
    require(arcOnly.TotalLayers() == p.bendLayers, "Zero-length straight segments add cells");
    p.inletExtension = h; p.outletExtension = h;
    p.inletLayers = p.outletLayers = 1;
    checkStations(BentPipe(p));
    p.inletExtension = 2*h;
    rejects([&] { BentPipe invalid(p); }, "Single straight cell differs from bend spacing");
    p.inletExtension = .5*h;
    rejects([&] { BentPipe invalid(p); }, "Straight shorter than interface cell accepted");
    p = PipeParameters(); p.bendAngleDegrees = 180.;
    p.inletLayers = p.outletLayers = 10;
    BentPipe hairpin(p);
    checkStations(hairpin);
    close(hairpin.At(hairpin.ArcLength()).tangent[0], -1., "180 degree outlet tangent");
    close(hairpin.At(hairpin.ArcLength()).center[2], 4., "180 degree bend end");
    p.inletExtension = p.outletExtension = 0.;
    p.inletLayers = p.outletLayers = 0;
    p.bendRadius = .5;
    rejects([&] { BentPipe(p).CheckRadius(.5); }, "Collapsed inner radius accepted");
    p.bendRadius = .4;
    rejects([&] { BentPipe(p).CheckRadius(.5); }, "Overlapping sections accepted");
    p = PipeParameters(); p.bendLayers = 0;
    rejects([&] { BentPipe invalid(p); }, "Zero bend layers accepted");
    p = PipeParameters(); p.inletLayers = 2;
    rejects([&] { BentPipe invalid(p); }, "Too few straight layers accepted");
    p = PipeParameters(); p.outletLayers = 100;
    rejects([&] { BentPipe invalid(p); }, "Too many straight layers accepted");
    p = PipeParameters(); p.inletLayers = 0;
    rejects([&] { BentPipe invalid(p); }, "Nonzero length with zero layers accepted");
    p = PipeParameters(); p.inletExtension = 0.;
    rejects([&] { BentPipe invalid(p); }, "Zero length with nonzero layers accepted");
    p = PipeParameters(); p.inletGrowthRate = .99;
    rejects([&] { BentPipe invalid(p); }, "Shrinking inlet spacing accepted");
    p = PipeParameters(); p.bendAngleDegrees = 0.;
    rejects([&] { BentPipe invalid(p); }, "Zero bend gives zero interface spacing");
    p = PipeParameters(); p.bendRadius = std::numeric_limits<double>::quiet_NaN();
    rejects([&] { BentPipe invalid(p); }, "NaN radius accepted");
    for (double rate : {1.001, 1.05, 1.5})
      for (int count : {2, 9, 40}) {
        p = PipeParameters();
        p.inletGrowthRate = p.outletGrowthRate = rate;
        p.inletLayers = p.outletLayers = count;
        const double maximum = h*(std::pow(rate, count)-1.)/(rate-1.);
        for (double fraction : {0., .1, .5, 1.}) {
          p.inletExtension = p.outletExtension = h*count+fraction*(maximum-h*count);
          checkStations(BentPipe(p));
        }
      }
    const PipeStations remap({2., 3., 4.}, {-2., 0., 3.});
    close(remap.At(2.5), -1., "Source z interpolation");
    close(remap.At(3.5), 1.5, "Curved axial point interpolation");
    std::cout << "Three-segment pipe geometry and spacing checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
