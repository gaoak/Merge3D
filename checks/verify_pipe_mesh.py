#!/usr/bin/env python3
"""Verify the actual swept circular sections, high-order curves and topology."""
import argparse
from collections import Counter
import csv
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET


def require(ok, message):
    if not ok:
        raise ValueError(message)


def ids(text):
    for token in text.strip().split("[", 1)[1].split("]", 1)[0].split(","):
        if "-" in token:
            a, b = map(int, token.split("-"))
            yield from range(a, b+1)
        else:
            yield int(token)


def load(filename):
    g = ET.parse(filename).getroot().find("GEOMETRY")
    result = {"points": {int(v.get("ID")): tuple(map(float, v.text.split()))
                          for v in g.find("VERTEX")}}
    for name in ("EDGE", "FACE", "ELEMENT"):
        result[name] = {int(v.get("ID")): (v.tag, tuple(map(int, v.text.split())))
                        for v in g.find(name)}
    result["boundaries"] = {int(v.get("ID")): list(ids(v.text))
                            for v in g.find("COMPOSITE")
                            if v.text.strip().startswith("F[")}
    result["domains"] = [i for v in g.find("COMPOSITE")
                          if not v.text.strip().startswith("F[") for i in ids(v.text)]
    result["curves"] = {"E": {}, "F": {}}
    for v in g.find("CURVED"):
        values = list(map(float, v.text.split()))
        require(len(values) == 3*int(v.get("NUMPOINTS")), "Wrong curved point count")
        points = [tuple(values[i:i+3]) for i in range(0, len(values), 3)]
        result["curves"][v.tag][int(v.get("EDGEID" if v.tag == "E" else "FACEID"))] = points
    return result


def face_vertices(mesh, fid):
    edges = mesh["FACE"][fid][1]
    result = []
    for i, edge in enumerate(edges):
        a = mesh["EDGE"][edge][1]
        b = mesh["EDGE"][edges[(i+1) % len(edges)]][1]
        result.append(a[1] if a[0] in b else a[0])
    return result


def distance(a, b):
    return math.sqrt(sum((x-y)**2 for x, y in zip(a, b)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", default="InputMesh/pipeStraight.xml")
    parser.add_argument("--mesh", default="pipe.xml")
    parser.add_argument("--axis", default="axis.csv")
    parser.add_argument("--summary", default="pipe_summary.json")
    parser.add_argument("--jacobian-log")
    parser.add_argument("--report", default="verification/pipe_geometry.json")
    args = parser.parse_args()
    if args.jacobian_log:
        log = Path(args.jacobian_log).read_text()
        match = re.search(r"Total negative Jacobians:\s*(\d+)", log)
        require(match is not None and int(match.group(1)) == 0,
                "NekMesh did not confirm zero negative Jacobians")
    source, target = load(args.source), load(args.mesh)
    metadata = json.loads(Path(args.summary).read_text())
    with Path(args.axis).open() as f:
        axis = [{key: float(value) for key, value in row.items()}
                for row in csv.DictReader(f)]
    require(len(axis) == metadata["layers"]+1, "Incorrect actual plane count")
    entry = metadata["inlet_layers"]
    exit = entry + metadata["bend_layers"]
    require(exit + metadata["outlet_layers"] == metadata["layers"],
            "Three segment counts do not match actual extrusion layers")
    arc_length = metadata["bend_radius"]*math.radians(metadata["bend_angle_degrees"])
    spacing = arc_length/metadata["bend_layers"]
    require(abs(spacing-metadata["bend_spacing"]) < 1e-10, "Wrong bend spacing")
    require(axis[entry]["s"] == 0. and abs(axis[exit]["s"]-arc_length) < 1e-10,
            "Missing exact bend entry/exit planes")
    for i in range(entry+1, exit+1):
        require(abs(axis[i]["s"]-axis[i-1]["s"]-spacing) < 1e-10,
                "Bend cells are not uniform")
    for start, side, count, growth in (
            (entry, -1, entry, metadata["inlet_growth_rate"]),
            (exit, 1, metadata["outlet_layers"], metadata["outlet_growth_rate"])):
        if not count:
            continue
        first = side*(axis[start+side]["s"]-axis[start]["s"])
        require(abs(first-spacing) < 1e-10, "Straight/bend interface spacing differs")
        previous = first
        geometric_length = first
        for i in range(2, count+1):
            ds = side*(axis[start+side*i]["s"]-axis[start+side*(i-1)]["s"])
            require(previous-1e-10 <= ds <= previous*growth+1e-10,
                    "Straight spacing decreases or exceeds its growth rate")
            previous = ds
            geometric_length += spacing*growth**(i-1)
        length = metadata["inlet_extension" if side < 0 else "outlet_extension"]
        require(spacing*count <= length+1e-10 and geometric_length >= length-1e-10,
                "Straight length cannot fit its specified count, spacing and growth rate")
    require(target["points"].keys() == source["points"].keys(), "Vertex IDs changed")
    for kind in ("EDGE", "FACE", "ELEMENT"):
        require(target[kind] == source[kind], f"{kind} topology changed")

    R = metadata["bend_radius"]
    r = metadata["pipe_radius"]
    require(R > r, "Inner curvature radius is not positive")
    require(abs(axis[0]["s"]+metadata["inlet_extension"]) < 1e-10,
            "Incorrect inlet extension")
    require(abs(axis[-1]["s"]-arc_length-metadata["outlet_extension"]) < 1e-10,
            "Incorrect outlet extension")

    def frame(s):
        arc = max(0., min(arc_length, s))
        phi = arc/R
        t = (math.cos(phi), 0., math.sin(phi))
        n = (math.sin(phi), 0., -math.cos(phi))
        c = (R*math.sin(phi)+(s-arc)*t[0], 0.,
             R*(1.-math.cos(phi))+(s-arc)*t[2])
        return c, t, n

    for row in axis[:entry+1]:
        require(abs(row["center_x"]-row["s"]) < 1e-10 and
                abs(row["center_y"])+abs(row["center_z"]) < 1e-10 and
                abs(row["tangent_x"]-1.)+abs(row["tangent_y"])+abs(row["tangent_z"]) < 1e-10,
                "Inlet centerline is not parallel to +x")

    plane_map = {row["source_z"]: row["s"] for row in axis}
    source_planes = sorted(plane_map)
    from bisect import bisect_left

    def arc_at(z):
        i = bisect_left(source_planes, z)
        if i < len(source_planes) and abs(source_planes[i]-z) < 1e-8:
            return plane_map[source_planes[i]]
        if i > 0 and abs(source_planes[i-1]-z) < 1e-8:
            return plane_map[source_planes[i-1]]
        require(0 < i < len(source_planes), "Curved z outside source range")
        a, b = source_planes[i-1], source_planes[i]
        q = (z-a)/(b-a)
        return plane_map[a]*(1.-q)+plane_map[b]*q

    tol = 1e-6
    maximum_error = 0.
    maximum_radius_error = 0.
    for row in axis:
        c, t, n = frame(row["s"])
        for name, expected in (("center", c), ("tangent", t), ("normal", n)):
            measured = tuple(row[f"{name}_{i}"] for i in "xyz")
            require(distance(measured, expected) < 1e-10, "Incorrect saved centerline frame")
    for vid, p in source["points"].items():
        c, t, n = frame(arc_at(p[2]))
        x, y = p[0]-metadata["source_center_x"], p[1]-metadata["source_center_y"]
        expected = (c[0]+x*n[0], y, c[2]+x*n[2])
        error = distance(expected, target["points"][vid])
        maximum_error = max(maximum_error, error)
        require(error < tol, f"Wrong section transform at vertex {vid}")
        offset = tuple(target["points"][vid][i]-c[i] for i in range(3))
        require(abs(sum(offset[i]*t[i] for i in range(3))) < tol,
                "Section is not perpendicular to the centerline")
        require(abs(math.sqrt(sum(v*v for v in offset))-math.hypot(x, y)) < tol,
                "Cross-section was scaled or distorted")

    for eid, points in target["curves"]["E"].items():
        a, b = source["EDGE"][eid][1]
        require(distance(points[0], target["points"][a]) < tol and
                distance(points[-1], target["points"][b]) < tol,
                f"Curved edge {eid} endpoints do not match its vertices")
        pa, pb = source["points"][a], source["points"][b]
        if abs(pa[2]-pb[2]) < 1e-8:
            c, t, _ = frame(arc_at(pa[2]))
            for point in points:
                offset = tuple(point[i]-c[i] for i in range(3))
                error = abs(math.sqrt(sum(v*v for v in offset))-r)
                maximum_radius_error = max(maximum_radius_error, error)
                require(error < tol and abs(sum(offset[i]*t[i] for i in range(3))) < tol,
                        "Circumferential curve is not a perpendicular circle")
        else:
            for j, point in enumerate(points):
                q = j/(len(points)-1)
                raw = tuple(pa[k]*(1.-q)+pb[k]*q for k in range(3))
                c, _, n = frame(arc_at(raw[2]))
                x, y = raw[0]-metadata["source_center_x"], raw[1]-metadata["source_center_y"]
                require(distance(point, (c[0]+x*n[0], y, c[2]+x*n[2])) < tol,
                        "Axial curve does not follow the centerline frame")

    for fid, points in target["curves"]["F"].items():
        n = math.isqrt(len(points))
        require(n*n == len(points), "Wall face points are not a tensor-product grid")
        corners = face_vertices(source, fid)
        a, b, c, d = (source["points"][i] for i in corners)
        for j in range(n):
            for i in range(n):
                u, v = i/(n-1), j/(n-1)
                z = (1.-u)*(1.-v)*a[2]+u*(1.-v)*b[2]+u*v*c[2]+(1.-u)*v*d[2]
                center, tangent, _ = frame(arc_at(z))
                point = points[j*n+i]
                offset = tuple(point[k]-center[k] for k in range(3))
                error = abs(math.sqrt(sum(q*q for q in offset))-r)
                maximum_radius_error = max(maximum_radius_error, error)
                require(error < tol and abs(sum(offset[k]*tangent[k] for k in range(3))) < tol,
                        f"Wall face {fid} departs from the circular sweep")
        for k, eid in enumerate(source["FACE"][fid][1]):
            curve = target["curves"]["E"].get(eid)
            require(curve is not None and len(curve) == n, "Missing compatible wall-face edge curve")
            if source["EDGE"][eid][1][0] != corners[k]:
                curve = list(reversed(curve))
            boundary = ([points[i] for i in range(n)] if k == 0 else
                        [points[j*n+n-1] for j in range(n)] if k == 1 else
                        [points[(n-1)*n+n-1-i] for i in range(n)] if k == 2 else
                        [points[(n-1-j)*n] for j in range(n)])
            require(max(distance(p, q) for p, q in zip(boundary, curve)) < tol,
                    f"Wall face {fid} is inconsistent with edge {eid}")

    usage = Counter(f for _, fs in target["ELEMENT"].values() for f in fs)
    require(set(usage) == set(target["FACE"]), "Unused or missing faces")
    require(all(n in (1, 2) for n in usage.values()), "Non-manifold faces")
    require(set(target["boundaries"]) == {0, 1, 2}, "Expected C0 wall, C1 inlet, C2 outlet")
    boundaries = Counter(f for fs in target["boundaries"].values() for f in fs)
    require(all(n == 1 for n in boundaries.values()), "Repeated boundary assignment")
    require(set(boundaries) == {f for f, n in usage.items() if n == 1},
            "Undefined exterior face or internal face in boundary")
    for boundary, faces in target["boundaries"].items():
        for f in faces:
            z = [source["points"][i][2] for i in face_vertices(source, f)]
            if boundary:
                expected_z = source_planes[0 if boundary == 1 else -1]
                require(all(abs(q-expected_z) < 1e-8 for q in z), "Inlet/outlet composite mislabeled")
            else:
                require(max(z)-min(z) > 1e-8, "End cap in wall composite")
    require(Counter(target["domains"]) == Counter({i: 1 for i in target["ELEMENT"]}),
            "Volume composites do not cover each cell exactly once")
    report = dict(metadata, topology_unchanged=True,
                  max_vertex_mapping_error=maximum_error,
                  max_curved_radius_error=maximum_radius_error,
                  curved_edges=len(target["curves"]["E"]),
                  curved_faces=len(target["curves"]["F"]),
                  boundary_face_counts={str(i): len(fs) for i, fs in target["boundaries"].items()},
                  negative_jacobians=0 if args.jacobian_log else None)
    output = Path(args.report)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2)+"\n")
    print(f"Verified {metadata['layers']+1} perpendicular circular sections; "
          f"{metadata['elements']} cells, {report['curved_faces']} curved wall faces.")


if __name__ == "__main__":
    main()
