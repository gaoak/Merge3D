#!/usr/bin/env python3
"""Check generated XML against its rectangular reference, without Nektar libs."""
import argparse
from collections import Counter, defaultdict
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def require(ok, message):
    if not ok:
        raise ValueError(message)


def indices(text):
    for part in text.strip().split("[", 1)[1].split("]", 1)[0].split(","):
        if "-" in part:
            first, last = map(int, part.split("-"))
            yield from range(first, last + 1)
        else:
            yield int(part)


def load(path):
    geometry = ET.parse(path).getroot().find("GEOMETRY")
    points = {int(v.get("ID")): tuple(map(float, v.text.split()))
              for v in geometry.find("VERTEX")}
    topology = {}
    for section in ("EDGE", "FACE", "ELEMENT", "COMPOSITE", "DOMAIN"):
        node = geometry.find(section)
        if section == "DOMAIN":
            topology[section] = " ".join(node.text.split())
        else:
            topology[section] = {
                v.get("ID", str(i)): (v.tag, " ".join(v.text.split()))
                for i, v in enumerate(node)}
    edges = {int(i): tuple(map(int, text.split()))
             for i, (_, text) in topology["EDGE"].items()}
    faces = {int(i): tuple(map(int, text.split()))
             for i, (_, text) in topology["FACE"].items()}
    curved = {int(v.get("EDGEID")): tuple(map(float, v.text.split()))
              for v in geometry.find("CURVED")}
    return points, topology, edges, faces, curved


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("reference")
    parser.add_argument("mesh")
    parser.add_argument("--root-chord", type=float, default=1.5)
    parser.add_argument("--tip-chord", type=float, default=.5)
    parser.add_argument("--tip-le-offset", type=float, default=.5)
    parser.add_argument("--source-chord", type=float, default=1.)
    parser.add_argument("--span", type=float, default=5.)
    parser.add_argument("--angle", type=float, default=15.)
    parser.add_argument("--radius", type=float, default=1./60.)
    parser.add_argument("--report")
    parser.add_argument("--preview")
    args = parser.parse_args()
    original, old_topology, _, _, old_curved = load(args.reference)
    points, topology, edges, faces, curved = load(args.mesh)
    require(topology == old_topology, "Topology or composite IDs changed")
    require(points.keys() == original.keys(), "Vertex IDs changed")
    require(curved.keys() == old_curved.keys(), "Curved edge IDs changed")
    tol = 2e-8  # Input NekMesh vertices are rounded to about 9 digits.
    co, si = math.cos(math.radians(args.angle)), math.sin(math.radians(args.angle))

    def local(p):
        return p[0] * co - p[1] * si, p[0] * si + p[1] * co

    def chord(z):
        t = max(0., min(1., z / args.span))
        return args.root_chord + t * (args.tip_chord - args.root_chord)

    def leading_at(z):
        return args.tip_le_offset * max(0., min(1., z / args.span))

    max_normal_change = 0.
    for i, p in points.items():
        u0, v0 = local(original[i])
        u, v = local(p)
        max_normal_change = max(max_normal_change, abs(v - v0))
        require(abs(v - v0) < tol and p[2] == original[i][2],
                f"Normal/span coordinate changed at vertex {i}")
        require(all(math.isfinite(x) for x in p), f"Non-finite vertex {i}")
        if (abs(original[i][0] + 40.) < 1e-10 or
                abs(original[i][0] - 60.) < 1e-10 or
                abs(original[i][1] + 40.) < 1e-10 or
                abs(original[i][1] - 40.) < 1e-10):
            require(p == original[i], f"Far-field vertex {i} moved")

    wall_faces = list(indices(topology["COMPOSITE"]["0"][1]))
    wall_vertices = {p for f in wall_faces for e in faces[f] for p in edges[e]}
    layers = defaultdict(list)
    cap_error = 0.
    for i in wall_vertices:
        u0, v0 = local(original[i])
        u, v = local(points[i])
        z = points[i][2]
        layers[z].append((u, v))
        if u0 <= args.radius + tol:
            error = abs(u - u0 - leading_at(z))
            cap_error = max(cap_error, error)
            require(error < tol, f"Leading cap distorted at vertex {i}")
        elif u0 >= args.source_chord - args.radius - tol:
            error = abs(u - u0 - (leading_at(z) + chord(z) - args.source_chord))
            cap_error = max(cap_error, error)
            require(error < tol, f"Trailing cap distorted at vertex {i}")
        else:
            require(abs(abs(v) - args.radius) < tol,
                    f"Straight wall thickness changed at vertex {i}")

    sections = []
    for z, layer in sorted(layers.items()):
        leading, trailing = min(u for u, _ in layer), max(u for u, _ in layer)
        thickness = max(v for _, v in layer) - min(v for _, v in layer)
        require(abs(leading - leading_at(z)) < tol,
                "Wrong section leading edge")
        expected_trailing = leading_at(z) + chord(z)
        require(abs(trailing - expected_trailing) < tol, "Wrong section trailing edge")
        require(abs(trailing - leading - chord(z)) < tol, "Wrong section chord")
        require(abs(thickness - 2. * args.radius) < tol, "Wrong section thickness")
        sections.append(dict(z=z, leading_edge=leading, trailing_edge=trailing,
                             chord=trailing-leading, thickness=thickness))
    require(0. in layers and args.span in layers, "Root or tip layer missing")
    root_le = min((i for i in wall_vertices if original[i][2] == 0.),
                  key=lambda i: original[i][0]**2 + original[i][1]**2)
    require(max(abs(v) for v in points[root_le]) < tol,
            "Root leading edge is not at the global origin")

    max_curve_error = 0.
    for eid, values in curved.items():
        original_values = old_curved[eid]
        require(len(values) == len(original_values), "Curved point count changed")
        for start, pid in ((0, edges[eid][0]), (len(values)-3, edges[eid][1])):
            error = max(abs(values[start+k] - points[pid][k]) for k in range(3))
            max_curve_error = max(max_curve_error, error)
            require(error < tol, f"Curved endpoint mismatch on edge {eid}")
        for i in range(0, len(values), 3):
            u, v = local(values[i:i+3])
            u0, v0 = local(original_values[i:i+3])
            z = values[i+2]
            require(abs(v - v0) < tol and z == original_values[i+2],
                    f"Curved normal/span coordinate changed on edge {eid}")
            if u0 <= args.radius + tol:
                center = args.radius + leading_at(z)
            else:
                require(u0 >= args.source_chord - args.radius - tol,
                        "This check expects circular leading/trailing curves")
                center = leading_at(z) + chord(z) - args.radius
            error = abs(math.hypot(u-center, v) - args.radius)
            max_curve_error = max(max_curve_error, error)
            require(error < tol, f"Curved radius changed on edge {eid}")

    usage = Counter(int(f) for _, text in topology["ELEMENT"].values()
                    for f in text.split())
    boundaries = Counter(f for _, text in topology["COMPOSITE"].values()
                         if text.startswith("F[") for f in indices(text))
    require(set(usage) == set(faces), "Missing or unreferenced faces")
    require(all(n in (1, 2) for n in usage.values()), "Non-manifold face")
    require({i for i, n in usage.items() if n == 1} == set(boundaries),
            "Boundary/internal face mismatch")
    require(all(n == 1 for n in boundaries.values()), "Repeated boundary face")
    report = dict(vertices=len(points), elements=len(topology["ELEMENT"]),
                  root_chord=args.root_chord, tip_chord=args.tip_chord,
                  tip_le_offset=args.tip_le_offset,
                  expected_thickness=2. * args.radius, expected_cap_radius=args.radius,
                  element_types=dict(Counter(tag for tag, _ in topology["ELEMENT"].values())),
                  topology_unchanged=True, boundary_faces=len(boundaries),
                  curved_edges=len(curved), max_normal_change=max_normal_change,
                  max_cap_translation_error=cap_error,
                  max_curved_radius_or_endpoint_error=max_curve_error,
                  sections=sections)
    text = json.dumps(report, indent=2)
    print(text)
    if args.report:
        Path(args.report).write_text(text + "\n")
    if args.preview:
        # Plot actual section vertices in the unrotated foil frame.
        def contour(z):
            center = leading_at(z) + chord(z)/2.
            return sorted(layers[z], key=lambda p: math.atan2(p[1], p[0]-center))
        root, tip = contour(0.), contour(args.span)
        def polygon(section, scale, ox, oy):
            return " ".join(f"{ox+scale*u:.3f},{oy-scale*v:.3f}" for u, v in section)
        le0, le1 = min(u for u, _ in root), min(u for u, _ in tip)
        te0, te1 = max(u for u, _ in root), max(u for u, _ in tip)
        svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="900" height="580" viewBox="0 0 900 580">
<rect width="900" height="580" fill="white"/>
<g font-family="sans-serif" fill="#243746">
<text x="40" y="40" font-size="23">Trapezoidal wing — generated mesh geometry</text>
<text x="40" y="70" font-size="15">Foil coordinates ({args.angle:g}° rotation removed); lengths in mesh units</text>
<text x="40" y="110" font-size="18">Planform</text>
<polygon points="100,{390-120*le0} 700,{390-120*le1} 700,{390-120*te1} 100,{390-120*te0}" fill="#d6eaf8" stroke="#2874a6" stroke-width="2"/>
<line x1="100" y1="390" x2="700" y2="390" stroke="#97a4ad" stroke-dasharray="5 5"/>
<circle cx="100" cy="390" r="4" fill="#243746"/>
<text x="100" y="145" font-size="15">Root: c = {args.root_chord:g}, z = 0</text>
<text x="700" y="145" font-size="15">Tip: c = {args.tip_chord:g}, z = {args.span:g}</text>
<text x="350" y="180" font-size="15">Root LE = 0; tip LE offset = {args.tip_le_offset:g}</text>
<text x="350" y="405" font-size="15">Span = {args.span:g}</text>
<text x="40" y="445" font-size="18">Root and tip sections (same scale)</text>
<polygon points="{polygon(root, 350., 120., 485.)}" fill="#d6eaf8" stroke="#2874a6" stroke-width="1.5"/>
<polygon points="{polygon(tip, 350., 120., 525.)}" fill="#d5f5e3" stroke="#239b56" stroke-width="1.5"/>
<text x="715" y="490" font-size="15">Root</text><text x="715" y="530" font-size="15">Tip</text>
<text x="40" y="565" font-size="15">Thickness = {2.*args.radius:.8g}; both cap radii = {args.radius:.8g}; contours use mesh vertices.</text>
</g></svg>'''
        Path(args.preview).write_text(svg)


if __name__ == "__main__":
    main()
