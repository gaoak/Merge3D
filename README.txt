git clone https://github.com/gaoak/Merge3D.git
cd Merge3D
git submodule init
git submodule update

To generate the rectangular 3D mesh (test.xml and testcomp.xml):
bash make.sh

To generate the trapezoidal 3D mesh (deformed.xml and deformedcomp.xml):
bash makeDeformed.sh
Both scripts accept --yes to skip the interactive confirmation.

To inspect the rectangular mesh surfaces, run
bash extractSurf.sh

The default executable and make.sh generate the rectangular mesh from
InputMesh. Shared span/layer settings remain in params3D.h. Deformation is
enabled only by mesh --deformed or makeDeformed.sh. In paramsDeformed.h,
rootChord=1.5 and tipChord=0.5 vary linearly over spanlength. The current source
InputMesh is a unit-chord plate at 15 degrees, thickness 0.05, with both
rounding radii 0.025. sourceChord, foilAngleDegrees and the rounding radii
must match the input mesh. Taper changes the straight section length in the
foil coordinate system; the trailing edge stays fixed, the leading cap is
translated, and thickness and both circular caps retain their dimensions.
For z beyond the wing tip, the tip transformation is used. Fluid deformation
smoothly vanishes outside the blend extents specified in paramsDeformed.h.

To generate without the interactive prompt:
./build/mesh --yes
To choose the output or explicitly select a mesh type:
./build/mesh --yes --deformed --output trapezoid.xml
./build/mesh --yes --rectangular --output rectangular.xml
To check the mapping:
ctest --test-dir build --output-on-failure
To check the generated high-order mesh and write the compressed mesh:
NekMesh -m jac:list deformed.xml deformedcomp.xml -f
To verify section geometry and unchanged topology against a saved reference:
python3 checks/verify_trapezoid_mesh.py test.xml deformed.xml





full wing:
C0, wing wall
C7, z=5 wing tip
C8, z=-5 wing tip
---------
C1, inlet
C2, outlet
C3, upper surface
C4, lower surface
C5, side z=-7
C6, side z=7

================================
half wing:
C0, wing wall
C7, z=5 wing tip
----------
C1, inlet
C2, outlet
C3, upper surface
C4, lower surface
C5, symmetry plane, z=0
C6, side, z=7
