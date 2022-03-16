git clone https://github.com/gaoak/Merge3D.git
cd Merge3D
git submodule init
git submodule update

To generate 3D mesh, second
bash make.sh

To check the mesh, run
bash extractSurf.sh





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
