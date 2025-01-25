infile="wing.xml"
nrootcomp=4

echo "need to recompile Mesh2D and Merge3D codes before runing"
ln -s /home/agao/testcode/Mesh2D/build/ConvertXml .
ln -s /home/agao/testcode/Merge3D/build/Fix3DMesh .
ln -s /home/agao/testcode/Merge3D/params3D.h . # to use wingn

# uncompressed tip mesh
NekMesh ${infile} tip.xml:xml:uncompress -f 

# uncompressed root mesh
NekMesh -m extract:surf=${nrootcomp} ${infile} root.xml:xml:uncompress -f
# modify space dim to 2
sed -i '3d' root.xml
sed -i -e "3i\    <GEOMETRY DIM=\"2\" SPACE=\"2\">" root.xml
# fix root mesh
./ConvertXml root.xml fixroot.xml -f
wingn=`cat params3D.h | awk '{if($2=="wingn") print $4}'`
NekMesh -m extrude:layers=${wingn} fixroot.xml root3D.xml:xml:uncompress -f

# first merge mesh to gmsh format
./Fix3DMesh silent
# Fix nodes orders for gmsh mesh
NekMesh test.msh test2.xml:xml:uncompress -f
# obtain the final xml mesh
./Fix3DMesh gmsh test2.xml silent
ls fixed.xml
