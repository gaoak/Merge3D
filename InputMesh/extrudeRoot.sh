rm 3DoutUcomp.xml  3DoutUcomp2.xml  3DinUcomp.xml 3Droot.xml
wingn=`cat ../params3D.h | awk '{if($2=="wingn") print $4}'`
NekMesh -m extrude:layers=${wingn} root.xml 3Droot.xml:xml:uncompress
