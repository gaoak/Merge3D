if [ -d ../../Mesh2D/build ]; then
  cp ../../Mesh2D/build/wallmapping.dat  ../../Mesh2D/build/outerRegion.xml  ../../Mesh2D/build/outerRegion_Otip.xml  ../../Mesh2D/build/inFoil.xml .
fi
rm 3DoutUcomp.xml  3DoutUcomp2.xml  3DinUcomp.xml
wingn=`cat ../params3D.h | awk '{if($2=="wingn") print $4}'`
tipn=`cat ../params3D.h | awk '{if($2=="tipn") print $4}'`
tipn1=$((tipn+1))
NekMesh -m extrude:layers=${wingn} outerRegion.xml 3DoutUcomp.xml:xml:uncompress
NekMesh -m extrude:layers=${tipn}  outerRegion_Otip.xml 3DoutUcomp2.xml:xml:uncompress
NekMesh -m extrude:layers=${tipn1} inFoil.xml 3DinUcomp.xml:xml:uncompress
