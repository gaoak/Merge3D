for((n=0;n<=8;++n))
do
NekMesh -m extract:surf=${n} testcomp.xml surf${n}.xml
rm surf${n}.plt
FieldConvert surf${n}.xml surf${n}.plt
done
