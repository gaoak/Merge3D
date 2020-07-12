for((n=0;n<9;++n))
do
NekMesh -m extract:surf=${n} testc.xml surf${n}.xml
rm surf${n}.plt
FieldConvert surf${n}.xml surf${n}.plt
done
