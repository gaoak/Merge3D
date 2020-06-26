for n in  9 10
do
NekMesh -m extract:surf=${n} test.xml surf${n}.xml
rm surf${n}.plt
FieldConvert surf${n}.xml surf${n}.plt
done
