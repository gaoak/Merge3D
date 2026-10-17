Merge3D：矩形与梯形机翼网格生成及验证经验
========================================

1. 当前几何约定
---------------
矩形源机翼：弦长 1、半展长 5、攻角 15 度。
矩形和梯形机翼厚度均为 1/30，前后缘均为半圆，半径均为 1/60。
梯形机翼：翼根弦长 1.5、翼尖弦长 0.5；当前翼尖前缘偏移为 0。
翼根前缘固定为 (0,0,0)，前缘和后缘位置均沿展向线性变化。
z=0 是翼根，z=5 是翼尖；当前计算域另一侧位于 z=9。

物理厚度来自 ../Mesh2D/params.h 的 Thickness；BLFlatPlate 使用
Thickness/2 作为前后缘半圆半径。二维参数中的
LEradius = Thickness/2 + upperBL0 等变量包含边界层厚度，不能当作
实体圆角半径。

厚度 0.05、半径 0.025 属于旧输入几何，不是当前验证基准。
verification/rectangular/ 和 rectangular_reference_20261010_222139 中的
旧备份属于这一版本。后续比较应使用已验证的厚度 1/30 的新矩形备份。

2. 文件职责与运行入口
---------------------
make.sh：矩形入口，输出 test.xml、testcomp.xml。
makeDeformed.sh：梯形入口，输出 deformed.xml、deformedcomp.xml。
mesh.cpp：共用的重映射、拼接、输出主程序；默认矩形，--deformed
启用梯形，--rectangular 显式选择矩形。
params3D.h：唯一的参数头文件，统一配置展长、层数、翼尖间距、
翼尖外侧增长倍率、计算域、源截面尺寸、攻角、rootChord、tipChord、
tipLEOffset 和流体网格的过渡范围。
TrapezoidWing.h：固定翼根前缘、保持厚度和圆角的逐截面变换。
CAD3D/Nektarpp.cpp：XML 读取、展向重映射、顶点和曲边变形。
CAD3D/MeshRegion.cpp：点/边/面拼接、边界与体区域整理、拓扑检查。
CAD3D/LineEdge.cpp、Util.cpp：展向非均匀分布和辅助函数。

获取与运行：
  git clone https://github.com/gaoak/Merge3D.git
  cd Merge3D
  git submodule init
  git submodule update
  bash make.sh --yes
  bash makeDeformed.sh --yes
省略 --yes 时，打印展向坐标后等待输入 c。
直接运行：
  ./build/mesh --yes
  ./build/mesh --yes --deformed
  ./build/mesh --yes --rectangular --output rectangular.xml

3. 二维输入、拉伸与三块拼接
-------------------------
二维源文件来自 ../Mesh2D/build：
outerRegion.xml：翼身外围网格。
outerRegion_Otip.xml：翼尖之后的外围网格。
inFoil.xml：填充翼尖之后截面内部区域的网格。
wallmapping.dat：壁面轮廓到外扩一层轮廓的对应关系。

InputMesh/extrude.sh 复制二维源文件，用 NekMesh 拉伸得到：
3DoutUcomp.xml：wingn 层。
3DoutUcomp2.xml：tipn 层。
3DinUcomp.xml：tipn+1 层。

mesh.cpp 将归一化展向坐标映射到非均匀 z 分布。
翼身壁面末层位于 spanlength，其余末层点延伸到
spanlength+tipoffset。第二块从外扩平面延伸到 domainz；第三块从
翼尖延伸到 domainz，底层保持实体轮廓，其余层边界通过 wallmapping
外扩。这样形成翼尖过渡层和可匹配的三块接口。

翼尖外侧的 z 向增长倍率由 params3D.h 的 tiphzgrowth 指定，当前为 1.6，
必须为有限值且 >=1。翼尖后的第一层厚度为 tipoffset；后续边界层
间距依次为 tipoffset*tiphzgrowth、tipoffset*tiphzgrowth^2 等。
tipoffset 默认等于 hFirstLayer；单独修改 tipoffset 不改变翼身段的
末层间距 hFirstLayer。此参数对矩形和梯形两个入口都生效。
现有 LineEdge 最多对外侧前 7 层采用指数增长，随后将剩余长度均分；
如果下一次增长会超过剩余长度/剩余层数，则提前转为等距尾段。
tiphzgrowth 控制指数增长部分，不保证尾段交界处仍保持该倍率。
调整倍率不会改变 tipn、spanlength 或 domainz；修改参数后运行相应
make 脚本重新编译、映射即可，wingn/tipn 未变时无需重新拉伸二维输入。

拼接按点、边、面、单元的顺序进行。重复接口面从边界集合中删除，
随后整理边界 composite 和体区域。先完成拼接，再统一变形，使共享
接口顶点只变换一次。

4. 梯形映射与圆角保护
---------------------
不能整体缩放弦向坐标，否则半圆会变成椭圆。
变换在机翼局部弦向 u、法向 v 中进行，保留原攻角。
Mesh2D 的坐标约定：
  x = u*cos(AoA) + v*sin(AoA)
  y = -u*sin(AoA) + v*cos(AoA)
逆变换：
  u = x*cos(AoA) - y*sin(AoA)
  v = x*sin(AoA) + y*cos(AoA)

源前缘 u=0，源后缘 u=sourceChord。变换只改变 u，保持 v、z。
形状由 params3D.h 的三个参数指定，长度单位与源网格相同：
  rootChord：翼根弦长。
  tipChord：翼尖弦长。
  tipLEOffset：翼尖前缘相对翼根前缘沿剖面弦向 +u 的偏移量。
正值表示朝后缘方向收缩，负值表示前掠；它不是全局 x 方向的距离。
在翼身范围，t=z/spanlength：
  c(z) = rootChord + (tipChord-rootChord)*t
  LE(z) = tipLEOffset*t
  TE(z) = LE(z)+c(z)
翼根 LE=0、TE=rootChord，翼尖 LE=tipLEOffset、TE=tipLEOffset+tipChord。
因此翼根前缘始终位于全局 (0,0,0)，无需平移整个计算域。
当 0<tipLEOffset<rootChord-tipChord 时，前后缘都向内收缩；
tipLEOffset=0 保持前缘沿展向不变；tipLEOffset=rootChord-tipChord
保持后缘沿展向不变（其位置为 rootChord，不再默认等于 sourceChord）。
超过翼尖时沿用翼尖位置和弦长，保持三块接口一致。

示例参数 rootChord=1.5、tipChord=0.5、tipLEOffset=0.5：
翼根局部范围 [0,1.5]，翼尖局部范围 [0.5,1.0]，前后缘各收缩 0.5。
局部尖点 (u,0,z) 的全局坐标为 (u*cos(AoA),-u*sin(AoA),z)。
例如翼尖前缘为 (0.5*cos(15deg),-0.5*sin(15deg),5)。

两个半圆分别整体平移，禁止整体缩放弦向坐标，否则半圆会变成椭圆：
  dLE(z) = LE(z)
  dTE(z) = TE(z)-sourceChord
  u<=leadingEdgeRadius：完整平移前缘半圆，位移 dLE。
  u>=sourceChord-trailingEdgeRadius：完整平移后缘半圆，位移 dTE。
两个半圆之间，令 q=(u-leadingEdgeRadius)/
(sourceChord-leadingEdgeRadius-trailingEdgeRadius)，S(q)=3*q*q-2*q*q*q：
  du = dLE*(1-S(q))+dTE*S(q)
这只改变中间平板段的长度，法向坐标不变，厚度和两个圆角半径均保留。

上游位移在 taperFarUpstream 到 taperNearUpstream 之间从零增加到 dLE；
下游位移在 taperNearDownstream 到 taperFarDownstream 之间从 dTE 衰减到零。
默认上游范围 [-5,-1]、下游范围 [2,6]，均按源局部弦向坐标指定。
法向因子在 taperNearNormal 到 taperFarNormal 之间从一衰减到零。
这些过渡保持远场不动，并避免只冻结边界点造成位移突变。
JacobianLowerBound 同时检查两圆角之间、上游和下游过渡的压缩；
位移过大时应调整形状或加宽对应过渡区，再检查实际高阶单元。

DeformPts 同时修改顶点和 CURVED 中全部曲边插值点。只修改 VERTEX
会导致圆弧与端点不一致。当前追加的两块输入没有曲边；以后若追加
带曲边的块，需要核对合并后的曲边记录和编号，AddMeshRegion 主要
处理拓扑结构。

5. 等弦长恢复矩形的条件
-----------------------
rootChord=tipChord 且 tipLEOffset=0 时，得到无后掠的等弦长矩形机翼。
若弦长相同但 tipLEOffset 非零，则是等弦长的前掠/后掠机翼。
只有 rootChord=tipChord=sourceChord 且 tipLEOffset=0 时，变换完全为
恒等映射。同一批输入、相同层数和公共参数下，输出应与矩形结果一致。
rootChord=tipChord=1.5、tipLEOffset=0 会得到弦长 1.5、前缘仍在 u=0
的矩形翼。矩形和梯形都保持实际源网格的厚度和圆角。

6. 本次遇到的问题与更新输入的方法
---------------------------------
早期矩形备份实际厚度为 0.05。二维参数和输入后来更新到 1/30，但旧
备份仍保持旧尺寸，梯形源半径也仍硬编码为 0.025。
必须分别核对参数要求、实际输入几何、输出文件和验证备份。

当前 params3D.h 中 sourceThickness=1/30，两个源半径由其一半
得到；目标厚度和目标半径也保留相同值。映射保持实际输入厚度，
仅修改这些数值不会自动改变旧 InputMesh 的实体尺寸。

修改二维厚度、攻角、弦长后，应先重生成 Mesh2D 外网格和内网格。
改变 wingn/tipn 后也必须重新拉伸。两个 make 脚本只检查拉伸文件是否
存在，已经存在的旧文件不会自动更新。
推荐顺序，在相应项目目录执行：
  Mesh2D：bash make.sh；bash makein.sh
  Merge3D/InputMesh：bash extrude.sh
  Merge3D：bash make.sh --yes；bash makeDeformed.sh --yes
同步 params3D.h 的源弦长、攻角和半径，使其匹配实际二维几何。
wingn/tipn 声明格式需兼容 extrude.sh 中按字段读取整数的 awk。

7. 验证、备份与可视化
---------------------
映射测试：
  ctest --test-dir build --output-on-failure

几何与拓扑比较，默认根部 1.5、翼尖 0.5、前缘偏移 0.5、半径 1/60：
  python3 checks/verify_trapezoid_mesh.py test.xml deformed.xml
检查矩形本身尺寸和圆角：
  python3 checks/verify_trapezoid_mesh.py test.xml test.xml --root-chord 1 --tip-chord 1 --tip-le-offset 0
其他形状可显式传入 --root-chord、--tip-chord、--tip-le-offset；
其他半径可显式传入 --radius。
脚本核对翼根前缘原点、所有翼身截面的前后缘位置、弦长、厚度、
圆角和曲边端点，并检查法向/展向
坐标、固定远场、拓扑、composite 编号及边界定义。

实际网格的 Jacobian：
  NekMesh -m jac:list deformed.xml deformedcomp.xml -f
makeDeformed.sh 已执行此检查。更详细报告：
  NekMesh -m jac:list:histo=1,10,10:quality deformed.xml deformedcomp.xml -f
CheckMesh() 主要检查拓扑和边界，不检查负 Jacobian 或单元翻转。
连续映射的 JacobianLowerBound()>0 不能替代实际高阶网格检查。

每份矩形基准应保存未压缩/压缩 XML、params3D.h、对应的二维参数和
源文件，记录实际厚度、半径及 SHA256。
新基准保存于 verification/rectangular_reference_1_30_<时间戳>/。
verification/current_rectangular_reference.txt 记录当前正确基准的路径。
保留旧备份并明确标记其尺寸和历史版本身份。

可视化：
  FieldConvert testcomp.xml testcomp.plt -f
  FieldConvert deformedcomp.xml deformedcomp.plt -f
  bash extractSurf.sh
extractSurf.sh 默认从 testcomp.xml 提取 surf0 到 surf7，即矩形网格。
查看梯形用 deformedcomp.plt，或另外从 deformedcomp.xml 提取表面。
重新生成 XML 后，旧 plt/surf 文件不会自动更新。

当前半翼边界编号：
C0：翼身壁面；C7：翼尖壁面，z=5。
C1：入口 x=-40；C2：出口 x=60。
C3：下边界 y=-40；C4：上边界 y=40。
C5：翼根/对称平面 z=0；C6：侧边界 z=9。
改变源网格边界拆分方式后需要重新核对编号。
