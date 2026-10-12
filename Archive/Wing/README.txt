Merge3D：矩形与梯形机翼网格生成及验证经验
========================================

1. 当前几何约定
---------------
矩形源机翼：弦长 1、半展长 5、攻角 15 度。
矩形和梯形机翼厚度均为 1/30，前后缘均为半圆，半径均为 1/60。
梯形机翼：翼根弦长 1.5、翼尖弦长 0.5，后缘固定，弦长沿展向线性变化。
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
params3D.h：共享的展长、层数、翼尖间距和计算域。
paramsDeformed.h：源截面尺寸、攻角、梯形弦长和流体过渡范围。
TrapezoidWing.h：保持厚度、圆角及后缘的逐截面变换。
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
spanlength+hFirstLayer。第二块从外扩平面延伸到 domainz；第三块从
翼尖延伸到 domainz，底层保持实体轮廓，其余层边界通过 wallmapping
外扩。这样形成翼尖过渡层和可匹配的三块接口。

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
在翼身范围：
  c(z) = rootChord + (tipChord-rootChord)*z/spanlength
  D(z) = sourceChord-c(z)
  u_new = u + D(z)*w(u)*g(v)
超过翼尖时沿用翼尖弦长和变换，不再增加前缘位移。

近翼区域 g(v)=1。前缘半圆采用 w=1，整体平移；后缘半圆采用 w=0，
保持原位。中间平板段改变长度，法向坐标保持不变，因此厚度不变。
新前缘 u=sourceChord-c(z)，后缘始终为 sourceChord。
当前翼根前缘为 -0.5，翼尖前缘为 0.5。

完整圆角保护区由实际源半径确定：
  u<=leadingEdgeRadius：完整平移前缘半圆。
  u>=sourceChord-trailingEdgeRadius：固定后缘半圆。
  两者之间以 1-S(t) 连接，S(t)=3*t*t-2*t*t*t。
上游位移在 taperFarUpstream 到 taperNearUpstream 之间平滑增加到完整
位移；法向因子在 taperNearNormal 到 taperFarNormal 之间平滑衰减到零。
固定远场与内部网格需要平滑过渡，避免旧 swept() 仅冻结边界点、整体
移动其余点造成的突然位移变化。

DeformPts 同时修改顶点和 CURVED 中全部曲边插值点。只修改 VERTEX
会导致圆弧与端点不一致。当前追加的两块输入没有曲边；以后若追加
带曲边的块，需要核对合并后的曲边记录和编号，AddMeshRegion 主要
处理拓扑结构。

5. 等弦长恢复矩形的条件
-----------------------
rootChord=tipChord 时，得到等弦长矩形机翼。
只有 rootChord=tipChord=sourceChord 时，D(z)=0，变换完全为恒等映射。
同一批输入、相同层数和公共参数下，输出应与矩形结果一致。
两者都设为 1.5 得到弦长 1.5 的矩形翼，坐标与弦长 1 的基准不同。
当前矩形和梯形共有厚度 1/30、半径 1/60；验证仍应检查实际输入尺寸。

6. 本次遇到的问题与更新输入的方法
---------------------------------
早期矩形备份实际厚度为 0.05。二维参数和输入后来更新到 1/30，但旧
备份仍保持旧尺寸，梯形源半径也仍硬编码为 0.025。
必须分别核对参数要求、实际输入几何、输出文件和验证备份。

当前 paramsDeformed.h 中 sourceThickness=1/30，两个源半径由其一半
得到；目标厚度和目标半径也保留相同值。映射保持实际输入厚度，
仅修改这些数值不会自动改变旧 InputMesh 的实体尺寸。

修改二维厚度、攻角、弦长后，应先重生成 Mesh2D 外网格和内网格。
改变 wingn/tipn 后也必须重新拉伸。两个 make 脚本只检查拉伸文件是否
存在，已经存在的旧文件不会自动更新。
推荐顺序，在相应项目目录执行：
  Mesh2D：bash make.sh；bash makein.sh
  Merge3D/InputMesh：bash extrude.sh
  Merge3D：bash make.sh --yes；bash makeDeformed.sh --yes
同步 paramsDeformed.h 的源弦长、攻角和半径，使其匹配实际二维几何。
wingn/tipn 声明格式需兼容 extrude.sh 中按字段读取整数的 awk。

7. 验证、备份与可视化
---------------------
映射测试：
  ctest --test-dir build --output-on-failure

几何与拓扑比较，默认根部 1.5、翼尖 0.5、半径 1/60：
  python3 checks/verify_trapezoid_mesh.py test.xml deformed.xml
检查矩形本身尺寸和圆角：
  python3 checks/verify_trapezoid_mesh.py test.xml test.xml --root-chord 1 --tip-chord 1
其他半径可显式传入 --radius。
脚本核对所有翼身截面的弦长、厚度、圆角和曲边端点，并检查法向/展向
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
