弯曲圆管内部网格（channel 分支）
================================

本分支只生成正圆截面的管内网格。旧机翼程序和说明保存在 Archive/Wing，
不参与当前编译。主程序 mesh.cpp 只负责读取参数并调用弯管生成流程。

1. 输入与运行
-------------
使用用户放入 InputMesh/inFoil.xml 的最新二维圆内网格，不自动从
../Mesh2D 复制文件，也不使用旧 wallmapping.dat。
输入必须是正圆内部的二维网格。圆心、半径和单元数量从当前输入读取，
不会使用说明中的历史计数作为生成约束。

执行：
  bash make.sh

脚本依次：
  1) CMake 编译；
  2) NekMesh extrude 拉伸最新的二维输入，始终重新生成
     InputMesh/pipeStraight.xml；
  3) 沿弯管中轴线重映射顶点、高阶边和侧面；
  4) NekMesh 检查实际高阶网格的负 Jacobian；
  5) 核对圆截面、曲面/边一致性和拓扑，成功后发布 pipecomp.xml。

输出：
  pipe.xml：未压缩的弯管网格。
  pipecomp.xml：通过检查的压缩网格。
  axis.csv：各截面的中轴线弧长、间距、中心、切向与法向。
  pipe_summary.json：参数、实际截面数和管径。
  verification/pipe_jacobian.log：NekMesh 检查日志。
  verification/pipe_geometry.json：实际几何和拓扑验证结果。

单独重新拉伸：
  bash InputMesh/extrude.sh
  # 拉伸层数由 ./build/mesh --layers 输出，等于三段指定层数之和。
直接处理已有直管输入：
  ./build/mesh
已有直管的实际层数必须与当前三段分布的总层数一致，否则报错并提示重新拉伸。
新旧三维输入不要混用；正常运行 make.sh 会重新拉伸。

2. 参数文件
-----------
几何和中轴线网格参数集中在 params3D.h：

  bendAngleDegrees = 90       两端切向的总转角，当前支持 0 < angle <= 180 度。
  bendRadius = 2              中轴线曲率半径。
  inletExtension = 10         圆弧前的直段长度。
  outletExtension = 20        圆弧后的直段长度。
  inletLayers = 34            入口直段的单元数量。
  bendLayers = 32             弯曲段的均匀单元数量。
  outletLayers = 45           出口直段的单元数量。
  inletGrowthRate = 1.06      从弯曲段入口向上游的间距增长倍率上限。
  outletGrowthRate = 1.06     从弯曲段出口向下游的间距增长倍率上限。
  geometryPoints = 6          高阶边每边点数、曲面每方向点数的下限。

三段层数均明确指定，总拉伸层数 = inletLayers + bendLayers + outletLayers。
原 extrudeLayers、centerSpacing、growthRate 参数已移除。
输入 params3D.h 的参数后，运行 make.sh 即可同步编译、拉伸和重映射。
文件名与脚本配套：二维输入 inFoil.xml，直管输入 pipeStraight.xml，
最终输出 pipe.xml。改变路径时需同步脚本和检查器。

管内半径由输入圆截面读取，并检查所有侧壁顶点位于同一正圆上，
所有拉伸平面包含相同的二维截面。不会另外缩放输入管径。

3. 轴向加密规则
---------------
中轴线分成入口直段、弯曲段和出口直段。以弯曲段入口为 s=0，
入口直段 s<0，弯曲段 0<=s<=L_b，出口直段 s>L_b。
L_b = R*alpha，其中 alpha 使用弧度。

弯曲段按 bendLayers 个单元均匀划分：
  h = L_b / bendLayers
弯曲段两端都有明确的网格平面，不会有单元跨越直段/弯曲段交界。
弯曲段单元数为奇数时，不强制添加中点截面。

两侧直段均从交界处向外排布，第一层间距严格等于 h。
分别按 inletGrowthRate、outletGrowthRate 增长：
  ds_k = h*q^(k-1)
入口使用 inletLayers 层，出口使用 outletLayers 层，增长率不改变指定层数。
当下一步增长超过“剩余长度 / 剩余层数”时，转为等距尾段，
准确到达指定端点。不会整体缩放网格或改变交界处的第一层间距。
因此 q 是增长倍率上限：主体指数增长，尾段倍率为 1；接入尾段的
那一步倍率可能小于 q。若直段长度恰好等于完整几何级数，全部层都
保持倍率 q。入口沿流动方向向弯曲段逐渐加密，出口向外逐渐加粗。
两侧增长率可以不同，改变一侧参数不会改变另一侧或弯曲段的分布。

直段长度为 0 时，其层数也必须为 0；非零长度必须指定正层数。
长度 L、层数 N、首层 h 和增长率 q 的可行性条件是：
  N*h <= L <= h*(1+q+...+q^(N-1))
另外 N=1 时必须 L=h，才能保证唯一一层与弯曲段间距匹配。
如果 N*h>L，减少该直段层数、增大 bendLayers 或增大直段长度；
如果完整几何级数仍达不到 L，增加该直段层数、增长率或减小直段长度。
增长率必须 >=1；q=1 时必须 L=N*h。程序拒绝不满足约束的参数，
不会自动改变层数或首层间距。弯曲角必须 >0，才能得到非零 h。

4. 中轴线与正圆截面映射
-----------------------
中轴线位于 x-z 平面。弯曲段入口为原点，入口直段沿 +x 方向流入。
圆弧向 +z 方向转弯，曲率圆心为 (0,0,R)。总弯角为 alpha，
在圆弧内 theta=s/R：
  C(s) = (R*sin(theta), 0, R*(1-cos(theta)))
  T(s) = (cos(theta), 0, sin(theta))
  N(s) = (sin(theta), 0, -cos(theta))
N 是朝向曲率圆外侧的截面法向，B=(0,1,0)，满足 N x B = T。
入口 C(s)=(s,0,0)、T=(1,0,0)；出口沿圆弧末端切向直线延伸。
90 度弯管的出口沿 +z，180 度弯管的出口沿 -x。
圆弧与两直段连接处位置、切向连续，曲率从 1/R 变为 0，
不额外加入过渡曲线。

二维截面扣除自身圆心后，映射为：
  P(x,y,s) = C(s) + x*N(s) + y*B

T 和 N 正交且长度为 1，截面只整体旋转和平移。因此截面保持原半径，
每个截面垂直于中轴线，不进行径向拉伸，也没有额外绕轴扭转。
原 NekMesh 的 z 平面先对应到物理弧长 s；高阶点在相邻实际平面间按
弧长插值，不能把轴向曲线的中间点简单四舍五入到某个网格平面。

5. 高阶边、曲面与单元
---------------------
只移动 VERTEX 不足以得到正确的弯管几何。PipeMesh.cpp 会同时建立：
  周向 E 曲线：各截面上的正圆弧。
  轴向 E 曲线：弯曲区所有轴向边，含内部边，沿 C+x*N+y*B 取样。
  F 曲面：弯曲区管壁侧面，同时具有周向圆弧和轴向弯曲，
           使用 N*N 的张量积点阵（默认每面 6*6 个点）。

内侧面对应二维直边，依靠两侧轴向曲线的高阶插值形成弯曲面；
管壁需要显式 F 点阵，才能表示周向和轴向两种曲率的耦合。
直段上的轴向边保持直线，直段圆柱侧面由周向圆弧和直边描述。
管径和截面正圆是映射约束；Nektar 使用有限阶多项式表示这些曲线/面，
在采样点之间仍是高阶近似，geometryPoints 与源圆弧点数决定几何阶数。
实际使用的点数不低于源圆弧的点数。

曲面点采用 Nektar 四边形局部顶点顺序 0,1,2,3；u 方向变化最快。
检查器逐条核对 F 点阵四边与对应 E 曲线一致，以及 E 端点与顶点一致。

CAD3D 修改集中在 Nektarpp.h/.cpp：
  支持带参数的 std::function 变形回调；
  同时处理 E 和 F 的所有插值点，F 不按边曲线反转；
  大曲面点阵使用动态文本缓冲；
  缺少 CURVED 时安全处理，并检查 XML 读写和插值点数量。
中轴线/分布在 BentPipe.h，正圆扫掠和曲面构造在 PipeMesh.cpp，
原 MeshRegion 的点/边/面/单元拓扑结构继续复用。
原有 LoadXml、DeformPts(void*) 接口保留；新增回调接口可以携带参数。
旧接口与新接口的曲边/顶点结果一致性，以及大 F 点阵读写，有回归测试。
错误处理有所变化：XML 读写失败现在抛出异常，需要调用方处理。
坐标文本改为动态缓冲和科学计数法，数值可往返读取，但文件字节格式
不保证与旧版本一致。

6. 内侧重叠与质量检查
---------------------
若管内半径为 r，必须保证 R>r，否则最内侧曲率半径 R-r 非正，
法向截面会塌缩、翻转或重叠。程序根据实际输入半径检查这一条件，
不满足时停止生成。

圆弧区连续映射对 (x,y,s) 的 Jacobian 为 1+x/R（当前 N 朝外）；
最小几何因子为 1-r/R。直段因子为 1。
样例 r=0.5、R=2，内侧半径为 1.5，最小连续因子约 0.75。
这是连续映射条件，不等于实际单元的最小缩放 Jacobian；半径接近 r
时，即使条件成立，仍可能产生非常扁的内侧单元。
make.sh 必须通过实际高阶 NekMesh 检查和几何核对后才发布压缩输出。

测试：
  ctest --test-dir build --output-on-failure
  python3 checks/verify_pipe_mesh.py
  NekMesh -m jac:list pipe.xml verification/pipecheck.xml -f
需要详细质量统计时：
  NekMesh -m jac:list:histo=1,10,10:quality pipe.xml verification/pipecheck.xml -f

实际层数、单元数、曲边与曲面数随输入网格和参数变化，请查看
pipe_summary.json、verification/pipe_geometry.json 和 Jacobian 日志。
检查器核对三段层数、均匀弯曲段、两侧首层匹配、独立增长倍率、
入口 +x 方向、各截面正圆与垂直性，以及曲面和边曲线的一致性。

7. 边界与 MHD 使用
------------------
C0：全部圆管壁面。
C1：入口（最小 s 的截面）。
C2：出口（最大 s 的截面）。
C100、C101：六面体/棱柱体区域，不与边界编号冲突。
程序保留入口、出口和管壁各自独立的 composite，不按圆弧转角重新合并。

这里输出几何网格。MHD 的场变量、磁场方向、流动边界条件和导电壁/
绝缘壁条件应在求解器的 session 中设置。几何插值阶数与求解场的
NUMMODES 是不同设置，当前输出的初始 EXPANSIONS 用于读取/查看网格。

可视化：
  FieldConvert pipecomp.xml pipe.plt -f
  bash extractSurf.sh
extractSurf.sh 提取 C0/C1/C2 到 pipe_wall、pipe_inlet、pipe_outlet 文件。
