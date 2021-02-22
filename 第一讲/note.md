## VIO概述

## 0.IMU与视觉进行比较
| IMU   |  视觉
| :-----:|:---:|
惯性测量单元|利用图像的VIO
六自由度IMU，陀螺仪测量角加速度、加速度计测量加速度|利用图像通过特征、像素（直接法）进行位姿估计|
高频>=100hz,应对载体位姿变化比较快的情况|以图像形式记录数据，频率在15hz-60hz，载体位姿不能变化太快，受外界环境影响|
|快速响应，不受成像质量影响，角速度普遍估计比较准确，可估计绝对尺度|不产生漂移、直接测量旋转与平移|
存在零偏，低精度IMU积分位姿发散，高精度价格昂贵|受图像遮挡、运动物体干扰；单目视觉无法测量尺度；单目纯旋转运动无法估计；快速运动容易丢失|
适合短时间、运动快时比较准|适合长时间、运动慢时估计准
 
利用视觉信息估计IMU的零偏，减少IMU由零偏导致的发散和累积误差；利用IMU为视觉提供快速运动时的定位。
    IMU虽然可以测得角速度和加速度，但这些量都存在明显的漂移（Drift），使得积分两次得到的位姿数据非常不可靠。好比说，我们将IMU放在桌上不动，用它的读数积分得到的位姿也会漂出十万八千里。但是，对于短时间内的快速运动，IMU能够提供一些较好的估计。这正是相机的弱点。当运动过快时，（卷帘快门的）相机会出现运动模糊，或者两帧之间重叠区域太少以至于无法进行特征匹配，所以纯视觉SLAM非常害怕快速的运动。而有了IMU，即370第14讲SLAM：现在与未来使在相机数据无效的那段时间内，我们还能保持一个较好的位姿估计，这是纯视觉SLAM无法做到的。
    相比于IMU，相机数据基本不会有漂移。如果相机放在原地固定不动，那么（在静态场景下）视觉SLAM的位姿估计也是固定不动的。所以，相机数据可以有效地估计并修正IMU读数中的漂移，使得在慢速运动后的位姿估计依然有效。
    当图像发生变化时，本质上我们没法知道是相机自身发生了运动，还是外界条件发生了变化，所以纯视觉SLAM难以处理动态的障碍物。而IMU能够感受到自己的运动信息，从某种程度上减轻动态物体的影响。
    总而言之，我们看到IMU为快速运动提供了较好的解决方式，而相机又能在慢速运动下解决IMU的漂移问题——在这个意义下，它们二者是互补的。

## 1.IMU与多种定位方案融合
- 自动驾驶中通常用 IMU+GPS/差分 GPS/RTK 的融合定位方案,形成 GNSS-INS 组合导航系统,达到厘米组定位精度。
- 头戴式 AR/VR 头盔则多使用视觉 +IMU 的 VIO 定位系统,形成高帧率定位方案。

## 2.融合方案
- 松耦合：卡尔曼滤波
- 紧耦合：MSCKF、非线性优化

## 3.预备知识
0. 三维刚体运动
![](https://img-blog.csdnimg.cn/20210222154411797.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl8zODIxNjU3Mw==,size_16,color_FFFFFF,t_70)
坐标系为from I to W
![](https://img-blog.csdnimg.cn/20210222154717184.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl8zODIxNjU3Mw==,size_16,color_FFFFFF,t_70)
1. 四元数
![](https://img-blog.csdnimg.cn/2021022215500944.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl8zODIxNjU3Mw==,size_16,color_FFFFFF,t_70)
具体运算详见SLAM14讲
**利用单位四元数可以表达三维空间内的旋转**
*从四元数为什么能表示三维旋转去理解它有的运算*
模长为1的复数可以表示复平面内的纯旋转；四元数与复数类似，单位四元数（模长为1、取模操作）可以表示三维内的旋转。
四元数不唯一，相同的旋转可以由互为共轭（共轭操作）的四元数表示。
连续旋转，四元数相乘，单位四元数的乘积还是单位四元数。
旋转的逆，四元数的逆，单位四元数的逆和共轭是一样的。（**求逆的时候简便操作**）
叉积结果中含有叉乘项，所以四元数相乘不能交换位置。
* ** 用四元数表示旋转在程序代和数学表示有一些细微的差别。**
例如,通过运算符重载,四元数和三维向量可以直接计算乘法,但在数学上则需要先把向量转成虚四元数,再利用四元数乘法进行计算,同样的情况也适用于变换矩阵乘三维向量的情况。总体而言,程序中的用法会比数学公式更灵
活一些。

https://blog.csdn.net/shao918516/article/details/105305525
对于理解是：q1*q2得到实部不为0的普通四元数，这时没办法映射到三维超平面（总不能转着圈就转到了四维空间吧），结果与最初的点不在同一三维空间。为了解决这个问题，先对第四维（角度）旋转一半，再用逆或共轭旋转回来，这时正好将产生的第四维变为0，重新回到初始的三维超平面空间。
![在这里插入图片描述](https://img-blog.csdnimg.cn/2021022223251912.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl8zODIxNjU3Mw==,size_16,color_FFFFFF,t_70#pic_center)
程序中使用
SO3的matrix形式和eigen下的旋转矩阵值相同，但是重载了opearator *运算
```cpp
Sophus::SO3d R2_SO3=R1_so3*Sophus::SO3d::exp(w);//小量w对应李代数
```
![eigen和Sophus库的使用，需要注意拉！！！](https://img-blog.csdnimg.cn/20190109150118920.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L2ZiXzk0MTIxOQ==,size_16,color_FFFFFF,t_70)
对旋转点和旋转矩阵求雅可比的注意点（都是对向量求雅可比，所以对连续旋转矩阵求雅可比时要先转换成向量，再对某一旋转矩阵乘上小量）
Q： PPTp31页，左右交换取矩阵，符号相反

## 4.习题解答
[https://blog.csdn.net/hitljy/article/details/107320682?utm_medium=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-3.control&dist_request_id=09cd2f58-ed1c-455b-b651-62cdc347e517&depth_1-utm_source=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-3.control](https://blog.csdn.net/hitljy/article/details/107320682?utm_medium=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-3.control&dist_request_id=09cd2f58-ed1c-455b-b651-62cdc347e517&depth_1-utm_source=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-3.control) 

1.2 有哪些常见的视觉 +IMU 融合方案？有没有工业界应用的例子？

VINS (单目+IMU、双目+IMU)
OKVIS (单目+IMU、双目+IMU)
ROVIO (单目+IMU）
RKSLAM (单目+IMU）
ORB_SLAM-IMU（单目+IMU）

AR/VR，自动驾驶，无人机，手机、无人机拍照防抖
### 重点关注李群的性质，看清是关于旋转点的雅可比还是连续旋转矩阵的雅克比
每一微小干扰都是作用在R上的，R^(-1)p----->(Rexp(w))^(-1)*p

![习题3](https://img-blog.csdnimg.cn/20210222233318944.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl8zODIxNjU3Mw==,size_16,color_FFFFFF,t_70#pic_center)
![习题4](https://img-blog.csdnimg.cn/20210222233328304.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl8zODIxNjU3Mw==,size_16,color_FFFFFF,t_70#pic_center)

## 5.我的补充理解等等
 
旋转向量θu，u是旋转轴向量，θ是向量的长度，也就是旋转的角度
旋转向量和旋转矩阵的变换公式、罗德里格斯公式等
https://blog.csdn.net/shao918516/article/details/105109278 
该链接解释的非常详细清楚
