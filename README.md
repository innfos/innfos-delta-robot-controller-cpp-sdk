# innfos-delta-robot-controller-cpp-sdk


## 1.示例介绍与应用
cd innfos-delta-robot-controller-sdk/example

mkdir build

cmake ..

make

可执行文件存储在build文件夹下

2个demo

mode1:记录示教的点并记录

mode2:执行示教点的轨迹


## 2.常用函数介绍
### 类函数:DeltaKinematics(Delta正逆运动学计算)
主要应用以下:

delta基础参数:struct DeltaGeometricDim

{

  RealDataType sb;					  // base equilateral triangle side [ mm ]

  RealDataType sp;					  // platform equilateral triangle side [ mm ]

  RealDataType L;						  // upper legs length [ mm ]

  RealDataType l;						  // lower legs parallelogram length [ mm ]

  RealDataType h;						  // lower legs prallelogram width [ mm ]

  RealDataType max_neg_angle;			  // max negative angle that each arm can achive ( knee above the fixed-base plane ) [ deg ]

  RealDataType min_parallelogram_angle; // the limitation introduced by universal joints [ deg ]
                       	
};
             
  逆运动学:int CalculateIpk(DeltaVector *v, int num);

  正运动学:int CalculateFpk(DeltaVector *v, int num);



### 类函数:speed_profile(s型曲线)
主要应用以下:

   设置基本的s型曲线的参数:void set_parameters(double s,double f,double fs,double fe,double A,double D,double J)

   其中各种参数含义如下

   s:   position

   f:   max velocity

   fs:  initial velocity

   fe:  final velocity

   A:   max acceleration

   D:   min acceleration

   J:   max jerk


   计算S型曲线分段时间:void calculate_jerk_limit_profile_time()

   计算S型曲线的插补点:void calculate_current_speed(double t,double PAVJ[])

   其中各种参数含义如下

   t: time

   PAVJ[0]: velocity

   PAVJ[1]: position




### 类函数:DeltaController(delta控制器)

主要应用以下:

   记录delta示教的点:void subscribeStartRecordEFF();

   保存delta记录的点:void subscribeStartWriteEFF();

   计算delta末端插补:void subscribeFKReadData_scurve(double s_vmax,double s_vinit ,double s_vend,double s_amax,double s_amin,double s_jerk);

   播放delta示教的插补点:void IKPlaybackCommand_Teach_scurve();

   读取delta基本机构参数:void getDeltaGeometricDim(DeltaKinematics<double>::DeltaGeometricDim test_robot_dim);






