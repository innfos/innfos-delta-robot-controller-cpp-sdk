# innfos-delta-robot-controller-cpp-sdk


## 1.示例介绍与应用

示例代码编译

```cpp
$ cd innfos-delta-robot-controller-sdk/example

$ mkdir build

$ cmake ..

$ make
```


可执行文件存储在build文件夹下

2个demo

mode1:记录示教的点并记录

mode2:执行示教点的轨迹


## 2.常用函数介绍
### 类函数:DeltaKinematics(Delta正逆运动学计算)
主要应用以下:

delta基础参数:

```cpp
struct DeltaGeometricDim

{

  RealDataType sb;// base equilateral triangle side [ mm ]

  RealDataType sp;// platform equilateral triangle side [ mm ]

  RealDataType L;// upper legs length [ mm ]

  RealDataType l;// lower legs parallelogram length [ mm ]

  RealDataType h;// lower legs prallelogram width [ mm ]

  RealDataType max_neg_angle;// max negative angle that each arm can achive ( knee above the fixed-base plane ) [ deg ]

  RealDataType min_parallelogram_angle;// the limitation introduced by universal joints [ deg ]
                       	
};
```
             
  delta逆运动学计算:
  
  ```cpp
  int CalculateIpk(DeltaVector *v, int num);
  
  //v: a pointer to the matrix of cartesian positions and joints vectors, which only joints are changed
  
  //num: a number of vectors in the matrix
  ```

  delta正运动学计算:
  
  ```cpp
  int CalculateFpk(DeltaVector *v, int num);
  
  //v: a pointer to the matrix of cartesian positions and joints vectors, which only cartesian positions are changed
  
  //num: a number of vectors in the matrix
  ```



### 类函数:speed_profile(s型曲线)
主要应用以下:

   设置基本的s型曲线的参数:
   
   ```cpp
   void set_parameters(double s,double f,double fs,double fe,double A,double D,double J)


   //s: position

   //f: max velocity

   //fs: initial velocity

   //fe: final velocity

   //A: max acceleration

   //D: min acceleration

   //J: max jerk
   
   ```

   计算S型曲线分段时间:
   
   ```cpp
   void calculate_jerk_limit_profile_time()
   ```

   计算S型曲线的插补点:
   
   ```cpp
   void calculate_current_speed(double t,double PAVJ[])

   //t: time

   //PAVJ[0]: velocity

   //PAVJ[1]: position
   ```




### 类函数:DeltaController(delta控制器)

主要应用以下:

   记录delta示教的点:
   
   ```cpp
   void subscribeStartRecordEFF();
   ```

   保存delta记录的点:
   
   ```cpp
   void subscribeStartWriteEFF();
   ```

   计算delta末端插补:
   
   ```cpp
   void subscribeFKReadData_scurve(double s_vmax,double s_vinit ,double s_vend,double s_amax,double s_amin,double s_jerk);
   ```

   播放delta示教的插补点:
   
   ```cpp
   void IKPlaybackCommand_Teach_scurve();
   ```

   读取delta基本机构参数:
   
   ```cpp
   void getDeltaGeometricDim(DeltaKinematics<double>::DeltaGeometricDim test_robot_dim);
   ```






