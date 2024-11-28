# Awesome-Robot-Learning

> 机器人学习整个仿真框架一般包括以下几部分：3D仿真物理引擎（用于对真实物理环境的近似模拟仿真）、机器人仿真平台（用于搭建工作场景，以实现智能体与环境的交互学习）、学习算法框架集合（不同的策略学习算法的标准实现集合）、算法测试环境及基准（针对不同的定制化算法的测试环境和评价基准）。下面将从以上四方面进行调研总结。

---

## 1. 3D仿真物理引擎

物理引擎是一个计算机程序用来模拟牛顿动力学模型，使用质量、速度、摩擦力和空气阻力等变量，为刚性或柔性物体赋予真实的物理属性的方式来模拟物体的运动、旋转和碰撞等交互过程，使得仿真的效果更准确、更真实。基于牛顿三大定律，在计算机中通过物理引擎来模拟刚体运动的流程大致都是如下左图所示。其中显示结果这部分中，物理引擎驱动图形渲染的流程如下右图所示。
![avatar](/assets/images/framework_1.png)
![avatar](/assets/images/framework_2.png)

物理引擎有两种类型常见的型类：实时物理引擎和高精度物理引擎。高精度物理引擎需要更多的处理能力来计算非常精确的物理，侧重于精确计算，仿真结果精度高，通常使用在科学研究（计算物理学）和电脑动画电影制作，比如ADAMS、ANSYS等。实时物理引擎使用通常使用在电子游戏并且简化运算，降低精确度增以减少计算时间，得到在电子游戏当中可以接受的处理速度，比如UnrealEngine、Bullet等。

下面将对目前比较知名流行的物理引擎进行介绍和分析。

>（1）Havok(Havok Game Dynamics SDK)
>>官网：<https://www.havok.com/>
>>
>>介绍：Havok引擎是Havok公司开发的一款物理引擎，有不少游戏和软件都选择了他做物理引擎，比如HALO3、失落星球、HL2、 细胞分裂、指环王Online等等。如今Havok被Intel收购了，以后可能对Intel的CPU会有特别的优化。Havok对PS2、XBOX、GameCube、PC多种游戏平台都有支持。也是世界顶级游戏公司Valve（Half Life的公司），Pandemci，Remedy等的合作伙伴。这个物理引擎曾经支持过各种类型的游戏，包括racing game，first-persion shooter，MMOGs，adventure games，puzzle games等等。Hovak还曾经负责电影Matrix的部分效果处理。
>>
>>特点：支持以下功能：
Collision Detection - including Continuous Physics™;
MOPP™ Technology - for compact representation of large collision meshes;
Dynamics and Constraint Solving;
Vehicle Dynamics;
Data Serialization and Art Tool Support;
Visual Debugger for in-game diagnostic feedback.
>
>（2）PhysX(AGEIA NovodeX)
>>官网：<https://developer.nvidia.com/gameworks-physx-overview>
>>
>>介绍：NovodeX是由开发PPU（物理加速卡）的公司AGEIA进行维护，因此对于将来PPU硬件的支持，无疑NovodeX是最有优势的。NovodeX是一个模拟刚体动力学的物理引擎，支持速度，加速度，动量，冲量，碰撞等等的物理概念。NovodeX的开发库支持跨平台，多线程，高速碰撞检测等特性，专门对汽车物理的模拟做了优化。因为特有的硬件卡支持，所以能处理大量的物理运算，其他几款暂时没得比，目前AGEIA公司己被NVIDIA收购。Unreal3，GameBryo，Reality Engine等多款商业引擎和游戏都使用了他。
>>
>>特点：支持以下功能：
Massively Parallel Physics Architecture;
High-speed GDDR3 Memory Interface;
AGEIA Universal Continuous Collision Detection;
AGEIA Physical Smart Particle Technology;
AGEIA Complex Object Physics System;
AGEIA Scalable Terrain Fidelity;
AGEIA Dynamic Gaming Framework.
>
>（3）ODE(Open Dynamics Engine)
>>官网：<https://www.ode.org/>
>>
>>介绍：ODE(Open Dynamics Engine，开源动力学引擎)是一个著名免费的具有工业品质的用于模拟关节连接的刚体动力学的库，由Russell Smith在一些贡献者的帮助下开发而成。经过验证的应用场景包括在虚拟显示环境中模拟地面车辆，腿形生物和移动物体。它具有快速、灵活、健壮的特点，并具有内置的碰撞检测功能。
>>
>>特点：支持以下功能：
Rigid bodies with arbitrary mass distribution;
Joint types: ball-and-socket, hinge, slider (prismatic), hinge-2, fixed, angular motor, linear motor, universal;
Collision primitives: sphere, box, cylinder, capsule, plane, ray, and triangular mesh, convex;
Collision spaces: Quad tree, hash space, and simple;
Simulation method: The equations of motion are derived from a Lagrange multiplier velocity based model due to Trinkle/Stewart and Anitescu/Potra;
A first order integrator is being used. It's fast, but not accurate enough for quantitative engineering yet. Higher order integrators will come later;
Choice of time stepping methods: either the standard ``big matrix'' method or the newer iterative QuickStep method can be used;
Contact and friction model: This is based on the Dantzig LCP solver described by Baraff, although ODE implements a faster approximation to the Coloumb friction model;
Has a native C interface (even though ODE is mostly written in C++);
Has a C++ interface built on top of the C one;
Many unit tests, and more being written all the time;
Platform specific optimizations.
>
>（4）Bullet(Bullet Physics SDK)
>>官网：<https://github.com/bulletphysics/bullet3>
>>
>>介绍：Bullet物理引擎是开源的，专业的集刚体、软体和碰撞检测于一身的动力学类库。Bullet的特性还包括快速和稳定的刚体动力约束和求解、动态车辆、人物控制和滑动器、铰链、普通的自由度和针对碎布木偶的圆锥和扭曲约束。这款物理引擎的历史也比较久了，但似乎国内知道的ODE的人更多一些，这款物理引擎被Nvidia的开发人员所关注（Nvidia前些时候说过，要用GPU来实现物理加速，可能会最先在这款物理引擎上实现。）
>>
>>特点：开源霸主，支持以下功能：
Multi Platform support;
Supports various shape types;
Discrete Collision Detection for Rigid Body Simulation;
Single Queries;
Sweep and Prune Broadphase;
Documentation and Support;
Auto generation of MSVC project files,comes with Jam build system;
Bullet Collision Detection works with Bullet Dynamics,but there is also a sample integration with Open Dynamics Engine;
Framework with 2 different Constraint Solvers;
Hinge,Point to Point Constraint,Twist Cone Constraint (ragdolls);
Automatic de-activation (sleeping);
Generic 6 Degree of Freedom Constraint , Motors, Limits;
LCP Warm starting of contact points;
Collada 1.4 Physics Import using FCollada and COLLADA-DOM;
Convex Decomposition Code.
>
>（5）Newton(Newton Dynamics)
>>官网：<http://newtondynamics.com/forum/newton.php>
>>
>>介绍：Newton引擎主要应用于3D游戏开发中，其快速方便的碰撞检测和关节构架等功能使其非常适合于机器人仿真环境的建立，避免了手工复杂的力学计算，并能提供逼真的仿真结果。这款物理引擎更多的专注于生活中的实例模拟，名声可能不是很响，但是功能上绝对不差。比较出名的作品有TV3D，Quest3D等。
>>
>>特点：支持以下功能：
Joints - Overview and examples for all the joint types that are offered;
Collision primitives - Overview of available collision shapes;
Materials - Overview of Newton Materials;
Buoyancy - Overview of how buoyancy works in Newton, and tips for building buoyant bodies.
>
>（6）Vortex(Vortex Studio)
>>官网：<https://www.cm-labs.com/>
>>
>>介绍：VORTEX是加拿大CM_LABS公司的旗舰产品，是一款工程级别的实时交互式动力学仿真软件，强调准确性和实时性并重。Vortex软件作为全世界最优秀的实时交互动力学仿真工具，具有优秀的数学模型和高效的计算方法，成功实现工程计算准确性和虚拟现实仿真实时性。VORTEX可以考虑任意复杂拓扑系统的多体动力学，自动进行快速稳定准确的干涉检查与碰撞检测，原生支持OSG视景引擎，也可以集成任意的图形引擎。
>>
>>特点：支持以下功能(<http://www.osgchina.org/show-list.php?id=64>)：
便于构建复杂的机械装置系统动力学并可视化显示机械装置，涉及应用领域广泛，包含军用车辆、工程机械、水面、水下设备等；
不管是在桌面程序还是自定义的多通道模拟器上，Vortex皆可提供视景、动作和声音的仿真；
不管工作环境是在海底油田还是在农田、战场，Vortex皆可再现操作环境；
应用Vortex直接验证仿真性能和仿真精度，快速排除故障，修正问题；
支持动力学核心开发、Vortex图形化用户(GUI)建模、Simulink接口、人体仿真、土方工程、缆索系统、海洋等多种专业领域模块。
>
>（7）DART(Dynamic Animation and Robotics Toolkit)
>>官网：<https://dartsim.github.io/>
>>
>>介绍：DART（动态动画和机器人工具箱）是由乔治亚理工学院的图形实验室和仿人机器人实验室创建的一个协作、跨平台的开放源码库。该库为机器人技术和计算机动画中的运动学和动力学应用提供数据结构和算法。由于DART使用广义坐标来表示铰接刚体系统，并使用Featherstone的铰接体算法来计算运动动力学，因此它的精度和稳定性得到了显著的提高。DART在机器人学和计算机动画中有着广泛的应用，因为它具有多体动力学模拟器和各种用于控制和运动规划的运动学工具。
>>
>>特点：支持以下功能(<https://dartsim.github.io/>)：
支持多种平台：Ubuntu，Archlinux，FreeBSD，macOS和Windows；
提供可扩展的API以解决各种优化问题，例如非线性编程和多目标优化；
支持多种碰撞检测器：FCL，Bullet和ODE；
支持各种碰撞形状，包括原始形状，凹面网格和概率体素网格；
通过可定制的惯性和材料属性，支持众多原始的和任意的身体形状；
提供对任意实体和坐标系的运动状态（例如，变换，位置，速度或加速度）的全面访问；
即插即用的分层整体逆运动学求解器；
使用李群表示法和Featherstone混合算法为铰接式动态系统实现高性能；
为动态量及其派生提供全面的API，例如质量矩阵，科里奥利力，重力，其他外力和内力；
使用隐式LCP处理接触和碰撞，以确保不穿透，定向摩擦以及近似的库仑摩擦锥条件；
提供多个约束求解器：Lemke方法，Dantzig方法和PSG方法；
支持“孤岛”技术来细分约束处理以提高性能。
>
>（8）SimBody(Multibody Physics API)
>>官网：<https://simtk.org/projects/simbody>
>>
>>介绍：Simbody是一种高性能的开放源代码工具包，用于对关节机制进行科学和工程质量的仿真，包括生物力学结构（例如人和动物的骨骼），机械系统（例如机器人，车辆和机器）以及任何其他可以描述的内容一组由关节相互连接，受力和运动影响且受约束约束的刚体。Simbody包括一个多体动力学库，用于建模O(n)时间中的广义/内部坐标下的运动。有时称为Featherstone样式的物理引擎。该工具集最初是由斯坦福大学Simbios中心的Michael Sherman开发的，由Peter Eastman和其他人做出了重要贡献，其完全在GitHub上开源，且完全支持Windows，Mac OSX或Linux系统。
>>
>>特点：支持以下功能：
Wide variety of joint, constraint, and force types; easily user-extended;
Forward, inverse, and mixed dynamics. Motion driven by forces or prescribed motion;
Contact (Hertz, Hunt and Crossley models);
Gradient descent, interior point, and global (CMA) optimizers;
A variety of numerical integrators with error control;
Visualizer, using OpenGL.
>
>（9）MuJoCo(Multi-Joint dynamics with Contact)
>>官网：<http://www.mujoco.org/>
>>
>>介绍：MuJoCo是由Emo Todorov为Roboti LLC而开发的一种物理引擎，旨在促进机器人技术，生物力学，图形和动画以及其他需要快速而准确的仿真的领域的研究和开发。它提供了速度，准确性和建模能力的独特组合，但它不仅仅是一个更好的模拟器。相反，它是第一个从头开始设计的全功能模拟器，其目的是基于模型的优化，尤其是通过接触进行的优化。MuJoCo使扩大计算密集型技术（例如最佳控制，物理一致状态估计，系统识别和自动化机制设计）成为可能，并将其应用于具有丰富接触行为的复杂动态系统。它还具有更多传统应用程序，例如在物理机器人上部署之前测试和验证控制方案，交互式科学可视化，虚拟环境，动画和游戏。
>>
>>特点：支持以下特性：
在广义坐标系中进行仿真，避免出现关节冲突；
即使存在接触也可以很好地定义逆动力学；
通过凸优化对约束进行统一的连续时间表述；
约束包括软接触，极限，干摩擦，等式约束；
能模拟粒子系统，布料，绳索和软物体；
包括马达，圆柱体，肌肉，腱，滑块曲柄的执行器；
可选择牛顿，共轭梯度或投影高斯-赛德尔求解器；
可选择金字塔形或椭圆形的摩擦锥，密集或稀疏的雅可比方程式；
可选择Euler或Runge-Kutta数值积分器；
多线程采样和有限差分近似；
直观的XML模型格式（称为MJCF）和内置的模型编译器；
跨平台GUI在OpenGL中具有交互式3D可视化；
用ANSI C编写并针对性能进行手工调整的运行时模块。
>
>（10）RaiSim
>>官网：<https://raisim.com/>
>>
>>介绍：RaiSim是RaiSim Tech Inc.开发的用于机器人技术和AI的跨平台多体物理引擎，其设计目的是为模拟机器人系统提供准确性和速度。但是，它是通用的刚体模拟器，可以非常有效地仿真任何刚体。它是开源的，且完全支持Linux，Mac Os和Windows系统。
>>
>>特点：支持以下功能：
The speed is benchmarked against other popular physics engine (https://leggedrobotics.github.io/SimBenchmark/);
The accuracy of RaiSim has been demonstrated in a number of papers;
Easiest C++ simulation library to learn/use;
A minimum number of dependencies (only on STL and Eigen).
>
>（11）SPE(Simple Physics Engine)
>>官网：<http://spehome.com/>
>>
>>介绍：目前国产的物理引擎精品，能够实现基本的物理仿真，尤其在碰撞检测和仿真方面比较突出。
>>
>>特点：支持以下功能(<http://spehome.com/>)：
使用独创的快速而稳定的Tri-Mesh碰撞检测算法，使载入模型数据异常简单。SPE的碰撞检测系统从一开始就是针对三角形网格（Tri-Mesh）而设计，所以用户可以方便地使用mesh文件创建任意形状的刚体，SPE内部将自动处理所有工作。同时，SPE支持球和胶囊两种基本几何形状，方便用户创建粒子特效和ragdoll系统。此外，SPE支持一定条件下的连续碰撞检测，可以正确地处理大多数情况下的高速运动物体；
碰撞信息分析。SPE对碰撞检测系统产生的数据进行智能化分析，为碰撞反应计算提供更可靠更正确的原始数据，极大地提高了系统的稳定性；
稳定的碰撞与接触解决系统。从1.5版开始，SPE采用全新的解决算法，更正确地计算摩擦与反弹，而且更稳定；
SPE提供一种稳定的基本Joint功能，支持最大距离、弹性系数以及破坏力等参数的配置，用户可以使用它方便地创建各种其他类型的Joint；
实时刚体破碎(Beta)。SPE提供“形状操作”的功能，任何模型均可被一组平面或另一个模型切成小块，SPE生成的模型中包括用于区分原始表面与切面的属性信息，方便用户更合理地渲染出新的形状；
高并行计算。SPE已经完成了多线程化以充分利用多核心CPU的性能。90％以上的计算任务都可均匀地分配到任意数量的线程中去。与单线程相比，双线程至少能提供60％的性能提升，而四线程可以带来150％以上的性能提升。使用SPEWorld::SetNumThreads()即可在任何时候开启多线程计算；
简单易用而人性化的接口，极大地降低了SPE与其他软件系统结合的难度，使用户在瞬间即可建立一个具有真实物理属性的世界。
>
>总结：通过调研发现，针对机器人学习领域，从开源程度、开发文档详细、计算实时性能、仿真效果、支持模块、流行领域来评价[12]，比较适合的物理引擎有：PhysX，Bullet，MuJoCo，RaiSim。下面的仿真平台有很多就采用上述物理引擎来进行实时计算。

---

## 2. 机器人仿真平台

机器人系统设计离不开仿真工具的支持。机器人仿真让我们在没有物理硬件的情况下也可以快速对算法进行验证；或者提高安全性，避免实验损伤我们的设备（比如在增强学习中，就需要大量random的exploration）。一般来说机器人仿真工具在物理引擎之上进行包装，如基于ODE、Bullet等。有些情况下我们只需要使用物理引擎就可以满足需要，但一般情况下我们也想通过可视化平台观察机器人运行的正确性。仿真一般只在系统前期使用，因为真实物理平台与仿真环境存在差异，后期还是要转换到实际硬件平台上进行调试。当然目前也有Sim-to-Real的研究可以加速移植的过程，甚至可以直接将仿真结果用于实际机器人平台。综上，我们要进行机器人学习就必不可少地对机器人仿真平台调研和总结。

下面将对目前在机器人仿真领域比较知名的仿真平台进行介绍和分析。

>（1）ROS-Gazebo(Gazebo)
>>官网：<http://gazebosim.org/>
>>
>>介绍：Gazebo是目前最广泛使用的仿真环境，最早在2004年由USC Robotics Research Lab（南加州大学机器人实验室）开发。依托于ROS的发展，Gazebo具有很强的仿真能力，同时也在机器人研究和开发中得到了广泛应用。Gazebo的功能包括：动力学仿真、传感器仿真、三维环境仿真、训练AI系统，同时支持多种机器人模型：包括PR2、Turtlebot、AR.Drone等。
>>
>>特点：具有以下特征：
可访问多个高性能物理引擎，包括ODE，Bullet，Simbody和DART；
使用OGRE，可提供逼真的环境渲染，包括高质量的照明，阴影和纹理；
从激光测距仪，2D/3D摄像头，Kinect样式传感器，接触传感器，力扭矩等生成传感器数据（可选带噪声）；
可开发用于机器人，传感器和环境控制的自定义插件。插件提供对Gazebo API的直接访问；
提供了许多机器人，包括PR2，Pioneer2 DX，iRobot Create和TurtleBot。或使用SDF构建自己的文件；
在远程服务器上运行模拟，并使用Google Protobufs通过基于套接字的消息传递与Gazebo进行交互；
可使用CloudSim在Amazon AWS和GzWeb上运行Gazebo，以通过浏览器与模拟进行交互；
广泛的命令行工具有助于仿真自省和控制。
>
>（2）CoppeliaSim(V-REP)
>>官网：<https://www.coppeliarobotics.com/>
>>
>>介绍：CoppeliaSim有非常完善的物理仿真引擎，支持移动机器人、飞行机器人、人型机器人、多足机器人以及多轴机械手的运动学仿真。CoppeliaSim的仿真程度非常高，不仅可以仿真机器人的本体与多种传感器，还支持障碍物以及地型(空中，地面，水底)的仿真。CoppeliaSim支持使用C/C++，Python，JAVA，Lua，Matlab编写脚本，十分适合于多机器人的仿真。作为已经商业化的软件，相比Gazebo有更好的稳定性与交互体验。CoppeliaSim可用于快速算法开发，工厂自动化仿真，快速原型制作和验证，机器人技术相关的教育，远程监控，安全性双重检查以及数字孪生等等。
>>
>>特点：具有以下特征：
CppeliaSim是跨平台的，并允许创建可移植，可伸缩和易于维护的内容：单个可移植文件可以包含功能完整的模型（或场景），包括控制代码；
模拟器和仿真是完全可定制的，具有6种相互兼容的编程方法，甚至可以携手工作。6种完全支持的编程语言；
100个可嵌入的CoppeliaSim功能：远程控制仿真或仿真器本身（例如，从真实的机器人或另一台PC）。易于使用，可扩展，快速，支持同步或异步操作；
4个物理引擎（Bullet Physics，ODE，Newton和Vortex Dynamics），用于快速和可自定义的动力学计算，以模拟现实世界中的物理和对象交互（碰撞响应，抓握等）；
任何类型的机制（分支，闭合，冗余，包含嵌套循环等）的反向/正向运动学计算。提供了IK/FK算法的可嵌入版本；
CoppeliaSim支持可自定义的粒子，可用于模拟空气或水射流，喷气发动机，螺旋桨等；
通过结合基本对象并通过嵌入式脚本链接各种功能，可以在CoppeliaSim中构建任何东西-从传感器或执行器到整个机器人系统。每个场景对象都可以附加自己的嵌入式脚本。
>
>（3）PyBullet(Bullet Real-Time Physics Simulation)
>>官网：<https://pybullet.org/wordpress/>
>>
>>介绍：PyBullet基于Bullet物理引擎开发的仿真环境，是Gazebo强有力的竞争对手。PyBullet和Python紧密结合，目前在增强学习（RL）中广泛应用。该环境可以结合TensorFlow实现RL训练，比如DQN、PPO、TRPO、DDPG等算法。目前看到比较多的都是仿真多关节机器人。
>>
>>特点：具有以下特征：
PyBullet是一个快速且易于使用的Python模块，用于机器人仿真和机器学习，重点是Sim到Real的转换；
使用PyBullet，可以从URDF、SDF、MJCF和其他文件格式加载铰接体；PyBullet提供正向动力学仿真、反向动力学计算、正向和反向运动学、碰撞检测和射线相交查询；
Bullet Physics SDK包括PyBullet机器人示例，例如模拟的四足机器人Minitaur，使用tensorflow进行决策的模拟人类跑步，以及KUKA抓取物体；
简化的坐标多体、刚体和变形体由统一的LCP约束求解器处理，类似于本文；
除了物理模拟之外，还具有渲染绑定，包括CPU渲染器（TinyRenderer）和OpenGL可视化，并支持HTC Vive和Oculus Rift等虚拟现实；
PyBullet还具有执行碰撞检测查询（最近的点，重叠对，射线相交测试等）并添加调试渲染（调试行和文本）的功能；
PyBullet具有跨平台的内置客户端服务器，支持共享内存，UDP和TCP网络，可以在连接到Windows VR服务器的Linux上运行PyBullet；
PyBullet包装了新的Bullet C-API，它独立于底层的物理引擎和渲染引擎，因此我们可以轻松地迁移到Bullet的较新版本，或者使用不同的物理引擎或渲染引擎。
>
>（4）MuJoCo(Multi-Joint dynamics with Contact)
>>官网：<http://www.mujoco.org/>
>>
>>介绍：MuJoCo是由Emo Todorov为Roboti LLC而开发的一种物理引擎，旨在促进机器人技术，生物力学，图形和动画以及其他需要快速而准确的仿真的领域的研究和开发。它提供了速度，准确性和建模能力的独特组合，但它不仅仅是一个更好的模拟器。MuJoCo侧重控制与接触相关的仿真与优化，是目前机器人强化学习中最流行的仿真器之一。
>>
>>特点：支持以下特性：
在广义坐标系中进行仿真，避免出现关节冲突；
即使存在接触也可以很好地定义逆动力学；
通过凸优化对约束进行统一的连续时间表述；
约束包括软接触，极限，干摩擦，等式约束；
能模拟粒子系统，布料，绳索和软物体；
包括马达，圆柱体，肌肉，腱，滑块曲柄的执行器；
可选择牛顿，共轭梯度或投影高斯-赛德尔求解器；
可选择金字塔形或椭圆形的摩擦锥，密集或稀疏的雅可比方程式；
可选择Euler或Runge-Kutta数值积分器；
多线程采样和有限差分近似；
直观的XML模型格式（称为MJCF）和内置的模型编译器；
跨平台GUI在OpenGL中具有交互式3D可视化；
用ANSI C编写并针对性能进行手工调整的运行时模块。
>
>（5）MATLAB Robotics Toolbox
>>官网：<https://www.mathworks.com/products/robotics.html>
>>
>>介绍：Robotics System Toolbox™ 提供了用于设计、仿真和测试操纵器、移动机器人及人形机器人的工具和算法。对于操纵器和人形机器人，该工具箱包含了使用刚体树表示形式的碰撞检查、轨迹生成、正向和逆向运动学以及动力学算法。对于移动机器人，该工具箱包含用于映射、定位、路径规划、路径跟踪和移动控制的算法。该工具箱提供了常用工业机器人应用的参考示例。该工具箱还包含可以导入、可视化和仿真的商用工业机器人模型库。通过将提供的运动学模型和动力学模型进行组合，您可以开发功能性机器人原型。借助该工具箱，您可以通过直接连接Gazebo机器人仿真器来协同仿真您的机器人应用。要在硬件上验证您的设计，可以连接到机器人平台，然后生成并部署代码（使用MATLAB Coder™ 或Simulink Coder™）。Robotics Toolbox还提供了ROS的接口，使得MATLAB代码和Simulink可以和ROS很好的结合。
>>
>>特点：具有以下特点：
您可以导入统一机器人描述格式(URDF)文件或Simscape Multibody™ 模型，以创建自定义机器人模型和视觉几何体；
对移动机器人和操纵器的基本运动学和动力学进行建模。对机器人运动进行可视化和仿真，以验证控制算法；
通过与3D物理仿真器进行接口通信，在现实世界的仿真环境中验证机器人模型。将Simulink®模型仿真与 Gazebo 仿真进行同步；
使用刚体树表示形式定义机器人模型。使用机器人模型构建高级运动控制器和接口，以完成机器人工作流程。对机器人模型执行碰撞检查和逆向运动学与动力学计算；
使用占据栅格创建环境地图，在地图中定位机器人，并为移动机器人开发路径规划和控制算法；
生成用于快速原型设计和硬件在环(HIL)测试的C/C++代码和MEX函数。
>
>（6）Webots
>>官网：<https://cyberbotics.com/>
>>
>>介绍：Webots是由Cyberbotics公司开发的一款开源的多平台机器人仿真软件，为机器人的建模、编程和仿真提供了完整的开发环境。Webots开源免费、简单易用、文档齐全并且支持多种类型的机器人。Webots内核基于开源动力学引擎ODE和OpenGL，可以在Windows、Linux和macOS上运行，并且支持多种编程语言(C/C++，Python，Java，MATLAB)。
>>
>>特点：具有以下特征：
Webots的主要功能是机器人的建模、控制与仿真，用于开发、测试和验证机器人算法。其内核基于ODE引擎，动力学仿真效果较为真实；
Webots支持多种不同类型的机器人仿真，如工业机械臂，轮式机器人，足式机器人，履带式机器人，汽车，无人机，水下机器人，航天器等。Webots支持多种虚拟传感器，如相机，雷达，力传感器，位置传感器，陀螺仪，惯性单元，GPS等。Webots还支持多种复杂环境的模拟，如室内，室外，崎岖路面，空中环境，水下环境等；
Webots场景采用OpenGL渲染，画面展示效果很棒。Webots可将仿真过程导出为动画或交互式HTML用于展示；
可以使用涵盖所有基本机器人技术需求的简单API，以C，C++，Python，Java，MATLAB或ROS进行编程开发。
>
>（7）UE4(Unreal Engine)
>>官网：<https://www.unrealengine.com/zh-CN/>
>>
>>介绍：UE（Unreal Engine）是目前世界最知名授权最广的顶尖游戏引擎，占有全球商用游戏引擎80%的市场份额。自1998年正式诞生至今，经过不断的发展，虚幻引擎已经成为整个游戏界--运用范围最广，整体运用程度最高，次世代画面标准最高的一款游戏引擎。UE4是美国Epic游戏公司研发的一款3A级次时代游戏引擎。它的前身就是大名鼎鼎的虚幻3（免费版称为UDK），许多我们耳熟能详的游戏大作，都是基于这款虚幻3引擎诞生的，例如：剑灵、鬼泣5、质量效应、战争机器、爱丽丝疯狂回归等等。其渲染效果强大以及采用pbr物理材质系统，所以它的实时渲染的效果做好了，可以达到类似[Vray]静帧的效果，成为开发者最喜爱的引擎之一。UE4不仅涉及主机游戏、PC游戏、手游等游戏方面，还涉及高精度模拟，战略演练，工况模拟，可视化与设计表现，无人机巡航等诸多领域。Microsoft就利用UE4开发了用于无人机和无人驾驶的仿真器AirSim(Aerial Informatics and Robotics Simulation)。UE4能很好地进行建模和渲染，通过接口可以对机器人进行运动学，动力学规划仿真，可作为一款不错的交互式机器人仿真平台。
>>
>>特点：具有以下特征：
实时逼真渲染：基于物理的渲染、高级动态阴影选项、屏幕空间反射和光照通道等强大功能将帮助您灵活而高效地制作出令人赞叹的内容，可以轻松获得好莱坞级别的视觉效果；
可视化脚本开发：游戏逻辑的开发提供了独创的蓝图方式和C++代码方式，其中蓝图是一种比较简单易用但又功能强大的可视化脚本开发方式；
专业动画与过场：动画方面提供了由影视行业专家设计的一款完整的非线性、实时动画工具（Sequencer），包括了动态剪辑、动画运镜以及实时游戏录制；
健壮的游戏框架：提供了包含游戏规则、玩家输出与控制、相机和用户界面等核心系统的GamePlay框架。同时内置了各种类型的游戏模板和多人游戏模板等；
灵活的材质编辑器：供了基于节点的图形化编辑着色器的功能；
先进的人工智能：提供了行为树，场景查询系统等AI相关的先进工具；
源代码开源：可以通过源代码更深入的学习或解决问题。
>
>（8）Unity3D
>>官网：<https://unity.com/>
>>
>>介绍：Unity3D是由Unity Technologies开发的一个让你轻松创建诸如三维视频游戏、建筑可视化、实时三维动画等类型互动内容的多平台的综合型游戏开发工具，是一个全面整合的专业游戏引擎。
>>
>>特点：有以下优点：
整合多种DCC文件格式，包含3dsMax, Maya, Lightwave, Collade等文档，可直接拖拽到Unity中，除原有内容外，还包含Mesh、多UVs, Vertex, Colors、骨骼动画等功能，提升游戏制作的资源应用；
Unity为开发者提供高性能的灯光系统，动态实时阴影、HDR技术、光羽和镜头特效等。多线程渲染管道技术将渲染速度大大提升，并提供先进的全局照明技术(GI)，可自动进行场景光线计算，获得逼真细腻的图像效果；
Unity渲染底层支持DirectX和OpenGL。内置的100组Shader系统，结合了简单易用，灵活，高效等特点，开发者也可以使用ShaderLab，建立自己的Shader。先进的遮挡剔除(OcclusionCulling)技术以及细节层级显示技术(LOD)，可支持大型游戏所需的运行性能；
Unity支持NVIDIAPhysX物理引擎，可模拟包含刚体&柔体、关节物理、车辆物理等；
包括GPU事件探查器、可插入的社交API应用接口，以实现社交游戏的开发。专业级音频处理API、为创建丰富通真的音效效果提供混音接口。引擎脚本编辑支持Java，C#，Boo三种脚本语言，可快速上手、并自由的创造丰宫多彩、功能强大的交互内容；
Unity可快速烘焙三维场景导航模型(NavMesh)，用来标定导航空间的分界线。目前在Unity的编辑器中即可直接进行烘焙，设定完成后即可大幅提高路径找寻(Path-finding)及人群仿真(CrowdSimulation)的效率；
Unity开发的游戏可以达到难以皿信的运行速度，在良好硬件设备下，每秒可以运算数百万面以上的多边形。高质量的粒子系统，内置的Shuriken粒子系统.可以控制粒子颜色、大小及粒子运动轨迹，可以快速创建下雨、火焰、灰尘、爆炸、烟花等效果；
Unity以创新的可视化模式让用户轻松建构互动体验，提供直观的图形化程序接口，开发者可以玩游戏的形式开发游戏，当游戏运行时，可以实时修改数值、资源甚至是程序，高效率开发，拖拽即可。
>
>总结：通过调研发现，针对机器人仿真，从开源程度、开发文档详细、仿真效果、开发便捷和流行程度来评价，比较不错的机器人仿真平台有：CoppeliaSim，Pybullet，MuJoCo，Gazebo。目前在机器人学习领域尤其火热流行的是Pybullet和MuJoCo。

---

## 3. 学习算法框架集合

强化学习（Reinforcement learning，简称RL）是机器学习中的一个领域，强调如何基于环境而行动，以取得最大化的预期利益。其灵感来源于心理学中的行为主义理论，即有机体如何在环境给予的奖励或惩罚的刺激下，逐步形成对刺激的预期，产生能获得最大利益的习惯性行为。

这个方法具有普适性，因此在其他许多领域都有研究，例如博弈论、控制论、运筹学、信息论、仿真优化、多主体系统学习、群体智能、统计学以及遗传算法。在运筹学和控制理论研究的语境下，强化学习被称作“近似动态规划”（approximate dynamic programming，ADP）。在最优控制理论中也有研究这个问题，虽然大部分的研究是关于最优解的存在和特性，并非是学习或者近似方面。在经济学和博弈论中，强化学习被用来解释在有限理性的条件下如何出现平衡。强化学习更加专注于在线规划，需要在探索（在未知的领域）和遵从（现有知识）之间找到平衡。

下面将对目前在机器人学习领域比较知名的算法框架进行调研和整理。

>（1）OpenAI Baselines(OpenAI)
>>官网：<https://github.com/openai/baselines>
>>
>>介绍：OpenAI Baselines是强化学习算法的一组高质量实现。这些算法将使研究团体更容易复制，完善和识别新思想，并将创建良好的基线以在此基础上进行研究。我们的DQN实施及其变体与已发表论文的得分大致相当。我们希望它们将被用作添加新想法的基础，以及将新方法与现有方法进行比较的工具。
>
>（2）rlpyt(UC Berkeley)
>>官网：<https://rlpyt.readthedocs.io/en/latest/>（<https://github.com/astooke/rlpyt>）
>>
>>介绍：rlpyt包含PyTorch中常见的深度RL算法的模块化优化实现，其统一的基础架构支持所有三个主要的无模型算法家族：策略梯度，深度q学习和q函数策略梯度。它旨在成为中小规模研究（如带有100个GPU的OpenAI Dota的大规模含义）的高吞吐量代码库。关键功能/特性包括：
以串行模式运行实验（有助于在开发过程中进行调试，或者可能足以进行实验）；
运行完全并行的实验，并提供用于并行采样和/或多GPU优化的选项；
在环境采样期间，使用CPU或GPU进行训练和/或批量操作选择；
同步和异步采样和优化（通过重播缓冲区）；
训练期间在线或离线智能体诊断评估；
启动用于在给定的本地硬件资源上并行地堆叠/排队实验集的实用程序（例如，在8-GPU机器上运行40个实验，每个GPU一次运行1个实验）；
与OpenAI Gym环境接口兼容；
模块化，易于修改/重复使用现有组件。
>
>（3）Stable Baselines3(DLR-RM)
>>官网：<https://stable-baselines.readthedocs.io/en/master/>（<https://github.com/DLR-RM/stable-baselines3>）
>>
>>介绍：Stable Baselines3（SB3）是用Pytorch编写的一组可靠的强化学习算法实现集合。它是“Stable Baselines”的下一个主要版本。这些算法将使研究社区和行业更容易复制，改进和识别新想法，并将创建良好的基准以在其上构建项目。我们希望这些工具将被用作添加新想法的基础，并用作将新方法与现有方法进行比较的工具。我们还希望这些工具的简单性将使初学者可以尝试使用更高级的工具集，而不会陷入实现细节中。
>
>（4）PyTorch DRL(p-christ)
>>官网：<https://github.com/p-christ/Deep-Reinforcement-Learning-Algorithms-with-PyTorch>
>>
>>介绍：该算法库包含深度强化学习算法和环境的PyTorch实现，侧重于实现hierarchical RL的算法。
>
>（5）Keras-RL2(Taylor McNally)
>>官网：<https://github.com/wau/keras-rl2>
>>
>>介绍：keras-rl2在Python中实现了一些最先进的深度强化学习算法，并与深度学习库Keras无缝集成。此外，还keras-rl2可以与OpenAI Gym一起使用。这意味着评估和使用不同算法很容易。当然，您可以keras-rl2根据自己的需要进行扩展。您可以使用内置的Keras回调和指标或定义自己的指标。更重要的是，只需扩展一些简单的抽象类，即可轻松实现自己的环境甚至算法。
>
>（6）RLlib(ray)
>>官网：<https://docs.ray.io/en/master/rllib.html>（<https://github.com/ray-project/ray/tree/master/rllib/>）
>>
>>介绍：RLlib是用于强化学习的开源库，它为各种应用程序提供高可伸缩性和统一的API。RLlib本机支持TensorFlow，TensorFlow Eager和PyTorch，但其大多数内部结构与框架无关。
>
>（7）Catalyst(catalyst-team)
>>官网：<https://catalyst-team.com/>（<https://github.com/catalyst-team/catalyst>）
>>
>>介绍：Catalyst是用于深度学习研究和开发的PyTorch框架。它着重于可重现性，快速实验和代码库重用，因此您可以创建新的东西，而不必编写另一个训练循环。
>
>（8）Tianshou(thu-ml group, Tsinghua University)
>>官网：<https://github.com/thu-ml/tianshou>
>>
>>介绍：Tianshou是基于纯PyTorch强化学习的平台。与主要基于TensorFlow，具有许多嵌套类，不友好的API或速度较慢的现有强化学习库不同，Tianshou提供了一种快速的模块化框架和类python风格的 API，以最少的行数来构建深度强化学习代理代码。
>
>（9）ReAgent(Facebook)
>>官网：<https://reagent.ai/>（<https://github.com/facebookresearch/ReAgent>）
>>
>>介绍：ReAgent是一个用于Facebook上开发和使用的应用强化学习（RL）的开源端到端平台。ReAgent是用Python构建的，并使用PyTorch进行建模和培训，并使用TorchScript进行模型服务。该平台包含用于训练流行的深度RL算法的工作流，包括数据预处理，功能转换，分布式训练，反事实策略评估和优化服务。
>
>（10）Dopamine(Google)
>>官网：<https://github.com/google/dopamine>
>>
>>介绍：Dopamine是用于强化学习算法的快速原型制作的研究框架。它旨在满足对小型，易处理的代码库的需求，用户可以在其中自由地试验荒诞的想法（投机研究）。具有以下设计特点：
轻松进行实验：使新用户可以轻松进行基准实验。
灵活的开发：使新用户可以轻松尝试研究思路。
紧凑且可靠：提供一些经过战斗测试的算法的实现。
可重现：促进结果的可重现性。
>
>（11）Spinning Up(OpenAI)
>>官网：<https://spinningup.openai.com/en/latest/>（<https://github.com/openai/spinningup>）
>>
>>介绍：这是OpenAI制作的教育资源，可让您更轻松地学习深度强化学习（deep RL）。对于不熟悉的人：强化学习（RL）是一种机器学习方法，用于教坐席如何通过反复试验来解决任务。深度RL是指RL与深度学习的结合。
本模块包含各种有用的资源，包括：
RL术语，各种算法和基本理论的简短介绍，
如何成长为一个RL研究角色的文章，
一个根据主题组织的重要论文列表，
简短的，独立的关键算法实现的，文档齐全的代码存储库，
以及一些快速上手的练习。
>
>（12）TRFL(DeepMind)
>>官网：<https://github.com/deepmind/trfl>
>>
>>介绍：TRFL（TensorFlow Reinforcement Learning）是在TensorFlow之上构建的库，它公开了一些用于实施强化学习代理的有用构建块。
>
>（13）Acme(DeepMind)
>>官网：<https://deepmind.com/research/publications/Acme>（<https://github.com/deepmind/acme>）
>>
>>介绍：Acme来自DeepMind，它可能是研究RL的最著名公司。它已被开发用于构建可读的，高效的，面向研究的RL算法，并且包含几种最新代理的实现，例如D4PG，DQN，R2D2，R2D3等。Acme使用Tensorflow作为后端，并且某些代理实现还使用JAX和Tensorflow的组合。
Acme的开发牢记要使其代码尽可能地可重用，因此其设计是模块化的，易于定制。它的文档并不丰富，但是足以为您很好地介绍该库，并且还提供了一些示例来帮助您入门Jupyter笔记本。
>
>总结：通过调研发现，大多的强化学习算法框架都为美国高的校，科研机构及高科技公司开发开源并维护的。其中比较活跃流行，维护较好，使用受欢迎的的框架为：OpenAI Baselines，PyTorch DRL，Stable Baselines3，RLlib，rlpyt，Acme。

---

## 4. 算法测试环境及基准

为了对不同的算法进行测试仿真训练效果，我们就需要一个测试环境（Environment）和评价的基准（Benchmark），下面我们来对算法测试环境及基准进行归纳和整理。

>（1）ControlSuite(DeepMind)
>>官网：<https://deepmind.com/research/open-source/deepmind-control-suite>（<https://github.com/deepmind/dm_control/>）
>>
>>介绍：DeepMind控制套件是一组连续的控制任务，具有标准化的结构和可解释的奖励，旨在用作强化学习代理的性能基准。这些任务使用Python编写，并由MuJoCo物理引擎提供支持，使其易于使用和修改。
>
>（2）Gym(OpenAI)
>>官网：<https://gym.openai.com/>（<https://github.com/openai/gym>）
>>
>>介绍：OpenAI Gym是用于开发和比较强化学习算法的工具包。这是gym开放源代码库，可让您访问一组标准化的环境。
>
>（3）RoboSchool(OpenAI)
>>官网：<https://openai.com/blog/roboschool/>（<https://github.com/openai/roboschool>）
>>
>>介绍：Roboschool提供了新的OpenAI Gym环境，用于在仿真中控制机器人。这些环境中的八个可作为先前存在的MuJoCo实现的免费替代方案，并对其进行了重新调整以产生更逼真的运动。我们还包括几个新的，具有挑战性的环境。
>
>（4）RLBench(ICL)
>>官网：<https://sites.google.com/view/rlbench>（<https://github.com/stepjam/RLBench>）
>>
>>介绍：RLBench是一个雄心勃勃的大型基准测试和学习环境，具有100个独特的手工设计任务，旨在促进在视觉引导的操纵研究领域中的研究，这些领域包括：强化学习，模仿学习，多任务学习，几何计算机视觉，尤其是少有的学习。
>
>（5）Robosuite(Stanford)
>>官网：<https://robosuite.ai/>（<https://github.com/ARISE-Initiative/robosuite>）
>>
>>介绍：robosuite是由MuJoCo物理引擎支持的用于机器人学习的仿真框架。它还提供了一套可重复研究的基准环境。发行版或robosuite v1.2具有操纵任务，并支持程序生成，高级控制器，远程操作等。该项目是更广泛的“通过模拟环境（ARISE）推进机器人智能”计划的一部分，旨在降低进入门槛在AI和机器人技术的交汇处进行前沿研究。
>
>（6）RoboTurk(Stanford)
>>官网：<https://roboturk.stanford.edu/>
>>
>>介绍：模仿学习在学习机器人操纵任务方面取得了最新进展，但由于缺乏可向其学习的高质量演示而受到限制。RoboTurk是一个系统，可通过快速众包高质量的演示来帮助解决此问题。这使我们能够为操纵任务创建大型数据集，从而提高了模仿学习策略的质量。
>
>（7）PyRoboLearn(IIT:Istituto Italiano di Tecnologia)
>>官网：<https://robotlearn.github.io/pyrobolearn/>（<https://github.com/robotlearn/pyrobolearn>）
>>
>>介绍：为了构建自主机器人，最近开发了几种具有不同功能的机器人学习框架。但是，缺乏将各种学习范例（例如模仿和强化学习）组合到一个普通地方的框架。现有的机器人往往是特定于机器人的，并且常常需要费时的工作才能与其他机器人一起使用。而且，它们的架构通常结构薄弱，主要是由于缺乏模块化和灵活性。这导致用户重新实现几段代码，以将其集成到自己的实验或基准测试工作中。为了克服这些问题，我们引入了PyRoboLearn，这是一个新的Python机器人学习框架，该框架将不同的学习范例合并为一个框架。我们的框架提供了大量的机器人环境，学习模型和算法。PyRoboLearn的开发特别注重模块化，灵活性，通用性和简化性，以支持（重用）可用性。这是通过抽象化每个关键概念，采用模块化编程方法，最小化不同模块之间的耦合以及通过继承而不是继承来实现更好的灵活性来实现的。我们通过不同的用例展示了我们框架的不同功能和实用性。
>
>（8）Meta-World(Stanford)
>>官网：<https://meta-world.github.io/>（<https://github.com/rlworkgroup/metaworld>）
>>
>>介绍：Meta-World是用于元强化学习和多任务学习的开源模拟基准，包含50种不同的机器人操纵任务。我们旨在提供足够广泛的任务分布，以评估Meta-RL算法对新行为的概括能力。
>
>（9）SURREAL(Stanford)
>>官网：<https://surreal.stanford.edu/>（<https://github.com/SurrealAI/surreal>）
>>
>>介绍：SURREAL是一个完全集成的开源的可复制且可扩展的分布式强化学习框架，可运行最新的分布式强化学习（RL）算法。SURREAL为构建分布式强化学习算法提供了高级抽象。我们在当前版本中实现了PPO和DDPG的分布式变体。
>
>（10）OpenSpiel(DeepMind)
>>官网：<https://deepmind.com/research/open-source/openspiel>（<https://github.com/deepmind/open_spiel>）
>>
>>介绍：OpenSpiel是用于一般强化学习和游戏中搜索/计划研究的环境和算法的集合。OpenSpiel支持n玩家（单人和多人）零和，合作和一般和，单发和顺序，严格的回合和同时移动，完美和不完美的信息游戏，以及传统的多人环境例如（部分和完全可观察到的）网格世界和社会困境。OpenSpiel还包括用于分析学习动态和其他常见评估指标的工具。本文档既是代码库的概述，又是增强学习，计算博弈论和搜索领域中的术语，核心概念和算法的简介。
>
>（11）PyRobot(Facebook)
>>官网：<https://pyrobot.org/>（<https://github.com/facebookresearch/pyrobot>）
>>
>>介绍：PyRobot是用于在机器人学习中进行基准测试和运行实验的Python软件包。该项目的目标是以一种易于使用的方式从高级运动生成和学习中抽象出单个机器人的低级控件。使用PyRobot可以让您运行机器人，而无需处理特定于机器人的软件，并且可以进行更好的比较。
>
>（12）S-RL Toolbox(Institut Polytechnique de Paris)
>>官网：<https://s-rl-toolbox.readthedocs.io/en/latest/>（<https://github.com/araffin/robotics-rl-srl>）
>>
>>介绍：该工具箱用于评估使用强化学习的状态表示学习方法。它集成（自动记录，绘图，保存，加载受过训练的代理）各种RL算法（PPO，A2C，ARS，ACKTR，DDPG，DQN，ACER，CMA-ES，SAC，TRPO）以及不同的SRL方法（请参见SRL Repo））（高效的方式）（使用8核cpu和1个Titan X GPU在1小时内达到1百万步）。我们还发布了可定制的Gym环境，用于仿真（Kuka手臂，PyBullet中的Mobile Robot，在8核计算机上以250 FPS运行）和真实机器人（带有ROS的Baxter和Robobo）。
>
>（13）ROBEL(Google)
>>官网：<https://sites.google.com/view/roboticsbenchmarks/home>（<https://github.com/google-research/robel>）
>>
>>介绍：ROBEL是具有成本效益的机器人和相关强化学习环境的开源平台，用于在现实世界中对强化学习进行基准测试。它提供了与Gym兼容的环境，可以轻松地在仿真（用于快速原型制作）和真实硬件中运行。ROBEL机器人功能强大且可扩展-通过各种基于学习的方法，他们已经在14000多个小时（截至2019年8月）的现实世界中进行了培训。提供了使用几种基于学习的方法的基准，以便于比较和可扩展性。
>
>（14）Menger(Google)
>>官网：<https://ai.googleblog.com/2020/10/massively-large-scale-distributed.html>
>>
>>介绍：Menger是一种大规模的大规模分布式RL基础结构，具有局部推断功能，可以在多个处理集群（例如Borg单元）中扩展至数千个参与者，从而减少了芯片放置任务中的总体训练时间。
>
>（15）SEED RL(Google)
>>官网：<https://github.com/google-research/seed_rl>
>>
>>介绍：SEED RL：具有加速的中央推理功能的可扩展，高效的Deep-RL。使用SEED的体系结构在TF2中实现了IMPALA和R2D2算法。
>
>（16）DERL(Stanford)
>>官网：<https://medium.com/syncedreview/stanford-university-deep-evolutionary-rl-framework-demonstrates-embodied-intelligence-via-learning-686c63e18dc9>
>>
>>介绍：斯坦福大学的研究人员提出了DERL（深度进化强化学习），这是一种新颖的计算框架，可使AI代理仅使用低水平的以自我为中心的感官信息即可在复杂的环境中演化形态并学习具有挑战性的运动和操纵任务。研究小组说，DERL是通过形态学学习实现的达尔文鲍德温效应的第一个证明。
>
>总结：通过调研整理发现，目前比较流行受欢迎且好用的算法测试环境及基准为：Gym，Robosuite，RoboTurk，PyRobot，RLBench，Meta-World。其中，Robosuite，RoboTurk，Gym特点比较突出。

---

## Star History Graph

[![Star History Chart](https://api.star-history.com/svg?repos=JadeCong/Awesome-Robot-Learning&type=Date&theme=dark)](https://star-history.com/#JadeCong/Awesome-Robot-Learning&Date&theme=dark)
