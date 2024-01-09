# UAV
Please refer to the master branch and other branch in this repository. 请参考本库中的master分支或其它分支。

Adaptive autopilot and guidance of UAV
针对无人机控制中经常出现的状态依赖、非结构不确定因素的影响，如非模型动力学、环境扰动、载荷变化等问题。目前广泛采用的是基于级联闭环PID控制器，如ArduPilot、PX4等开源飞控系统，然而这类控制系统是将非线性无人机系统线性化，导致PID控制器不足以处理无人机运行中的非线性不确定问题，需要频繁调整控制增益。为了攻克这一难题，提出了一种嵌入式自适应的ArduPilot飞控系统，这种飞控系统的最大特点是不用改变ArduPilot的原始控制架构，允许用户照常使用飞控，但是能提升飞控的鲁棒性和自适应性，有效处理无人机运行中的非线性不确定问题。实验结果表明，提出的ArduPilot飞控方案相比原始ArduPilot，能够在飞行动力学改变的情况下极大提升控制性能。与此同时，当前无人系统控制都只针对特定类型的系统来设计对应的控制器，如无人机，无人车，无人船等，对于不同的无人系统缺乏一种统一的控制架构。提出一种针对非结构不确定的无人系统控制框架，并应用于ArduPlane，ArduCopter，ArduRover等ArduPilot的套件中，使得这种控制框架既能控制无人机，无人车等不同类型无人系统，也能有效应对无人系统控制中的非线性不确定问题。实验表明提出的控制架构在多种类型的无人系统中均能有效提升ArduPilot的鲁棒性和自适应性。通过在ArduPilot中嵌入自适应模块，我们不仅有效解决了ArduPilot控制架构固有存在的无法处理状态依赖不确定问题，同时提出的无人系统控制架构能够应用到多种类型的无人系统，解决了ArduPilot的非结构不确定问题。
![loop](https://github.com/Friend-Peng/Adaptive-ArduPilot-Autopilot/blob/main/loop.jpg)

请参考论文：
10.1109/TMECH.2022.3145910  10.1109/TCST.2023.3329908  10.1109/CDC51059.2022.9993292
