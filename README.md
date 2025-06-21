## Laero Model

更简化的Rac Model见`StandaloneRacModel`目录

### 1、基础依赖库 (`OeBase.hpp`)

从OpenEaagles中提取的基础工具的集合，以使其余代码可以正常工作。

### 2、飞机状态结构体 (`AircraftState.hpp`)

定义了用于存储所有飞机实时数据的结构体。

###  3、独立动力学模型 - 头文件 (`StandaloneLaeroModel.hpp`)

这是改造后模型的公共接口。

### 4、独立动力学模型 - 实现文件 (`StandaloneLaeroModel.cpp`)

这是模型核心逻辑的实现，所有对 `Player` 的依赖都已被移除。

### 5、示例主程序 (main.cpp)

沿着一个由样条曲线生成的、以0.1秒为周期离散化的机动轨迹飞行。包含以下核心改动：

1. **轨迹数据结构**：`TrajectoryPoint` 将包含时间戳 `timestamp`，这是时间同步的关键。
2. **模拟的机动轨迹**：创建一个函数 `createManeuverTrajectory()` 来生成一条密集的、平滑的S型转弯爬升轨迹，以模拟您给定的数据。
3. **时间同步的控制逻辑**：仿真循环将不再是“飞向”下一个航路点，而是根据当前的仿真时间 `simTime`，从期望轨迹数据中查找对应的目标状态，并以此作为飞机的控制指令。
4. **精确的初始状态设置**：为了避免初始的巨大误差，我们会给 `StandaloneLaeroModel` 增加一个设置初始状态的函数，让飞机在仿真开始时就精确地位于轨迹的起点。
5. **详尽的数据记录**：CSV文件将记录每个仿真时刻的飞机实际状态、期望状态以及它们之间的误差。

这个主程序现在将执行时间同步的轨迹跟踪任务。

1. **`createManeuverTrajectory` 函数**:
   - 该函数模拟了一个预先生成好的轨迹数据。它以0.1秒为间隔，循环生成了120秒的飞行数据点。
   - 轨迹包含S型转弯、爬升、加速、平飞、下降、减速等多个阶段，形成了一个复杂的机动动作。
   - **注意**: 这里的轨迹生成方式比较简单，主要是为了提供一个数据源。在您的实际应用中，您会直接加载您已有的离散化轨迹数据文件到 `std::vector<TrajectoryPoint>` 中。
2. **`main` 函数中的初始化**:
   - 程序不再是随意起飞，而是通过调用新增的 `aircraft.setInitialState(initialState)`，让飞机的初始位置、速度、航向等**精确匹配**轨迹的第一个点。这是实现高精度跟踪的前提。
3. **时间同步的仿真循环**:
   - 循环的核心不再是检查与航路点的距离，而是检查**仿真时间 `simTime`**。
   - `while (trajectoryIndex < trajectory.size() - 1 && trajectory[trajectoryIndex].timestamp < simTime)` 这一行代码是关键。它的作用是，在每个仿真步长，都向前查找轨迹数据，直到找到第一个**时间戳大于等于**当前仿真时间的轨迹点。这个点就是飞机当前应该对准的目标。
   - 这样，即使仿真步长 `dt` (1/60秒) 和轨迹采样周期 `Ts` (0.1秒) 不一样，飞机也总能找到正确的目标状态进行跟踪。
4. **数据记录**:
   - CSV文件头增加了 `TargetHdg` 等列，因为期望航向现在是轨迹的已知部分。
   - 文件中记录了每个仿真时刻飞机的完整实际状态、期望目标状态以及两者之间的各项误差，为性能评估提供了详尽的数据支持。

## 输入输出

### 1.  模型输入

模型的输入分为两类：初始化输入和周期性更新输入。

* **初始化输入**:
  * **初始状态 (`AircraftState`)**: 通过 `setInitialState(const AircraftState& initialState)` 函数设置飞机的初始运动状态，包括：
    * 初始位置 (`position`)
    * 初始姿态（偏航角 `yaw`）
    * 初始速度大小 (`bodyVelocity`)

* **周期性更新输入 (在每个仿真循环中调用)**:
  * **时间步长 (`double dt`)**: 通过 `update(const double dt)` 函数传入，是驱动模型状态积分的核心参数。
  * **高层飞行指令**:
    * `setCommandedAltitude(double meters, ...)`: 设置期望的飞行高度（米）。
    * `setCommandedHeadingD(double degs, ...)`: 设置期望的飞行航向（度）。
    * `setCommandedVelocityKts(double kts, ...)`: 设置期望的飞行速度（节）。

### 2. 模型输出

模型在每个 `update` 调用后，会更新其内部的飞机状态。外部应用程序可通过以下方式获取完整的飞机状态：

* **完整状态数据 (`AircraftState`)**: 通过 `const AircraftState& getState() const` 函数返回一个包含飞机当前完整六自由度（6-DOF）状态的只读引用。`AircraftState` 结构体包含：
  * **位置**: `position` (Vec3d, 世界坐标系)
  * **速度**: `velocity` (Vec3d, 世界坐标系), `bodyVelocity` (Vec3d, 机体坐标系)
  * **姿态**: `roll`, `pitch`, `yaw` (double, 欧拉角，弧度)
  * **角速度**: `angularVelocity` (Vec3d, 机体坐标系 p, q, r)

## 编译

将这五个文件保存在同一个目录下，然后使用C++17兼容的编译器（如g++）通过以下命令进行编译和运行：

```bash
g++ main.cpp StandaloneLaeroModel.cpp -o TrajectorySim -std=c++17 -I.

./LaeroSim`
```



## 参数调整

可调整的参数可以分为三大类：

### A. 飞机性能/控制律参数

这些参数是**最核心的调节参数**，它们定义了飞机如何响应指令，直接影响其机动性能和稳定性。可以在 `StandaloneLaeroModel` 的代码中找到它们，主要是在高层指令函数的默认参数或内部实现中。

1.  **最大坡度/滚转角 (`maxBankD`)**:
    * **作用**: 限制飞机在转弯时允许的最大倾斜角度。这个值越大，飞机转弯时就越“激进”，能够实现更快的转弯速率。
    * **位置**: `StandaloneLaeroModel.cpp` 中 `setCommandedHeadingD` 函数的默认参数。
    * **默认值**: `30.0` (度)。

2.  **最大转弯速率 (`hDps`)**:
    * **作用**: 限制飞机偏航（转弯）的最大角速度。
    * **位置**: `StandaloneLaeroModel.cpp` 中 `setCommandedHeadingD` 函数的默认参数。
    * **默认值**: `20.0` (度/秒)。

3.  **最大爬升/下降率 (`aMps`)**:
    * **作用**: 限制飞机垂直方向的最大速度。
    * **位置**: `StandaloneLaeroModel.cpp` 中 `setCommandedAltitude` 函数的默认参数。
    * **默认值**: `150.0` (米/秒)。

4.  **最大俯仰角 (`maxPitchD`)**:
    * **作用**: 限制飞机机头向上或向下的最大角度。这可以防止飞机做出过于剧烈的爬升或俯冲动作。
    * **位置**: `StandaloneLaeroModel.cpp` 中 `setCommandedAltitude` 函数的默认参数。
    * **默认值**: `15.0` (度)。

5.  **最大加速度 (`vNps`)**:
    * **作用**: 限制飞机加速或减速的快慢。
    * **位置**: `StandaloneLaeroModel.cpp` 中 `setCommandedVelocityKts` 函数的默认参数。
    * **默认值**: `5.0` (节/秒)。

6.  **控制律时间常数 (`TAU`)**:
    * **作用**: 这是一个非常关键的内部参数，决定了飞机响应的**“平滑度”和“攻击性”**。它在 `flyPhi`, `flyTht` 以及 `setCommanded...` 等函数的内部使用，用于计算误差断点。
        * **较小的 `TAU`**: 飞机会更“激进”，更快地尝试修正误差，但可能导致超调和振荡。
        * **较大的 `TAU`**: 飞机会更“平滑”和“迟缓”，响应更稳定，但修正误差的速度较慢。
    * **位置**: `StandaloneLaeroModel.cpp` 文件中各个控制律函数的内部。
    * **默认值**: 高度控制（`setCommandedAltitude`）中为 `4.0`，其他控制（航向、速度）中为 `1.0`。

### B. 期望轨迹参数 (定义飞行任务)

这些参数定义了飞机需要飞行的目标路径，您可以通过修改 `main.cpp` 中的 `createManeuverTrajectory` 函数来完全自定义。

1.  **轨迹采样周期 (`Ts`)**:
    * **作用**: 生成期望轨迹数据点的时间间隔。
    * **位置**: `main.cpp` 中 `createManeuverTrajectory` 函数的参数。
    * **默认值**: `0.1` (秒)。

2.  **轨迹持续时间 (`duration`)**:
    * **作用**: 整个期望轨迹的总时长。
    * **位置**: `main.cpp` 中 `createManeuverTrajectory` 函数的参数。
    * **默认值**: `120.0` (秒)。

3.  **轨迹剖面 (Profile)**:
    * **作用**: 轨迹的核心，定义了每个时刻期望的高度、速度和航向。您可以通过修改 `createManeuverTrajectory` 函数内部的 `if/else` 逻辑来创建任何您想要的机动动作，例如更急的转弯、更陡峭的爬升、不同的速度变化等。

### C. 仿真环境参数

这些参数定义了仿真本身的基本属性。

1.  **仿真步长 (`dt`)**:
    * **作用**: 动力学模型每次更新的时间间隔。较小的 `dt` 会提高积分精度，但增加计算量。
    * **位置**: `main.cpp` 的主循环前。
    * **默认值**: `1.0 / 60.0` (即每秒更新60次)。

**调节建议**:
* **初级调节**: 从修改 `main.cpp` 中的 `createManeuverTrajectory` 函数开始，设计您自己的飞行路径。
* **中级调节**: 调整 `setCommanded...` 函数的默认参数（如 `maxBankD`, `maxPitchD`），以改变飞机的总体机动性能限制。
* **高级调节**: 如果您发现飞机响应过于迟缓或过于振荡，可以尝试微调 `StandaloneLaeroModel.cpp` 内部的 `TAU` 值，这是最能影响飞行“风格”的参数。

## 模型测试分析

在运行测试代码之前，请确保已经安装了 `pandas` 和 `matplotlib`。如果尚未安装，可以通过pip进行安装：

```bash
pip install pandas matplotlib
```

确保`analyze_trajectory.py`与C++程序生成的 `maneuver_log.csv` 文件在同一个目录下。

1. **运行脚本**:

     * 打开您的终端或命令行。

     * 导航到包含 `analyze_trajectory.py` 和 `maneuver_log.csv` 的目录。

     * 运行命令：

       ````bash
       python analyze_trajectory.py
       ````

2.  **解读图表**:

      * **飞行轨迹图 (Flight Trajectory)**:

          * **蓝色实线**代表飞机的实际飞行路径。
          * **红色虚线**代表期望的轨迹。
          * 理想情况下，蓝色实线应与红色虚线高度重合。您可以直观地看到飞机在转弯时的切入和切出表现，以及跟踪直线航段的精度。

      * **高度剖面图 (Altitude Profile)**:

          * 此图显示了飞机在爬升、平飞和下降阶段的表现。
          * 观察蓝色实线（实际高度）跟随红色虚线（期望高度）的速度和精度。在高度指令变化时（例如 t=20s 或 t=100s），蓝色曲线与红色曲线之间的差距，以及它需要多长时间才能重新对齐，反映了高度控制系统的响应性能。

      * **误差变化图 (Tracking Errors)**: 这是进行定量分析最关键的图表。

          * **整体趋势**: 所有误差曲线都应趋向于0。如果某条曲线长时间保持在一个较大的值，说明该控制通道存在问题。
          * **收敛速度 (Convergence Speed)**: 当指令变化时（例如 t=30s 开始转弯），观察误差曲线（特别是航向误差）从峰值下降到稳定状态的速度。下降得越快，说明系统的响应和收敛速度越快。
          * **稳定性与超调 (Stability & Overshoot)**: 观察误差曲线是否平滑。如果曲线在达到0附近时反复上下振荡（oscillation），或者在指令变化时先朝相反方向偏离（undershoot）或冲过头（overshoot），这可能意味着控制参数（如 `LaeroModel` 中的 `TAU` 值）可能需要调整。一个稳定系统的误差曲线应该是平滑且阻尼良好的。
          * **稳态误差 (Steady-State Error)**: 在飞行状态稳定后（例如平飞阶段），观察误差是否完全为0。如果误差稳定在一个很小的非零值，这被称为稳态误差，反映了控制器消除长期偏差的能力。对于这个简单的P控制器模型，存在一些小的稳态误差是正常的。

