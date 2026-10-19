export const chinese = Object.freeze({
  "Planning trajectory…": "正在规划轨迹…",
  "The mission planner uses seven translational states and three world-force inputs. It integrates thrust, propellant depletion, gravity and exponential drag. The seed attitude approximates projected area; full rotational dynamics are left to the tracking MPC. The green path shows the current mission plan, blue shows the local MPC prediction, and silver shows the actual path. Replays retain each plan revision.":
    "任务规划器使用七个平动状态和三个世界坐标推力输入，积分推力、推进剂消耗、重力及指数大气阻力。初始猜测的姿态用于估算投影面积，完整转动动力学由跟踪 MPC 处理。绿色表示当前任务计划，蓝色表示局部 MPC 预测，银色表示实际轨迹。回放保留每次计划更新。",
  "rocket mission goal planner": "火箭任务目标规划器",
  "Mission plan": "任务计划",
  "Building a feasible rocket mission…": "正在生成火箭可行初始猜测…",
  "Optimizing the complete rocket trajectory…": "正在优化火箭任务轨迹…",
  "Replanning the remaining rocket trajectory…": "正在重新规划火箭剩余航程…",

  "Calibration references:": "校准参考：",
  "Fraunhofer separation observation": "Fraunhofer 分离观测",
  "(about 70 km and 6,000 km/h),": "（约 70 km、6,000 km/h），",
  "CRS-7 planned ascent": "CRS-7 上升计划",
  "(80 km at cutoff), and the Falcon user guide's sample staging timeline. Missions have different ascent profiles; these references set an approximate range.":
    "（关机高度 80 km），以及猎鹰用户指南中的分离时间示例。各任务的上升轨迹不同；这些资料用于确定大致范围。",
  "MuJoCo WASM at 0.002 s: free chassis, four rotating wheels, compliant tire contacts, segmented ramp and parked-vehicle collision boxes.":
    "MuJoCo WASM 以 0.002 s 步长计算自由车体、四个转动车轮、柔性轮胎接触、分段坡道和停放车辆碰撞盒。",
  "MuJoCo simulates the chassis, four rotating wheels, ramp contact, flight and landing at 0.002 s steps. Wheel torques and fan suction drive the ramp. Each jet applies force at its mounting point. Tire friction brakes the car after landing, and body or display-vehicle contact stops the run.":
    "MuJoCo 以 0.002 s 步长计算车身、四个转动车轮、坡道接触、飞行和着陆。车轮力矩与风扇吸力驱动车辆上坡，每个喷口在安装位置施力。着陆后由轮胎摩擦制动，车身或停放车辆碰撞会停止运行。",
  "The analytical RK4 model predicts flight for MPC; MuJoCo advances the actual car. The predictor includes wheel spin angular momentum but omits tire contacts. Mass, inertia, nozzle geometry and aerodynamic assumptions remain known. Tires use ellipsoid collision shapes and compliant contacts, with rigid axles and no suspension travel. Tank pressure dynamics, sensor noise and structural failure are omitted.":
    "解析 RK4 模型为 MPC 预测飞行，MuJoCo 推进实际车体。预测器考虑车轮自转角动量，但不含轮胎接触力。质量、惯量、喷口几何与空气动力假设仍视为已知。轮胎使用椭球碰撞体和柔性接触，车轴刚性连接，不模拟悬架行程。模型未包含储气压力动态、传感器噪声和结构失效。",
  "The jump scene includes simplified models of a Model 3, Model Y, Semi, Cybertruck and Cybercab. Their dimensions and spacing are approximate. Fixed MuJoCo collision boxes represent the parked vehicles. Clearance is also checked along the planned path. The model does not simulate crash deformation.":
    "越车场景包含 Model 3、Model Y、Semi、Cybertruck 和 Cybercab 的简化模型，尺寸与间距均为近似值。停放车辆使用固定的 MuJoCo 碰撞盒，规划时也检查路径净空。模型不模拟碰撞变形。",
  "MuJoCo advances the car throughout ground drive, the 180° C-ramp, flight and landing. Four wheel hinges receive drive or braking torque, and bounded fan suction holds the tires against a ramp made of 160 collision segments. The car leaves when its tires lose ramp contact at the upper lip. SCvx controls the six jets in flight. Each nozzle force acts at its body-fixed mounting point. MuJoCo tire contact cuts the jets and governs the rollout.":
    "MuJoCo 负责地面行驶、180° C 坡道、飞行和着陆的全过程。四个车轮转轴接受驱动或制动力矩，受限的风扇吸力让轮胎贴住由 160 段碰撞体组成的坡道。轮胎在顶部边缘失去坡道接触时离坡。SCvx 控制空中的六个喷口，每个喷口都在车身固定安装点施力。MuJoCo 轮胎接触触发喷口关闭，并决定接地后的滑行。",
  "The analytical RK4 flight predictor and MuJoCo plant use separate integration and contact code. The predictor includes wheel spin angular momentum but omits tire contact forces. Its COM floor can extend below tire height so MPC commands descent through contact; points beyond contact are flight extrapolations. MuJoCo advances at 0.002 s and handles friction and compliant tire contact. Mass, inertia, nozzle geometry and aerodynamic assumptions remain known, and states are read without sensor noise. Separate integration alone does not establish robustness to model error.":
    "解析 RK4 飞行预测器与 MuJoCo 物理环境使用不同的积分和接触代码。预测器考虑车轮自转角动量，但不含轮胎接触力。质心高度下限可低于轮胎接地高度，让 MPC 能持续指令下降；接地之后的预测点只是自由飞行外推。MuJoCo 以 0.002 s 步长处理摩擦和柔性轮胎接触。质量、惯量、喷口几何与空气动力假设仍视为已知，读取状态时不加入传感器噪声。仅仅分开积分器，并不能证明对模型误差的鲁棒性。",
  "GitHub Pages serves the files, and your browser runs both simulations. A module worker loads Clarabel WASM and the official single-thread MuJoCo WASM package for either vehicle. Three.js renders on the main thread. The site needs no Python API, shared memory, server-side solver or special cross-origin headers.":
    "GitHub Pages 提供静态文件，两种仿真都在浏览器内运行。模块工作线程为两种载具加载 Clarabel WASM 和官方单线程 MuJoCo WASM 包，Three.js 在主线程渲染。网站无需 Python API、共享内存、服务器求解器或特殊跨域响应头。",
  "analytical flight predictor and MPC references": "解析飞行预测器与 MPC 参考",
  "MuJoCo car plant and collision model": "MuJoCo 汽车物理与碰撞模型",
  "WHEEL DRIVE / MUJOCO": "车轮驱动 / MUJOCO",
  "FAN + WHEEL DRIVE / MUJOCO": "风扇吸力 + 车轮驱动 / MUJOCO",
  "ROADSTER SCvx MPC / MUJOCO": "ROADSTER SCvx MPC / MUJOCO",
  "WHEEL CONTACT / MUJOCO": "车轮接地 / MUJOCO",
  "BALLISTIC / MPC OFF / MUJOCO": "自由飞行 / MPC 关闭 / MUJOCO",
  "Car timestep must be a positive multiple of 0.002 s":
    "汽车时间步长必须为 0.002 s 的正整数倍",

  "Click the blue ground strip to set the landing point along the runway. Drag to move the camera.":
    "点击蓝色地面区域，选择跑道上的落点。拖动可调整镜头。",
  "Landing target changed. Compute a new flight to apply it.":
    "落点已更改，重新计算飞行后生效。",
  "Drive up the ramp, launch upside down and roll upright. Land at the selected target, facing the approach.":
    "驶上坡道，倒置离坡后翻转至正立，在所选落点着陆，车头朝向来路。",
  "Flat ground / upright jump": "平地起飞 / 正立跳跃",
  CLEAR: "越车",
  "full-flight goal planner": "完整飞行目标规划器",
  "C-ramp recovery turns the inverted car upright. In the jump scene, choose a landing position and whether to require a 360° barrel roll. Both maneuvers use the same air supply and wheel contacts.":
    "C 坡道模式将倒置车辆翻转至正立。越车模式可以选择落点，以及是否必须完成 360° 桶滚。两种动作使用同一份气体储量和车轮接触模型。",
  "Whole-flight planning optimizes the airborne trajectory from the takeoff state to the selected landing target. Guidance supplies an initial guess and duration; terminal goals, roll gates and clearance constraints shape the optimized path. A nonlinear rollout validates it before the short-horizon MPC tracks it. Amber shows the complete plan and blue shows the current MPC prediction. The duration and contact sequence remain fixed. Planning or MPC rejection stops the run. Jets shut off at physical wheel contact.":
    "完整飞行规划从离地状态出发，优化到所选落点的空中轨迹。引导规则提供初始猜测和时长，终端目标、翻滚约束与避障约束共同决定优化路径。通过非线性回放验证后，短时域 MPC 才开始跟踪。金色显示完整规划，蓝色显示当前 MPC 预测。飞行时长和接触顺序固定。规划或 MPC 验证失败时停止运行，车轮实际接地后关闭喷口。",
  "The jump uses a flat approach. Choose a landing position and whether to require a 360° barrel roll. The C-ramp always requires recovery from its inverted release. Both maneuvers use the same six jets and 250 kg air limit.":
    "越车模式从平地起跑，可选择落点和是否必须完成 360° 桶滚。C 坡道始终需要从倒置离坡状态恢复正立。两种动作使用同样的六个喷口，气体上限均为 250 kg。",
  "With whole-flight planning enabled, guidance first supplies an initial guess and flight duration. A separate SCvx solve optimizes every airborne control together, using terminal position, velocity, attitude and angular-rate costs plus air use. Intermediate positions are free. Three orientation gates preserve a requested full roll. Local separating planes and body-height constraints keep the trajectory above the ground and clear of the parked vehicles.":
    "开启完整飞行规划后，引导规则先给出初始猜测和飞行时长，独立的 SCvx 求解再共同优化所有空中控制量。代价包括终端位置、速度、姿态、角速度误差及气体消耗，中间位置由优化器决定。三个姿态约束保证所要求的完整桶滚，局部分离平面和车身高度约束用于避免触地及碰撞停放车辆。",
  "The planner uses a 0.12 s grid and up to 35 iterations. A nonlinear merit check backtracks control changes, and a finer rollout verifies clearance before acceptance. Flight duration and the contact sequence remain fixed. The accepted trajectory supplies references to the short-horizon MPC, which replans from measured states. The amber dashed line shows the complete plan; blue shows the current MPC prediction. Failed planning stops the run. Disabling whole-flight planning uses the guidance reference directly.":
    "规划器使用 0.12 s 网格，最多迭代 35 次。非线性代价检查用于回退控制步长，通过更细的积分验证净空后才接受规划。飞行时长和接触顺序固定。接受的轨迹作为短时域 MPC 的参考，MPC 根据测得的状态重新规划。金色虚线显示完整规划，蓝色显示当前 MPC 预测。规划失败时停止运行。关闭完整飞行规划后，直接使用引导参考。",
  "Feedback: 10 steps × 0.12 s; replan every 0.12 s; up to 3 convex iterations. Full-flight planning: a 0.12 s grid over the initialized flight duration, up to 35 iterations.":
    "反馈 MPC：10 步 × 0.12 s，每 0.12 s 重新规划，最多 3 次凸化迭代。完整飞行规划：在初始化的飞行时长内使用 0.12 s 网格，最多 35 次迭代。",
  "The car could not reach takeoff": "车辆未能到达离地状态",
  "Could not initialize a flight to this target": "无法为该落点生成初始轨迹",
  "This target needs a longer flight than the planner supports":
    "到达该落点所需的飞行时长超出规划器范围",
  "Planned flight is too close to a display vehicle":
    "规划轨迹距离停放车辆过近",
  "Planned flight contacts the ground before its landing goal":
    "规划轨迹在到达着陆目标前已经触地",
  "Planned flight exceeds its air budget":
    "规划轨迹超出气体预算或产生了无效状态",
  "The planned flight did not complete the selected maneuver":
    "规划轨迹未能完成所选动作",
  "The planned trajectory did not satisfy the roll gates":
    "规划轨迹未满足桶滚约束",
  "The planned landing heading is outside tolerance": "规划的着陆朝向误差过大",

  "Roadster jump": "Roadster 越车跳跃",
  "Lift over five parked Teslas and land upright at the selected target.":
    "跃过五辆停放的 Tesla，在所选落点正立着陆。",
  CLEARANCE: "越过车辆",
  "Vehicle clearance": "越过车辆",
  "Descent begins": "开始下降",
  "Flight planning": "飞行规划",
  "Landing position": "目标落点",
  "Plan whole flight from target": "根据落点规划整段飞行",
  "Require a 360° barrel roll": "必须完成 360° 桶滚",
  "Choose a target, then compute a new flight.": "选择落点后，点击计算飞行。",
  "Building a feasible initial guess…": "正在生成初始轨迹…",
  "Optimizing the complete flight…": "正在优化完整飞行轨迹…",
  "Financial Times: Roadster reveal and barrel roll, October 16, 2026":
    "金融时报：Roadster 发布与桶滚，2026 年 10 月 16 日",
  "Car and Driver: thrusters and fan system, October 17, 2026":
    "Car and Driver：推进器与风扇系统，2026 年 10 月 17 日",
  "MUJOCO PHYSICS · CLARABEL OPTIMIZER · THREE.JS":
    "MUJOCO 物理引擎 · CLARABEL 优化器 · THREE.JS",
  "SCvx MPC Demos": "SCvx MPC 演示",
  "SCvx MPC demos": "SCvx MPC 演示",
  "SCvx MPC Demos home": "SCvx MPC 演示首页",
  "SCvx MPC DEMOS": "SCvx MPC 演示",
  "← SCvx MPC Demos": "← SCvx MPC 演示",
  "Roadster · SCvx MPC Demos": "Roadster · SCvx MPC 演示",
  "Rocket · SCvx MPC Demos": "火箭 · SCvx MPC 演示",
  "SCvx formulation · SCvx MPC Demos": "SCvx 方法 · SCvx MPC 演示",
  Vehicle: "载具",
  Car: "汽车",
  CAR: "汽车",
  Rocket: "火箭",
  ROCKET: "火箭",
  "Formulation & papers ↗": "公式与论文 ↗",
  "Roadster SCvx simulation": "Roadster SCvx 仿真",
  "Rocket SCvx simulation": "火箭 SCvx 仿真",
  "BROWSER PHYSICS": "浏览器物理仿真",
  "Model assumptions and sources": "模型假设与来源",
  "TESLA ROADSTER · COLD-GAS FLIGHT STUDY": "特斯拉 ROADSTER · 冷气推进飞行",
  "Roadster ramp recovery": "Roadster C 形坡道翻转",
  "Roadster barrel roll": "Roadster 桶滚",
  "Drive up the ramp, launch upside down and roll upright. Land back at the starting marker, facing the approach.":
    "驶上坡道，倒置离坡后翻转回正，回到起点落地，车头朝向来路。",
  "Lift over five parked Teslas and roll 360°. Start beside the row and stop just beyond it.":
    "从车队旁起步，飞越五辆特斯拉并完成 360° 桶滚，在车队另一侧落地停车。",
  "RIGID-BODY SIMULATION": "刚体仿真",
  "Concept geometry · Assumed hardware": "概念外形 · 估算硬件",
  "01 / FLIGHT SETUP": "01 / 飞行设置",
  "SI UNITS": "国际单位制",
  "Roadster concept": "Roadster 概念车",
  "Original model based on reference images": "根据参考图片制作的模型",
  "Car maneuver": "汽车动作",
  Maneuver: "动作",
  "C-ramp recovery": "C 形坡道翻转",
  "360° barrel roll": "360° 桶滚",
  "Launch conditions": "起步条件",
  "Ramp speed": "上坡速度",
  "Approach speed": "接近速度",
  "Ramp radius": "坡道半径",
  "Stored air": "储存气体",
  "Propulsion model": "推进系统",
  "Ramp exit": "离坡姿态",
  "180° / inverted": "180° / 倒置",
  Nozzles: "喷口",
  "4 bottom + 2 front side": "底部 4 个 + 前侧 2 个",
  "Thrust / nozzle": "单喷口推力",
  "Exhaust speed": "排气速度",
  "Fan suction limit": "风扇吸附上限",
  "Predictive control": "预测控制",
  "Airborne recovery and landing": "空中翻转与着陆",
  "Run live": "实时运行",
  "Stop live": "停止运行",
  "Compute flight": "计算飞行",
  "Compute mission": "计算任务",
  "Computing…": "计算中…",
  "Recorded run ready": "回放已就绪",
  "Interactive Three.js flight simulation": "Three.js 交互飞行仿真",
  "PRE-LAUNCH": "起步前",
  "WHEEL DRIVE": "车轮驱动",
  "LOADING VEHICLE…": "正在加载车辆…",
  "Camera view": "视角",
  Overview: "全景",
  Follow: "跟随",
  Side: "侧视",
  "MPC close-up": "MPC 近景",
  "Z UP / EARTH 9.81 M/S²": "Z 轴向上 / 重力 9.81 M/S²",
  "DRAG TO ORBIT · SCROLL TO ZOOM": "拖动旋转 · 滚动缩放",
  "Flight path": "实际轨迹",
  "Thruster plumes": "喷气效果",
  "MPC prediction": "MPC 预测",
  "02 / TELEMETRY": "02 / 遥测",
  "ALTITUDE / COM": "质心高度",
  "Altitude over the flight": "飞行高度变化",
  VELOCITY: "速度",
  "TOTAL BODY ROLL": "车身滚转角",
  "AIR REMAINING": "剩余气体",
  "NOZZLE ACTIVITY": "喷口状态",
  ACTIVE: "工作中",
  "0–8.5 kN / jet": "每个喷口 0 至 8.5 kN",
  "● WHEEL CONTACT": "● 车轮接地",
  "○ AIRBORNE": "○ 空中飞行",
  "Export simulation data": "导出仿真数据",
  "Play simulation": "播放仿真",
  "Pause simulation": "暂停仿真",
  "Restart flight": "重新开始",
  "Simulation time": "仿真时间",
  "Playback speed": "播放速度",
  "1× speed": "1× 速度",
  "7× speed": "7× 速度",
  "MPC inspector": "MPC 详情",
  "03 / MPC INSPECTOR": "03 / MPC 详情",
  REPLAY: "回放",
  LIVE: "实时",
  "Waiting for first plan": "等待首次规划",
  "The wheels drive the ramp, and fan suction keeps the tires in contact. MPC controls six jets from ramp exit to landing.":
    "车轮驱动车辆上坡，风扇保持轮胎贴地。离坡后由 MPC 控制六个喷口，直至着陆。",
  "Inspect flight ↗": "查看飞行规划 ↗",
  "Next solve →": "下一次求解 →",
  "MPC PATH / SIDE VIEW": "MPC 轨迹 / 侧视",
  METERS: "米",
  "MPC predicted versus actual path; horizontal X position and vertical Z height in meters, axes scale independently":
    "MPC 预测与实际轨迹；横轴为 X 位置，纵轴为 Z 高度，单位为米，两轴独立缩放",
  "MPC predicted versus actual path; horizontal X position and vertical Z center of mass height in meters, axes scale independently":
    "MPC 预测与实际轨迹；横轴为 X 位置，纵轴为质心 Z 高度，单位为米，两轴独立缩放",
  "X position (m)": "X 位置（m）",
  "● Predicted path": "● 预测轨迹",
  "● Actual path": "● 实际轨迹",
  "● Rejected candidate": "● 被拒绝的方案",
  "The dot marks the current position. Each axis scales independently.":
    "圆点表示当前位置，两轴独立缩放。",
  SOLVE: "求解",
  HORIZON: "预测时域",
  LATENCY: "求解耗时",
  "PLAN AGE": "规划距今",
  "REQUEST / DELIVERED": "请求 / 实际推力",
  "REQUEST / APPLIED": "请求 / 施加推力",
  "SCALED DEFECT": "缩放后动力学残差",
  "Airborne flight predictions": "空中飞行预测",
  "Airborne flight MPC": "空中飞行 MPC",
  "8 FIXED JETS · BROWSER WASM SOLVER": "固定喷口 · 浏览器 WASM 求解器",
  "SCvx formulation ↗": "SCvx 公式 ↗",
  "Model notes & sources ↗": "模型说明与来源 ↗",
  "Close notes": "关闭说明",
  "MODEL NOTES": "模型说明",
  "What this simulation assumes": "仿真假设",
  "Six fixed jets": "六个固定喷口",
  "How MPC controls the run": "MPC 如何控制",
  "The parked vehicles": "停放车辆",
  "Contact and motion": "接触与运动",
  "Reading the display": "界面说明",
  Sources: "来源",
  Takeoff: "起飞",
  "Flat ground / 360° roll": "平地起飞 / 360° 桶滚",
  "Upright landing": "正立着陆",
  "Landing unsuccessful.": "着陆失败。",
  "GROUND APPROACH": "地面接近",
  "AIRBORNE / ROLL": "空中 / 滚转",
  "WHEEL CONTACT": "车轮接地",
  "BODY IMPACT": "车身碰撞",
  "LOAD ERROR": "加载失败",
  "MODEL READY": "模型已就绪",
  APPROACH: "接近",
  RAMP: "坡道",
  LIFTOFF: "起飞",
  FLIP: "翻转",
  DESCENT: "下降",
  BARREL_ROLL: "桶滚",
  SETTLED: "已停稳",
  CRASH: "碰撞",
  DRIVE: "行驶",
  LIFT: "起飞",
  ROLL: "滚转",
  LAND: "着陆",
  "FALCON 9 CONCEPT · MUJOCO WASM": "猎鹰 9 号概念模型 · MUJOCO WASM",
  "Rocket launch and booster landing": "火箭发射与一级返航着陆",
  "SCvx controls the booster from liftoff to landing. Follow its predicted path as it replans during flight.":
    "SCvx 从起飞到着陆控制一级火箭。飞行中可查看随状态更新的预测轨迹。",
  "Atmospheric flight · Simplified model": "含大气阻力 · 简化模型",
  "01 / MISSION SETUP": "01 / 任务设置",
  "Two-stage rocket": "两级火箭",
  "Original 3D model": "自行制作的三维模型",
  "Separation altitude": "分离高度",
  "The stages separate at this altitude, three seconds after main engine cutoff.":
    "一级提前三秒关机，滑行到所选分离高度后分离。",
  "Live simulation pace": "实时仿真速度",
  "1× real time": "1× 实时速度",
  "5× real time": "5× 实时速度",
  "7× real time": "7× 实时速度",
  "10× real time": "10× 实时速度",
  "25× real time": "25× 实时速度",
  "Booster actuators": "一级执行机构",
  Ascent: "上升",
  "9 engines": "9 台发动机",
  "Separation / turn": "分离 / 转向",
  "Engines off · RCS": "发动机关机 · RCS",
  "Return burn": "返航点火",
  "3 engines": "3 台发动机",
  Landing: "着陆",
  "1 engine": "1 台发动机",
  Gimbal: "推力矢量偏转",
  "±6° cone": "±6° 锥角",
  Coast: "无动力滑行",
  "Bounded RCS": "有界 RCS 力矩",
  "Loading recorded mission…": "正在加载任务回放…",
  "Your browser computes physics and control plans in the background. A slow solve can delay the simulation while the display stays responsive.":
    "浏览器在后台计算物理状态和控制方案。求解较慢时仿真会等待，界面仍可操作。",
  "Three.js rocket mission": "Three.js 火箭任务",
  ASCENT: "上升",
  MECO: "主发动机关机",
  SEPARATION: "级间分离",
  "UPPER IGNITION": "二级点火",
  BOOSTBACK: "返航点火",
  COAST: "无动力滑行",
  ENTRY: "再入点火",
  DESCENT_COAST: "下降滑行",
  LANDING: "着陆点火",
  "CENTER ENGINE": "切换中心发动机",
  HOVER: "悬停",
  "SLOW DESCENT": "缓降",
  CONTACT: "触地",
  "RECORDED / WASM": "回放 / WASM",
  Camera: "视角",
  "Follow rocket": "跟随火箭",
  "Follow booster": "跟随一级",
  "Upper stage": "二级",
  "Mission path": "完整轨迹",
  "Landing pad": "着陆台",
  "MPC plan": "MPC 规划",
  Plumes: "尾焰",
  "BOOSTER ALTITUDE": "一级高度",
  "VERTICAL VELOCITY": "垂直速度",
  "HORIZONTAL SPEED": "水平速度",
  PROPELLANT: "剩余推进剂",
  "THRUST / ENGINES": "推力 / 发动机数",
  "PAD DISTANCE": "距发射台",
  "Export states & plans ↓": "导出状态与规划 ↓",
  "Play mission": "播放任务",
  "Restart mission": "重新开始任务",
  "Mission time": "任务时间",
  "REPLAY / WASM": "回放 / WASM",
  "LIVE / WASM": "实时 / WASM",
  "MPC controls booster thrust, gimbal and RCS from liftoff to landing.":
    "MPC 从起飞到着陆控制一级推力、发动机偏转和 RCS。",
  "Inspect MPC ↗": "查看 MPC ↗",
  "The dot marks the latest recorded center of mass. Each axis scales independently.":
    "圆点表示最新记录的质心位置，两轴独立缩放。",
  "Booster flight predictions": "一级飞行预测",
  "Mission phases": "任务阶段",
  "Rocket implementation ↗": "火箭实现 ↗",
  "Model notes": "模型说明",
  "ROCKET MODEL": "火箭模型",
  "What controls the flight": "飞行如何控制",
  "Read the equations, assumptions and papers ↗": "查看公式、假设与论文 ↗",
  "Sound settings": "声音设置",
  "Sound off": "声音关",
  "Sound on": "声音开",
  "Sound unavailable": "声音不可用",
  "Sound volume": "音量",
  Music: "音乐",
  Volume: "音量",
  Equations: "公式",
  Papers: "论文",
  "How the controller works": "控制器如何工作",
  "State, dynamics and convex subproblem": "状态、动力学与凸子问题",
  "Rigid body equations": "刚体运动方程",
  "SCvx subproblem": "SCvx 子问题",
  "Implementation:": "实现：",
  "SCvx loop": "SCvx 迭代",
  "sparse conic builder": "稀疏锥规划构造",
  "RK4 and quaternion math": "RK4 与四元数运算",
  and: "和",
  "Car: six fixed cold-air jets": "汽车：六个固定冷气喷口",
  Model: "模型",
  "Browser implementation": "浏览器实现",
  Control: "控制量",
  "Bottom jets": "底部喷口",
  "Side jets": "侧面喷口",
  "Mass / exhaust": "质量 / 排气",
  Prediction: "预测",
  Plant: "被控系统",
  "Roadster reveal": "Roadster 发布演示",
  "engineering interview": "工程访谈",
  "thruster and fan explanation": "喷口与风扇说明",
  "Source:": "源码：",
  "browser car dynamics, ramp contact and MPC references":
    "浏览器汽车动力学、坡道接触与 MPC 参考",
  "Python reference controller": "Python 参考控制器",
  "Rocket: SCvx from liftoff through landing": "火箭：SCvx 从起飞控制到着陆",
  Phase: "阶段",
  "Actuators and reference": "执行机构与参考轨迹",
  "Cutoff / separation": "关机 / 分离",
  "Boostback / entry / landing": "返航 / 再入 / 着陆点火",
  Contact: "接触",
  "Body thrust obeys": "机体系推力满足",
  "engine data": "发动机数据",
  "RTLS burn sequence": "返回发射场的点火流程",
  "mission phases and references": "任务阶段与参考轨迹",
  "prediction and mass properties": "预测与质量属性",
  "MuJoCo plant": "MuJoCo 被控系统",
  "Three.js model": "Three.js 模型",
  "Live computation on GitHub Pages": "浏览器实时计算",
  "Papers and source": "论文与源码",
  "Project source on GitHub": "GitHub 项目源码",
  "Clarabel WASM provenance and license": "Clarabel WASM 来源与许可",
  "Open car demo": "打开汽车演示",
  "Open rocket demo": "打开火箭演示",
  "MPC disabled": "MPC 已关闭",
  "Plan rejected; run stopped": "方案被拒绝，运行已停止",
  "MPC controlling flight": "MPC 控制飞行",
  "Wheels down · MPC complete": "车轮接地 · MPC 已完成",
  "Phase-specific horizon": "按阶段调整预测时域",
  "Foot contact · engines off": "支脚触地 · 发动机关机",
  "MPC braking directly to touchdown": "MPC 持续制动至触地",
  "Minimum engine thrust exceeds the hover margin at this mass. MPC keeps braking toward foot contact without a hover.":
    "当前质量下，最低发动机推力无法留出悬停余量。MPC 持续制动至支脚触地，跳过悬停。",
  "MPC lowering the booster gently": "MPC 控制一级缓慢下降",
  "MPC hovering above the pad": "MPC 控制一级在发射台上方悬停",
  "MPC approaching the hover point": "MPC 正在接近悬停位置",
  "Main engines off before separation": "主发动机已关机，准备分离",
  "MPC turning the booster · engines off": "MPC 控制一级转向 · 发动机已关机",
  "Preparing live run…": "正在准备实时运行…",
  "Preparing compute run…": "正在准备计算…",
  "Loading maneuver…": "正在加载动作…",
  "Flight complete. Replay or drag the timeline to inspect it.":
    "飞行完成，可回放或拖动时间轴查看。",
  "Run stopped. You can replay the flight so far.":
    "运行已停止，可回放已计算的飞行片段。",
  "Mission stopped. You can replay the flight so far.":
    "任务已停止，可回放已计算的飞行片段。",
  "Mission stopped before completion": "任务未完成便已停止",
  "Mission incomplete": "任务未完成",
  "Run stopped": "运行已停止",
  "Browser worker failed": "浏览器计算线程出错",
  "Recording could not be loaded": "无法加载回放",
  "The car is an original glTF model based on reference images. Its dimensions and hardware performance are estimates. The simulation cannot establish what a production Tesla could do.":
    "汽车采用根据参考图片制作的 glTF 模型，尺寸与硬件性能均为估算。仿真结果不能证明量产特斯拉具备这些能力。",
  "The layout has four bottom jets and two front side jets. The side nozzles sit at the front body corners, clear of the wheels. Bottom jets push along the car's upward axis. Side jets push inward from either side. There are no roof jets or jets pointing along the car's length. The car must tilt to accelerate or brake along its heading.":
    "布局包含四个底部喷口和两个前侧喷口。侧喷口位于车身前角，避开车轮。底部喷口沿车身向上的轴线施力，侧喷口从两侧向内施力。车顶和车身纵向均无喷口，因此车辆必须倾斜才能沿车头方向加速或制动。",
  "The assumed dry mass is 1,428 kg, with up to 250 kg of air. Principal inertias are [800, 2,600, 2,900] kg·m² at full mass. Each nozzle produces up to 8.5 kN at a fixed exhaust speed of 650 m/s. Air use equals total thrust divided by exhaust speed. The model omits tank pressure, cooling, valve delays and recharge.":
    "干质量设为 1,428 kg，最多携带 250 kg 气体。满载主惯量为 [800, 2,600, 2,900] kg·m²。各喷口最大推力 8.5 kN，固定有效排气速度 650 m/s。耗气率等于总推力除以排气速度。模型忽略储气压力、冷却、阀门延迟与补气。",
  "The wheels drive the approach and ramp. Fan suction keeps the tires in contact. At ramp exit, a browser version of the repository's SCvx and RK4 controller plans the car's motion and orientation by choosing all six nozzle forces. It checks each plan against nonlinear dynamics, applies the first command, then replans from the next measured state.":
    "车轮驱动车辆接近并驶上坡道，风扇保持轮胎贴地。离坡后，仓库 SCvx 与 RK4 控制器的浏览器版本通过六个喷口的推力规划运动与姿态。控制器用非线性动力学校验方案，执行第一条指令，再从下一次测得的状态重新规划。",
  "C-ramp recovery targets a 180° roll to upright, facing back toward the approach. The controller also tilts the car to brake. Barrel-roll mode takes off from flat ground, rolls 360° over five parked vehicles, and lands facing forward. The wheels then brake the car. MPC uses the same air supply throughout liftoff, roll and descent. The mission logic sets the phases for ramp exit and wheel contact. The optimizer does not choose the contact sequence or plan the whole maneuver at once. If it rejects a plan, the run stops and shows the failure. The jets shut off at wheel contact, and the suspension settles.":
    "C 形坡道模式以 180° 滚转回正为目标，落地时车头朝向来路，并通过倾斜车身制动。桶滚模式从平地起飞，在五辆停放车辆上方滚转 360°，车头朝前着陆后用车轮制动。起飞、滚转和下降共用同一气源。任务逻辑规定离坡和轮胎接地阶段，优化器不选择接触顺序，也不一次性规划整个动作。方案被拒绝时运行停止并显示原因。轮胎接地后喷口关闭，由悬架完成稳定。",
  "The barrel-roll scene includes original simplified models of a Model 3, Model Y, Semi, Cybertruck and Cybercab. Their dimensions and spacing are approximate. Conservative bounding boxes check clearance beneath the Roadster. An overlap stops the run. The parked vehicles stay fixed, and the model does not simulate crash deformation.":
    "桶滚场景包含自行制作的 Model 3、Model Y、Semi、Cybertruck 和 Cybercab 简化模型，尺寸和间距为近似值。保守包围盒检查 Roadster 下方的间隙；发生重叠即停止运行。停放车辆保持不动，模型不模拟碰撞变形。",
  "Gas is stored centrally. The dry center of mass stays fixed, inertia scales with mass, and the tires are rigid. The model omits ramp recontact and structural failure.":
    "气体集中存放。干质心固定，惯量随质量缩放，轮胎视为刚性。模型不包含再次接触坡道和结构失效。",
  "Run live computes physics and new SCvx plans in a browser worker using Clarabel WASM. A 500 ms sample queue smooths playback while the solver works. The inspector shows how long each solve takes. Compute flight runs as fast as the worker allows and records the result for playback. The dashed blue line shows the prediction; silver shows the path flown. The dot marks the current position, and red marks a rejected plan. The inspector compares requested and delivered thrust. It waits during the approach and ramp, when the wheels control motion. Export saves the states and solver diagnostics.":
    "“实时运行”在浏览器工作线程中用 Clarabel WASM 计算物理状态和新的 SCvx 方案。500 ms 状态队列平滑求解期间的播放，详情面板显示每次求解耗时。“计算飞行”以工作线程允许的最快速度运行并保存回放。蓝色虚线表示预测轨迹，银色表示实际轨迹，圆点表示当前位置，红色表示被拒绝的方案。详情面板比较请求推力和实际推力；接近和上坡时由车轮控制，MPC 等待。“导出”保存状态和求解诊断。",
  "The car performs an inverted ramp exit and a full barrel roll over the display vehicles. Six jets provide lift and attitude control, fed by a shared 250 kg gas budget.":
    "汽车演示包含倒置离坡，以及在展示车辆上方完成一整圈桶滚。六个喷口提供升力和姿态控制，共用 250 kg 气体额度。",
  "SCvx chooses thrust and reaction-control system (RCS) torque throughout ascent, coasting and powered descent. RCS torque stays within the model's limits. MuJoCo applies the first command from each accepted plan. A rejected plan stops the run. MPC remains in control through landing, with no handoff to PD control.":
    "SCvx 在上升、滑行和动力下降期间选择推力及反作用控制系统（RCS）力矩，力矩受模型上限约束。MuJoCo 执行每个已接受方案的第一条指令。方案被拒绝即停止运行。MPC 持续控制到着陆，不切换为 PD 控制。",
  "The booster engines shut down before separation. Once the stages are clear, the upper stage ignites. The booster makes three separate burns: boostback, entry and landing, with engines off between them. MPC guides it back to the launch pad and slows for touchdown. It briefly hovers before lowering onto the pad when the remaining mass allows; lighter boosters land directly.":
    "一级发动机在分离前关机，两级拉开距离后，二级点火。一级随后分别进行返航、再入和着陆三次点火，每次之间均关机。MPC 引导一级返回发射台并减速着陆。剩余质量允许时会短暂悬停后下降到台面；较轻的一级直接着陆。",
  "Each run plans a new mission from the selected separation altitude and landing goal. Translational SCvx optimizes the full path, velocity, mass and thrust from a feasible initial guess. Six-DoF MPC tracks it in MuJoCo. At return-burn transitions, the planner recomputes the remaining flight from the actual state. Near the pad, MPC uses measured foot clearance to brake and land upright. Engine counts and burn order remain scheduled. The upper stage follows its own prescribed burn.":
    "每次运行都根据所选分离高度与着陆目标规划新任务。平动 SCvx 从可行初始猜测出发，优化整段路径、速度、质量和推力，六自由度 MPC 在 MuJoCo 中跟踪。返航各点火阶段切换时，规划器从实际状态重新计算剩余航程。接近发射台时，MPC 根据实测支脚净空制动并直立着陆。发动机数量和点火顺序仍由任务安排，二级独立按预设推力运行。",
  "MuJoCo simulates rigid-body motion and foot contact with changing mass, center of mass and inertia. Thrust and RCS commands act instantly. An exponential atmosphere adds drag at the center of mass. The model omits wind, lift, aerodynamic moments, Earth curvature, actuator lag, slosh and structural loads. During coast, RCS torque limits are 2 MN·m per pitch/yaw axis and 17.5 kN·m in roll.":
    "MuJoCo 用变质量、变质心和变惯量模型计算刚体运动与支脚接触。推力和 RCS 指令立即生效，指数大气阻力施加在质心。模型忽略风、升力、气动力矩、地球曲率、执行机构延迟、推进剂晃动及结构载荷。滑行时 RCS 俯仰和偏航力矩上限各为 2 MN·m，滚转为 17.5 kN·m。",
  "Each demo plans a short section of flight and checks it against nonlinear dynamics. It applies the first actuator command, then plans again from the simulated state.":
    "每个演示都规划一小段飞行，并用非线性动力学校验。控制器执行第一条指令后，根据仿真状态重新规划。",
  "Both controllers adapt the repository's SCvx approach to tracking a flight reference. They use fixed prediction times and scheduled mission phases. They do not implement the paper's full free-final-time problem or all of its state-triggered constraints, and its convergence guarantees do not carry over to these demos.":
    "两个控制器将仓库的 SCvx 方法用于跟踪飞行参考轨迹，采用固定预测时间和预设任务阶段。它们未实现论文的完整自由终端时间问题及全部状态触发约束，因此论文中的收敛保证不能直接用于这些演示。",
  "maps body vectors into the world. The rocket uses position and velocity at its changing center of mass; the car carries gas centrally.":
    "将机体系向量转换到世界坐标系。火箭的位置与速度对应随燃料消耗而变化的质心；汽车的气体集中存放。",
  "The predictor integrates these equations with RK4 and normalizes the quaternion after each step. Finite differences around a nominal nonlinear trajectory give the local discrete dynamics. The solver scales the variables before building the sparse conic problem:":
    "预测器使用 RK4 积分，并在每一步后归一化四元数。围绕名义非线性轨迹做有限差分，得到局部离散动力学。求解器先缩放变量，再构造稀疏锥规划：",
  "keeps the linearized subproblem feasible, with an L1 penalty of 30,000 in scaled coordinates. Clarabel solves each subproblem as a quadratic cone program. The predictor then simulates the candidate controls with nonlinear dynamics and linearizes around that trajectory for the next iteration.":
    "通过缩放坐标下权重为 30,000 的 L1 惩罚保持线性化子问题可行。Clarabel 将各子问题作为二次锥规划求解。预测器随后用非线性动力学模拟候选控制，并围绕所得轨迹重新线性化，进入下一次迭代。",
  "To accept a plan, Clarabel must report an optimal result. Every control must respect the actuator bounds, all states must be finite, and virtual control must stay below 0.01. The largest scaled nonlinear defect must be below 0.12 for the car or 0.04 for the rocket. A rejected plan stops the simulation and stays visible for inspection. These numerical checks do not establish flight safety.":
    "方案被接受需要 Clarabel 返回最优结果，所有控制满足执行机构限制，所有状态有限，虚拟控制小于 0.01。最大缩放非线性残差对汽车须小于 0.12，对火箭须小于 0.04。被拒绝的方案会停止仿真并保留显示。这些数值检查不能证明飞行安全。",
  "The blue path shows a prediction available at the displayed time; silver shows the path already flown. The worker measures solve latency. Plan age, nonlinear defect and virtual control show how recent the plan is and how closely it agrees with nonlinear dynamics.":
    "蓝色轨迹是当前显示时刻已获得的预测，银色轨迹是已飞过的路径。工作线程测量求解耗时。规划距今时间、非线性残差和虚拟控制反映方案的新近程度及其与非线性动力学的一致性。",
  "The wheels drive up a 180° circular ramp, with fan suction holding the tires against it. The wheelbase model handles the transition when one axle is on flat ground and the other is on the curve. The car leaves the ramp inverted. MPC controls all six jets until wheel contact, braking from release for a direct landing at the starting marker. It completes a half-roll and lands upright, facing back toward the approach. Springs, dampers and friction limits govern landing and the backward rollout.":
    "车轮驱动车辆驶上 180° 圆形坡道，风扇保持轮胎贴合。轴距模型处理一轴在平地、另一轴在曲线上的过渡。汽车倒置离坡后，MPC 控制六个喷口直到轮胎接地，并从离坡起制动，直接回到起点着陆。车辆完成半圈滚转后正立落地，车头朝向来路。弹簧、阻尼和摩擦上限决定着陆与向后滑行。",
  "Barrel-roll mode uses a flat approach. MPC begins at thruster liftoff, follows a 1.8 s powered climb and a 2 s full-roll attitude reference, then controls descent. The car lands facing forward and brakes on its wheels. The same six jets and 250 kg air limit apply. Guidance sets the timing of the reference. The controller does not optimize that timing or plan a complete path around obstacles.":
    "桶滚模式从平地接近。MPC 从喷气起飞开始控制，跟踪 1.8 s 动力爬升和 2 s 整圈滚转姿态参考，随后控制下降。车辆车头朝前着陆，用车轮制动，仍使用同样的六个喷口和 250 kg 气体上限。制导逻辑设定参考轨迹的时间，控制器不优化该时间，也不规划绕障的完整路径。",
  "Five original simplified meshes represent the parked Model 3, Model Y, Semi, Cybertruck and Cybercab. Dimensions and spacing are approximate. Separate clearance checks use conservative bounding boxes along the flown trajectory. An overlap fails the run. The parked vehicles stay fixed. Tesla's physical barrel-roll demonstration is described in":
    "五个自行制作的简化模型表示停放的 Model 3、Model Y、Semi、Cybertruck 和 Cybercab，尺寸与间距均为近似值。沿实际轨迹用保守包围盒独立检查间隙，发生重叠即判失败。停放车辆保持固定。特斯拉的实车桶滚演示见",
  "In barrel-roll mode, the flight predictor allows the center of mass (COM) down to 0.30 m, below tire contact height. The predictor omits suspension forces, so points after contact extrapolate flight. The physical simulation cuts the jets when the tires touch near 0.62 m and settles the car on compliant wheels. Setting the prediction floor below contact prevents the controller from holding the car above the ground.":
    "桶滚模式的飞行预测允许质心最低降至 0.30 m，低于轮胎接地高度。预测器忽略悬架力，因此接触后的点仍是飞行动力学外推。物理仿真在轮胎约 0.62 m 高度接地时关闭喷口，由柔性车轮稳定车身。预测下限低于接触高度，可避免控制器把汽车一直托在地面上方。",
  "Positions (+2.00, ±0.80, −0.08) m. Force inward along body Y, clear of the tires. Both are at the front, so lateral thrust also produces yaw torque.":
    "位置为 (+2.00, ±0.80, −0.08) m，避开轮胎，沿机体系 Y 轴向内施力。两个喷口都在车头，因此侧向推力也产生偏航力矩。",
  "At the full initial mass of 1,678 kg, weight is 16.46 kN. Four level bottom jets need 4.12 kN each to hover, or 6.21 kN each for the reference takeoff acceleration of 5 m/s². Their 8.5 kN limit provides 34 kN total lift and up to 10.45 m/s² net upward acceleration while level, before drag and attitude-control demands.":
    "初始总质量为 1,678 kg，重力为 16.46 kN。车身水平时，四个底部喷口各需 4.12 kN 维持悬停；实现参考起飞净加速度 5 m/s² 则各需 6.21 kN。单口 8.5 kN 上限可提供总升力 34 kN，在不计阻力和姿态控制需求时，水平车身最大净向上加速度为 10.45 m/s²。",
  "Each front side jet has a 2 m yaw moment arm. With yaw inertia 2,900 kg·m², a smooth 180° turn in two seconds needs about 6.58 kN peak side thrust, using the quintic reference's peak angular acceleration. The 8.5 kN limit gives 17 kN·m maximum yaw torque. Opposing jets act through their thrust difference. The complete simulation checks coupled rotation and gas use; the ramp maneuver reaches nearly full side thrust. Bottom thrust differences produce the barrel roll.":
    "每个前侧喷口的偏航力臂为 2 m。偏航惯量为 2,900 kg·m²，按五次参考轨迹的峰值角加速度估算，两秒内平滑转向 180° 需要约 6.58 kN 峰值侧推力。8.5 kN 上限对应最大 17 kN·m 偏航力矩，对向喷口通过推力差起作用。完整仿真检查耦合转动与耗气；坡道动作几乎用满侧推力。底部喷口的推力差产生桶滚。",
  "A least-squares attitude estimate initializes the first optimization. Only optimized commands reach the physical simulation. With MPC off, the car follows an uncontrolled ballistic path after leaving the ramp. The browser controller has no PD fallback.":
    "首次优化用最小二乘姿态估计初始化，只有优化后的指令才施加到物理仿真。关闭 MPC 时，汽车离坡后沿无控制弹道飞行。浏览器控制器没有 PD 回退。",
  "The car uses four bottom jets and two front side jets. Stored gas is capped at 250 kg and fan suction at 2 g. Each jet is capped at 8.5 kN; effective exhaust speed is 650 m/s. Gas use follows thrust divided by effective exhaust speed. The total initial mass is 1,678 kg. References:":
    "汽车采用四个底部喷口和两个前侧喷口。气体上限为 250 kg，风扇吸附上限为 2 g。每个喷口最大 8.5 kN，有效排气速度为 650 m/s。耗气率为推力除以有效排气速度。初始总质量为 1,678 kg。参考资料：",
  "Every run first builds a feasible initial guess in MuJoCo, then optimizes a full mission using translational SCvx with mass, position, velocity and world force. The objective uses the selected separation altitude, the upright landing location and velocity, and fuel use. Intermediate seed positions have zero tracking weight. The seed supplies the discrete burn schedule and force-direction corridor. Six-DoF MPC tracks the plan, including attitude, gimbal torque and changing inertia. At boostback, entry, landing ignition and the center-engine transition, the mission planner optimizes the remaining flight again from the actual state. Below 300 m, MPC uses a terminal braking reference from measured clearance and the landing goal. Burn order and phase durations are scheduled; the planner does not optimize discrete choices or guarantee a global optimum.":
    "每次运行先用 MuJoCo 生成可行初始猜测，再用平移动力学 SCvx 优化整段任务的质量、位置、速度和世界坐标推力。目标包括所选分离高度、直立着陆的位置与速度，以及燃料消耗。初始猜测的中间位置不参与跟踪代价；它提供离散点火安排和推力方向范围。六自由度 MPC 跟踪计划，并考虑姿态、推力偏转力矩和变惯量。在返航、再入、着陆点火及切换中央发动机时，任务规划器从实际状态重新优化剩余航程。300 m 以下，MPC 根据实测净空与着陆目标生成终端制动参考。点火顺序与阶段时长由任务安排，规划器不优化离散选择，也不保证全局最优。",
  "Nine engines, up to 9 × 845,000 N. The initial guess uses a height-based pitch profile reaching 20° from vertical. The selectable separation altitude is 70–80 km, with 75 km as the default. Cutoff anticipates the three-second coast to that altitude.":
    "九台发动机，最大总推力 9 × 845,000 N。初始猜测中，按高度设定的俯仰角逐渐达到偏离竖直方向 20°。分离高度可选 70–80 km，默认为 75 km。关机时提前计入到达分离高度前的三秒滑行。",
  "Booster engines shut down three seconds before separation. The upper engine waits eight more seconds and at least 5 m axial clearance before ignition. Opposite impulses give 3 m/s relative axial speed. MPC turns the booster with RCS while its engines remain off. Boostback begins after at least 6 s, 30 m axial clearance and attitude alignment.":
    "一级发动机在分离前三秒关闭。分离后再经过八秒、且轴向间距至少达到 5 m，二级发动机才点火。反向冲量产生 3 m/s 相对轴向速度。关机期间 MPC 用 RCS 控制一级转向。至少经过 6 s、两级轴向间距达到 30 m 且姿态对准后，开始返航点火。",
  "Engines off. MPC commands bounded attitude RCS: 2 MN·m per pitch/yaw axis and 17.5 kN·m in roll. The pitch/yaw authority is an educational assumption.":
    "发动机关闭。MPC 控制有界姿态 RCS：俯仰和偏航每轴最大 2 MN·m，滚转最大 17.5 kN·m。俯仰和偏航能力为教学模型假设。",
  "Three independent burns follow separation. Boostback uses three engines until the predicted ballistic impact reaches the pad. A twelve-second entry burn starts below 60 km on descent, followed by at least three seconds of coasting. A numerical stopping rollout accounts for propellant depletion and atmospheric drag to schedule landing ignition and shape the braking reference. Landing starts with three engines and reduces to one when single-engine braking can reach the ground. The center engine stays lit until contact. Every lit engine obeys the 57–100% throttle bounds. Horizontal references bring position and velocity toward the launch point over the estimated remaining descent time. Below 80 m, a mass check selects direct landing when minimum thrust prevents hovering. With sufficient mass, the booster instead settles with its feet 2 m above the pad for one second, then lowers at up to 0.7 m/s toward 0.2 m/s at contact. Below 30 m, height tracks the lowest physical foot, including tilt.":
    "分离后分别进行返航、再入和着陆三次点火。返航使用三台发动机，直到预测弹道落点到达发射台。下降至 60 km 以下时进行十二秒再入点火，随后至少滑行三秒。数值制动预测同时考虑推进剂消耗和空气阻力，用于安排着陆点火和生成制动参考。着陆先使用三台发动机，单台制动能力足够时切换至中央发动机，并保持燃烧至触地。每台发动机均遵守 57% 至 100% 油门限制。横向参考按预计剩余下降时间，同时收敛到发射点的位置与速度。80 m 以下检查质量；最低推力无法悬停时直接着陆。质量足够时，支脚在发射台上方 2 m 稳定一秒，再以不超过 0.7 m/s 下降，触地前降至约 0.2 m/s。30 m 以下的高度使用最低支脚的实际净空，并计入倾斜。",
  "Engines cut at actual foot contact. MuJoCo settles the booster for 12 seconds on a 20 m radius pad centered on the launch point.":
    "支脚实际触地时发动机关机。MuJoCo 在以起飞点为中心、半径 20 m 的发射台上继续计算 12 s，使一级稳定。",
  ". These are second-order cone constraints. Gimbal force at the engine moment arm produces pitch and yaw torque. A separate bounded RCS input supplies roll torque; powered pitch/yaw RCS is zero.":
    "。这些是二阶锥约束。发动机力臂上的偏转推力产生俯仰和偏航力矩。独立有界 RCS 输入提供滚转力矩；动力飞行时俯仰和偏航 RCS 为零。",
  "Three-engine landing uses thirty-two prediction steps of at most 0.06 s. One-engine landing uses sixteen steps of at most 0.25 s during braking and hover, then 0.12 s during final descent. Steps shrink as low as 0.002 s near contact or propellant exhaustion. Coast uses sixteen 0.15 s steps, entry uses ten 0.12 s steps, and ascent and boostback use ten 0.4 s steps. MuJoCo advances at 0.005 s. Three-engine landing replans every 0.1 s; otherwise the interval is 0.4 s, shortening to 0.2 s below 300 m, 0.04 s below 30 m and 0.01 s below 5 m. Mass, tank centers and principal inertia change as propellant burns.":
    "三台发动机着陆使用 32 个预测步，单步最多 0.06 s。单台发动机使用 16 步：制动和悬停时单步最多 0.25 s，最终下降时 0.12 s。接近触地或燃料耗尽时可缩至 0.002 s。滑行阶段为 16 × 0.15 s，再入为 10 × 0.12 s，上升和返航点火为 10 × 0.4 s。MuJoCo 步长为 0.005 s。三台发动机着陆时每 0.1 s 重规划；其他阶段为 0.4 s，300 m 以下缩至 0.2 s，30 m 以下 0.04 s，5 m 以下 0.01 s。质量、储箱质心和主惯量随推进剂消耗变化。",
  "The nominal demonstration separates at 75 km with a full initial first-stage tank of 407,500 kg. Fuel is integrated from commanded thrust and effective specific impulse. Separation speed, remaining fuel and touchdown conditions are outputs of the simulation; residual fuel is not prescribed. The default flight lands directly because its final mass cannot balance minimum engine thrust.":
    "默认任务在 75 km 分离，一级初始推进剂为 407,500 kg。燃料消耗由指令推力和有效比冲积分得到。分离速度、余油和触地状态均由仿真计算，着陆余油没有预设目标。默认飞行直接着陆，因为最终质量不足以平衡发动机最低推力。",
  "Each engine is bounded to 57–100% throttle with a maximum of 845 kN, held constant throughout flight. Dry mass is 25.6 tonnes, tank capacity is 407,500 kg and effective specific impulse is 282 s. The model includes exponential atmospheric drag and omits altitude-dependent thrust. References:":
    "每台发动机的油门范围为 57% 至 100%，最大推力固定为 845 kN。干质量为 25.6 吨，储箱容量为 407,500 kg，有效比冲为 282 s。模型包含指数大气阻力，忽略推力随高度的变化。参考资料：",
  "The second stage inherits the separation pose and angular velocity. Both bodies receive opposite axial impulses that conserve momentum. Its engine ignites eight seconds after separation once axial clearance exceeds 5 m, then follows a prescribed axial 981 kN burn outside the booster MPC.":
    "二级继承分离时的姿态与角速度。两级受到方向相反的轴向冲量并保持动量守恒。分离八秒后、且轴向间距超过 5 m 时，二级发动机点火，按预设轴向 981 kN 推力运行，独立于一级 MPC。",
  "Nozzle and flame directions ease over 0.18 s for display. Physics applies MPC commands immediately. An exponential atmosphere uses sea-level density 1.225 kg/m³, scale height 8.5 km and drag coefficient 0.8 with an attitude-dependent projected area. Drag acts at the center of mass in both the predictor and MuJoCo plant. Wind, lift, aerodynamic moments, actuator lag, Earth curvature, slosh and structural limits are omitted. The native controller has separate guidance and PD modes; browser flights use SCvx.":
    "喷管和火焰方向在显示中按 0.18 s 响应时间平滑变化，物理仿真立即施加 MPC 指令。指数大气采用海平面密度 1.225 kg/m³、尺度高度 8.5 km 和阻力系数 0.8，投影面积随姿态变化。预测器和 MuJoCo 仿真均在质心施加阻力。模型忽略风、升力、气动力矩、执行机构延迟、地球曲率、推进剂晃动及结构限制。原生控制器有独立制导与 PD 模式；浏览器飞行使用 SCvx。",
  "Run live integrates new states and solves new plans. Compute mission runs the same code as fast as the worker permits and records the result. The included recordings let the page open immediately. Their labels distinguish replay from live computation. Stop terminates the worker, including an in-progress solve. Switching vehicle tabs stops the inactive simulation.":
    "“实时运行”积分新的状态并求解新方案。“计算任务”用相同代码尽快运行并记录结果。内置回放使页面打开即可查看，界面区分回放与实时计算。“停止”终止工作线程，包括正在进行的求解。切换载具会停止未显示的仿真。",
  "Relative URLs let the site run in a repository subdirectory on Pages. Runtime packages ship with the site. If a device cannot keep up with the selected pace, the simulation slows down. The display uses the states it has received and reports the measured solve latency.":
    "相对 URL 允许网站部署在 Pages 的仓库子目录中，运行时依赖随网站一起提供。设备无法跟上所选速度时，仿真会变慢。界面显示已收到的状态并报告实测求解耗时。",
  "Successive Convexification for Real-Time Six-Degree-of-Freedom Powered Descent Guidance with State-Triggered Constraints":
    "带状态触发约束的实时六自由度动力下降制导逐次凸化方法",
  "The state has 14 components: mass": "状态包含 14 个分量：质量",
  ", world position": "、世界坐标位置",
  ", world velocity": "、世界坐标速度",
  ", a scalar-first unit quaternion": "、标量在前的单位四元数",
  ", and body angular velocity": "、机体系角速度",
  ". Rotation": "。旋转矩阵",
  "The car uses the fuel term. Virtual control": "汽车使用燃料成本项。虚拟控制",
  ". The original Python formulation is documented in": "。原始 Python 公式见",
  "the October 16 reveal report": "10 月 16 日的发布报道",
  "Six nonnegative nozzle forces, each capped at 8.5 kN and tapered below 12 kg of remaining gas.":
    "六个非负喷口推力，各上限 8.5 kN，剩余气体低于 12 kg 时逐渐降低上限。",
  "Positions (±1.45, ±0.78, −0.32) m. Force in body +Z.":
    "位置 (±1.45, ±0.78, −0.32) m，沿机体系 +Z 施力。",
  "1,428 kg dry, up to 250 kg gas, 650 m/s effective exhaust velocity. Inertia scales with mass.":
    "干质量 1,428 kg，气体最多 250 kg，有效排气速度 650 m/s。惯量随质量缩放。",
  "10 steps × 0.12 s; replan every 0.12 s; up to 3 convex iterations.":
    "10 步 × 0.12 s；每 0.12 s 重规划；最多 3 次凸迭代。",
  ". The foundational SCvx method.": "。SCvx 的基础方法。",
  ". The main powered-descent reference used by this repository.":
    "。本仓库动力下降控制的主要参考论文。",
  "Successive Convexification of Non-Convex Optimal Control Problems and Its Convergence Properties":
    "非凸最优控制问题的逐次凸化及其收敛性质",
  "Successive Convexification for 6-DoF Mars Rocket Powered Landing with Free-Final-Time":
    "自由终端时间六自由度火星火箭动力着陆的逐次凸化方法",
  "The wheels drive the approach. MPC controls six jets through liftoff, the full roll and descent. The wheels brake after landing.":
    "车轮驱动车辆接近。MPC 在起飞、整圈滚转与下降期间控制六个喷口，着陆后由车轮制动。",
  "The wheels drive the ramp, and fan suction keeps the tires in contact. MPC controls all six jets from ramp exit to wheel contact.":
    "车轮驱动车辆上坡，风扇保持轮胎贴地。离坡后 MPC 控制六个喷口，直到轮胎接地。",
  "MPC is off. The car cannot lift from flat ground.":
    "MPC 已关闭，汽车无法从平地起飞。",
  "MPC is off. The car coasts after ramp release.":
    "MPC 已关闭，汽车离坡后沿弹道飞行。",
  "The solver rejected this plan and stopped the run. The red path stays visible for inspection.":
    "求解器拒绝了该方案并停止运行，红色轨迹保留供查看。",
  "MPC predicts motion and orientation 1.2 seconds ahead through the climb, full roll and descent. It chooses the six jet forces. The wheels brake after landing.":
    "MPC 在爬升、整圈滚转和下降期间预测未来 1.2 秒的运动与姿态，选择六个喷口的推力。着陆后由车轮制动。",
  "MPC predicts motion and orientation 1.2 seconds ahead and chooses all six jet forces. It applies the first command, then replans. The car tilts to brake.":
    "MPC 预测未来 1.2 秒的运动与姿态并选择六个喷口的推力，执行第一条指令后重新规划。汽车通过倾斜车身制动。",
  "The wheels now support and brake the car. The last flight plan remains visible.":
    "现在由车轮支撑并制动车辆，最后一次飞行规划仍保留显示。",
  "The red path shows the rejected plan. A plan that fails the acceptance checks stops the run.":
    "红色轨迹表示被拒绝的方案。方案未通过验收检查时，运行停止。",
  "The booster has landed with its engines off. The last plan remains visible.":
    "一级已着陆，发动机已关闭，最后一次规划仍保留显示。",
  "After the brief hover, MPC lowers the booster at up to 0.7 m/s and slows toward 0.2 m/s before foot contact.":
    "短暂悬停后，MPC 以不超过 0.7 m/s 的速度下降，并在支脚触地前减速至接近 0.2 m/s。",
  "MPC settles the booster with its feet 2 m above the pad, holds for one second, then lowers it to the pad. The center engine stays lit until contact.":
    "MPC 将一级稳定在支脚距台面 2 m 处，保持一秒后下降到发射台。中心发动机持续点火直到触地。",
  "The main engines are off. The stages coast together until the separation impulse. MPC uses RCS to hold their orientation.":
    "主发动机已关机。两级共同滑行直到施加分离冲量，MPC 用 RCS 保持姿态。",
  "MPC turns the booster with RCS while its engines stay off. The upper stage ignites once clear. The booster waits for enough clearance and alignment for return before relighting.":
    "一级关机时 MPC 用 RCS 控制转向，两级拉开距离后二级点火。一级等待足够间距并对准返航姿态后再次点火。",
  "MPC controls the boostback burn to return toward the launch pad. Engines shut down before the entry burn.":
    "MPC 控制返航点火，使一级返回发射台方向，随后关机，等待再入点火。",
  "The entry burn is complete. Engines stay off while MPC holds attitude with RCS until landing ignition.":
    "再入点火已完成。发动机保持关闭，MPC 用 RCS 保持姿态，直到着陆点火。",
  "Loading browser solver…": "正在加载浏览器求解器…",
  "Loading MuJoCo physics…": "正在加载 MuJoCo 物理引擎…",
  "Run live ●": "实时运行 ●",
  "Stop live ■": "停止运行 ■",
  "LIVE / BROWSER PHYSICS": "实时 / 浏览器物理仿真",
  "LIVE / MUJOCO WASM": "实时 / MUJOCO WASM",
  "REPLAY / BROWSER PHYSICS": "回放 / 浏览器物理仿真",
  "glTF / 6 JETS": "glTF / 6 个喷口",
  "Settings changed. Compute a new flight to apply them.":
    "设置已更改，重新计算飞行后生效。",
  "Settings changed. Run live or compute a new flight.":
    "设置已更改，请实时运行或重新计算飞行。",
});

export function translateDynamic(text) {
  const rules = [
    [
      /^Could not compute flight: (.+)$/,
      (_, reason) =>
        `无法计算飞行：${chinese[reason] ?? translateDynamic(reason)}`,
    ],
    [
      /^No validated full-flight plan for this target \((.+), position error ([\d.]+) m\)$/,
      "未找到该落点的有效完整规划（$1，位置误差 $2 m）",
    ],
    [
      /^Full-flight plan · ([\d.]+) s flight · ([\d.]+) s planning$/,
      "完整飞行规划 · 飞行 $1 s · 规划耗时 $2 s",
    ],
    [
      /^MPC controlling (.+)$/,
      (_, phase) =>
        `MPC 正在控制${chinese[phase.toUpperCase().replaceAll(" ", "_")] ?? phase}`,
    ],
    [/^#(\d+) \/ (\d+) iterations$/, (_, id, n) => `#${id} / ${n} 次迭代`],
    [/^FAN ([\d.]+) g$/, "风扇 $1 g"],
    [
      /^(Bottom|Front side) nozzle (\d+)(.*)$/,
      (_, side, n, tail) =>
        `${side === "Bottom" ? "底部" : "前侧"}喷口 ${n}${tail}`,
    ],
    [
      /^Booster landed · ([\d.]+) m from pad center$/,
      "一级已着陆 · 距台面中心 $1 m",
    ],
    [
      /^Booster landed · ([\d.]+) m pad error · (\d+) kg propellant left$/,
      "一级已着陆 · 位置误差 $1 m · 剩余推进剂 $2 kg",
    ],
    [
      /^SCvx · (\d+) accepted \/ (\d+) rejected$/,
      "SCvx · 接受 $1 次 / 拒绝 $2 次",
    ],
    [
      /^Landing (successful|unsuccessful) · ([\d.]+) s compute$/,
      (_, status, seconds) =>
        `${status === "successful" ? "着陆成功" : "着陆失败"} · 计算耗时 ${seconds} s`,
    ],
    [
      /^([\d.]+) m\/s (touchdown|contact) · ([\d.]+)° tilt$/,
      "触地速度 $1 m/s · 倾角 $3°",
    ],
    [/^([\d.]+) kg of air remaining$/, "剩余气体 $1 kg"],
    [
      /^Computing (.+) · t=([\d.]+) s$/,
      (_, phase, t) =>
        `正在计算${chinese[phase.toUpperCase()] ?? phase} · t=${t} s`,
    ],
    [/^Live physics · t=([\d.]+) s · (.+)$/, "实时物理仿真 · t=$1 s · $2"],
    [
      /^MPC predicts motion and orientation ([\d.]+) seconds ahead\. It applies the first thrust and RCS command, then replans\. The chart tracks the center of mass\.$/,
      "MPC 预测未来 $1 秒的运动与姿态，执行第一条推力和 RCS 指令后重新规划。图表跟踪质心。",
    ],
  ];
  for (const [pattern, replacement] of rules)
    if (pattern.test(text)) return text.replace(pattern, replacement);
  return text
    .replace(/virtual control/g, "虚拟控制")
    .replace(/\bSolved\b/g, "已求解")
    .replace(/\bAlmostSolved\b/g, "近似求解")
    .replace(/\bSCvx six fixed jets\b/g, "SCvx 六个固定喷口")
    .replace(/\bSCvx booster\b/g, "SCvx 一级火箭")
    .replace(/^\/ LIVE$/, "/ 实时");
}
