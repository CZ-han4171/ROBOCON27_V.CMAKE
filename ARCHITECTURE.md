# ARCHITECTURE.md — 一页纸工程地图

> 本文件回答两个问题：**① CAN 通信在哪？② 怎么看懂这个工程？**
> 目标：新队员照着这张图，30 分钟能说清"一个功能从遥控到电机是怎么跑通的"。

---

## 一、目录分层职责（先认目录，别按文件名猜）

| 目录 | 职责（自下而上） | 判断标准 | 能改吗 |
|------|------------------|----------|--------|
| `Drivers/` `Middlewares/` `USB_DEVICE/` | 厂商/第三方库 | 名字是 ST 官方库名 | ❌ 不动 |
| `Core/` | CubeMX 生成 + main + 中断向量 | 含 `/* USER CODE BEGIN/END */` | ⚠️ 只在 USER CODE 区写 |
| `BSP/` | 板级支持包：Canbus、UartPort、UsbPort、bsp_dwt、bsp_usart | "某外设怎么用" | ✅ |
| `Algorithm/` | 纯算法/数据结构（无锁队列、双缓冲、环形缓冲） | "通用工具" | ✅ |
| `Module/` | 设备/模块抽象：motor、Axis、controller(PID)、imu、Position、rm_pocket、pc、servo、topics | "某设备怎么控制" | ✅ |
| `APP/` | 任务与业务逻辑：robot_init / robot_task / robot_com / control_task / chassis_task / debug_task | "机器人要干什么" | ✅ 主要在这写 |

**根目录 readme.md 已写死"新代码放哪"的规则：**
- 加外设：`module 层调用 BSP 层接口 → module 层写功能函数/封装 → APP 层调用`
- 加执行逻辑：`APP 层调功能函数 → debug_task 写状态机 → 测试 → 新建任务文件装进去`

---

## 二、CAN 通信在哪（6 层，从下到上）

### A. 硬件配置层（CubeMX 生成，回 CubeMX 改，别手改）
- `ROBOCON26.ioc`：引脚/时钟源总配置
- `Core/Src/fdcan.c` + `Core/Inc/fdcan.h`：3 路 FDCAN 初始化
- `Core/Src/stm32h7xx_it.c`：中断入口
- `Core/Src/main.c`：调 `MX_FDCAN1/2/3_Init()`

| 总线 | RX | TX | AF | NVIC 优先级 | 波特率参数 |
|------|----|----|----|-------------|-----------|
| FDCAN1 | PD0 | PD1 | AF9 | 5 | Prescaler=1, SJW=2, Seg1=15, Seg2=9 |
| FDCAN2 | PB12 | PB13 | AF9 | 5 | 同上 |
| FDCAN3 | PF6 | PF7 | AF2 | 5 | 同上 |

### B. HAL 驱动层（厂商库，别动）
`Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_fdcan.c`

### C. BSP 抽象层 ⭐ CAN 收发核心
- `BSP/Canbus.hpp` / `BSP/Canbus.cpp`：本工程自写的总线驱动
- `Algorithm/lockfree_queue.hpp`：发送用无锁 MPSC 队列 `MpscQueue<ClassicPack, 8>`

关键结构：
- `CanDevice` 基类：`matchRx()`（按 ID 匹配）、`onRx()`（收报文虚回调）、`buildTx()`（组报文虚函数）
- `CanBus` 类：`init()`、`registerDevice()`（一条总线最多挂 6 个设备）、`addCanMsg()`（入发送队列）、`txService()`（原子锁串行化发送）、`processRxInterrupt()`（中断里遍历设备按 ID 分发）
- `canFilterInit()`：标准帧→FIFO0，扩展帧→FIFO1
- `bspCanInit()`：使能 Rx/Tx 中断 + `HAL_FDCAN_Start()`
- HAL 回调：`HAL_FDCAN_RxFifo0Callback()` / `RxFifo1Callback()`

### D. 模块层（谁收、谁发、怎么解析）
- `Module/motor/Motor.hpp` / `Motor.cpp`：`C610Motor::onRx()`、`C620Motor::onRx()`、`DM4310Motor::onRx()`、`VESCMotor::onRx()`；`packDJIMotorCanMsg()`（大疆 4 电机合一打包 8 字节）

### E. 应用层（总线路由 + 周期发送任务）
- `APP/robot_com/com_config.cpp`：**所有 CAN 实例化与路由都在这（全局信息交互中心）**
- `APP/robot_task/robot_task.cpp`：创建 can1/2/3SendTask 三个线程

`com_config.cpp` 里的实例化（74~96 行）：

| 总线 | 设备 | CAN ID |
|------|------|--------|
| fdcan1_bus | VESC U8_1~U8_4 | 0x965~0x968（扩展帧） |
| fdcan2_bus | arm2006 / arm3508 / arm4310 | 0x205 / 0x206 / 0x301 |
| fdcan3_bus | 底盘 chassis_motor1~4 | 0x201~0x204 |

`comServiceInit()`（183~228 行）：`canFilterInit()`×6 → `bspCanInit()`×3 → `bus.init()` → `registerDevice()`×11
`can1/2/3SendTask`（299~360 行）：1kHz 周期组帧发送

### F. 完整数据流（看懂这张图 ≈ 看懂 CAN）

```
【发送】
control/chassis 任务 PID → motor.setMotorCmd(cmd)
  → canNSendTask (1kHz) → packDJIMotorCanMsg()/VESCMotorCanTrans() 组帧
  → CanBus::addCanMsg(pack) → tx_queue_.TryPush() (lockfree_queue)
  → CanBus::txService() → HAL_FDCAN_AddMessageToTxFifoQ() → 硬件发出

【接收】
CAN 硬件收帧 → 中断
  → stm32h7xx_it.c: FDCANx_IT0_IRQHandler()
  → HAL_FDCAN_IRQHandler(&hfdcanx)                 [HAL 库]
  → Canbus.cpp: HAL_FDCAN_RxFifo0Callback()        [HAL 回调]
  → CanBus::processRxInterrupt(): HAL_FDCAN_GetRxMessage() 取报文 → 遍历 device_[6] 按 ID 匹配
  → device->onRx(data, len)                        [Motor.hpp 解析]
  → 更新 speed_/pos_/torque_ → 下一轮 PID 用
```

一句话：`Core/` 管"硬件怎么配"，`BSP/Canbus.*` 管"总线怎么收发"，`Module/motor/Motor.hpp` 管"报文怎么解析"，`APP/robot_com/com_config.cpp` 管"挂哪些设备、什么时候发"。

---

## 三、任务（线程）清单 — 一张表看全

来源：`APP/robot_task/robot_task.cpp` 的 `osTaskInit()`，全部 `osThreadNew`。

| 任务函数 | 干什么 | 周期 | 优先级 |
|----------|--------|------|--------|
| can1SendTask | VESC U8×4 组帧发送 | 1ms | Normal |
| can2SendTask | arm2006/3508/4310 组帧发送 | 1ms | Normal |
| can3SendTask | 底盘 4 电机组帧发送 | 1ms | Normal |
| chassisTask | 底盘运动学逆解 + PID | — | Normal |
| controlTask | 控制逻辑/状态机 | — | BelowNormal7 |
| debugTask | 调试化粪池（新功能先写这） | — | Normal |
| uart2RxProcessTask | IMU 姿态解析 (USART2) | 中断唤醒 | Normal |
| uart3RxProcessTask | 遥控 rm_pocket 解析 (USART3) | 中断唤醒 | Normal |
| uart4RxProcessTask | Position 定位解析 (USART4) | 中断唤醒 | Normal1 |
| uart5RxProcessTask | 上位机/调试 (USART5) | 挂起 | Normal1 |
| DebugSerialTask | 电机在线检测，发 debug 串口 | 10ms | Low |
| usbCdcProcess/SendTask | 二维码相关（已屏蔽） | 挂起 | Normal1 |
| uart3SendTask | 航模回传（挂起） | — | Normal1 |

---

## 四、三条万能搜索线索（RTOS 工程三件套）

别从头读文件，用关键词定位：

1. **从哪开始跑？** 看 `main.c` → 搜 `MX_.*_Init()`（用了哪些外设）、`Robot_Init`（自定义初始化入口）、`osKernelStart`（进 RTOS）
2. **有哪些任务？** 搜 `osThreadNew`（`robot_task.cpp` 一次看全）、搜 `\w+Task\(`
3. **中断从哪进？** 搜 `_IRQHandler`（`Core/Src/stm32h7xx_it.c`）、`_Callback`（HAL 回调）、`HAL_NVIC_`

**数据怎么跨层流动？** 本工程用"话题（Topic）"机制：
- 搜 `TypedTopicPublisher`（谁发布）/ `TypedTopicSubscriber`（谁订阅）/ `\.Publish\(`（数据产生点）/ `\.TryGet\(`（数据消费点）
- 实战：grep `chassis_cmd`，能把 `control_task`（发布）→ `chassis_task`（订阅）串起来

---

## 五、看懂一个功能的万能三步

问三个问题，别问"这个文件是干嘛的"：
1. **输入从哪来？**（中断？串口？CAN？遥控？）
2. **中间怎么处理？**（哪个 task、哪个 PID、哪个解算）
3. **输出到哪去？**（哪个电机、哪条总线）

**例子——"遥控控制底盘"：**
```
CRSF 帧 (USART3 中断) → rm_pocket 解析 → rm_data_pub("rm")
→ control_task 订阅 → JoyToVelocity → chassis_cmd_pub("chassis_cmd")
→ chassis_task 订阅 → OmniChassis 逆解+PID → setMotorCmd
→ can3SendTask → 0x200 CAN 帧 → 电机
```
这 6 步写纸上，工程懂了一大半。

---

## 六、30 分钟上手清单

| 步骤 | 做什么 | 时间 |
|------|--------|------|
| 1 | 读 `readme.md` + `flow_chart/readme.md` + 所有 `whatisit.md`/`howtouse.md` | 5 min |
| 2 | 读 `CMakeLists.txt` + `.ioc`（用了哪些外设/宏） | 3 min |
| 3 | 读 `Core/Src/main.c` 的 `main()`（初始化顺序 + `Robot_Init` 入口） | 3 min |
| 4 | 读 `APP/robot_init/robot.cpp`（初始化总入口 → comServiceInit → osTaskInit） | 2 min |
| 5 | 读 `APP/robot_task/robot_task.cpp`（一张表看清所有任务/优先级/栈） | 3 min |
| 6 | ⭐ 读 `APP/robot_com/com_config.cpp`（所有外设实例 + 发送任务，全局枢纽） | 8 min |
| 7 | 顺着一个 topic 名 grep 一条完整数据流（推荐 `chassis_cmd`） | 6 min |

> 第 6 步最关键：`com_config.cpp` 是全局信息交互中心，读懂它 ≈ 读懂工程骨架。

---

## 七、给队友的提醒

- `Core/` 里 90% 是 CubeMX 自动生成的噪音，只看 `/* USER CODE BEGIN/END */` 之间的内容，其余重新生成会覆盖。
- 改配置回 CubeMX 改 `.ioc`，别手改 `Core/`。
- `debug_task` 是"调试化粪池"，作者正在调什么，看它最快。
- 宏开关（如 `CMakeLists.txt` 里的 `DEBUG_ARM`、`DM_MOTOR_SINGLE`）= 作者留的功能开关，对应注释里的"注意"。
