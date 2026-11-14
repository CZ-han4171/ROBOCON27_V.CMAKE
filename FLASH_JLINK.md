# J-Link 烧录 / 调试说明（STM32H723ZGT6）

> 目标板：学院画的 H7 板（本工程按 **STM32H723ZGT6 / LQFP144 / 1MB Flash** 配置）
> 本机 J-Link：`D:\JLink_V988\JLink.exe`（V9.88 软件，探头 S/N 69730384）
> ⚠️ 别用 `D:\java\bin\jlink.exe`，那是 JDK 的 jlink 工具，不是 SEGGER 的。

---

## 一、怎么烧（两种方式，任选）

> ⚠️ 首次使用 / `build/` 已清空时，**必须先 configure 一次**：
> `cmake --preset Debug`（VSCode 里跑任务 `configure: Debug`，或用 `flash: J-Link`，脚本会自动先 configure）

### 方式 A：VSCode 任务（推荐）

`Ctrl+Shift+P` → `Tasks: Run Task`：

| 任务名 | 作用 |
|--------|------|
| `configure: Debug` | 生成/刷新 `build/Debug` 构建文件（清空 build 后先跑这个） |
| `build: Debug` | 只编译（默认 build 任务，`Ctrl+Shift+B`，会自动先 configure） |
| `flash: J-Link` | 编译 + 烧录 + 复位运行（会自动先 configure） |
| `flash: 探测 SWD 连线` | **不烧录**，只连 J-Link 读 `DBGMCU_IDCODE`，用来判断板子芯片型号和连线是否正常 |
| `flash: 擦除整片再烧录` | 整片擦除后再烧（芯片状态异常时用） |

### 方式 B：命令行

```powershell
cd d:\stm32_my_project\My_ROBOCON27_V.CMAKE_clone

# 先 configure（只在 build 被删过 / 第一次需要）
cmake --preset Debug

# 再编译
cmake --build --preset Debug

# 只探测连线（不烧录，最安全的自检）
powershell -ExecutionPolicy Bypass -File tools\flash_jlink.ps1 -ProbeOnly

# 编译 + 烧录
powershell -ExecutionPolicy Bypass -File tools\flash_jlink.ps1 -Build

# 烧录另一个 elf / 换芯片型号
powershell -ExecutionPolicy Bypass -File tools\flash_jlink.ps1 -Elf .\build\Debug\ROBOCON27_V.CMAKE.elf -Device STM32H743ZI
```

调试：`F5` 选 **J-Link Debug (STM32H723)**（需已装 Cortex-Debug 插件，本机已装 1.12.1）。

---

## 二、故障记录（2026-10-28，**已解决** ✅）

> 根本原因：**板子没有真正上电**（只插 J-Link，靠转接板 LDO 带不动整板）。
> 接 24V、电源灯亮之后，SWD 一次就连上了，详见下一节「二·补」。

排查期间的原始失败日志：

```
J-Link 探头 OK      : Firmware V9 (V9.70), S/N 69730384, VTref = 3.29V
SWD 连接      FAIL  : Error: Failed to initialize DAP
                       Can not attach to CPU. Trying connect under reset.
                       Error occurred: Could not connect to the target device.
```

已排除的项：

- 探头本身能枚举、能报 VTref（3.29V），J-Link 端 OK
- 4800/4000/1000/100 kHz 都试过 → 都失败（不是速度问题）
- SWD 与 JTAG 都试过 → JTAG 会先做 JTAG→SWD 切换，同样 DAP 失败
- `connect under reset`（自动）与 `RSetType 2` 都试过 → 都失败
- OpenOCD 交叉验证：本机 OpenOCD 没有绑定 libusb 驱动，找不到 J-Link（`LIBUSB_ERROR_NOT_FOUND`），所以 OpenOCD 这条路走不通，只能修 J-Link/硬件

结论：**`Failed to initialize DAP` = SWD 线上完全收不到 DP 的应答，属于连线 / 供电 / 芯片状态问题，不是工程代码问题。**

---

## 二·补：2026-10-28 烧录成功记录（可作为以后复现的基线）

### 触发条件
**板子接 24V 上电（电源灯亮）之后**，SWD 立刻就连上了。
→ 之前所有 `Failed to initialize DAP` 的根因就是 **MCU 没有正常供电**（只靠转接板 LDO 带不动整板）。

### 探测结果

```
VTref        = 4.858 V      ← ⚠️ 异常，见下方说明
DBGMCU_IDCODE= 0x10016483   → DEV_ID = 0x483 → STM32H72x/H73x  ✅ 与工程目标一致
FLASHSIZE    = 1024 KB      → 与 STM32H723ZGT6 一致              ✅
```

### 烧录结果

```
J-Link: Flash download: Bank 0 @ 0x08000000: 1 range affected (262144 bytes)
        Total: 2.717s (Erase: 1.667s, Program: 0.603s, Verify: 0.090s)
        Program & Verify speed: 368 KB/s
O.K.
```

### 运行验证（4 个硬指标全过）

| 检查项 | 方法 | 结果 |
|--------|------|------|
| 系统时钟 550 MHz | 停表 300 ms 比较 `CycleCnt`（DWT） | 165M cycles / 0.3 s = **550 MHz** ✅ （说明 HSE 25MHz 起振、PLL 锁定正常） |
| HAL tick（TIM6）1 kHz | `mem32 0x2000019c`（`uwTick`）间隔 1 s 读两次 | 增加 **1002** ✅ |
| FreeRTOS tick 1 kHz | `mem32 0x200006bc`（`xTickCount`） | 增加 **1001** ✅ |
| 任务已创建 | `mem32 0x200006b8`（`uxCurrentNumberOfTasks`） | **0x11 = 17 个任务** ✅ |
| 调度器在跑 | 采样 PC | 落在 `prvIdleTask` / `prvCheckTasksWaitingTermination`（`0x08012670+`），PSP 非 0 ✅ |

> 复现脚本：`tools/jlink_state_probe.jlink`（地址随固件重编译会变，用前先 `arm-none-eabi-nm build\Debug\ROBOCON27_V.CMAKE.elf` 查 `uwTick` / `xTickCount` / `uxCurrentNumberOfTasks` 的地址并替换文件里的 `0x2000....`）。

### ⚠️ VTref = 4.86 V 的隐患（待你确认）

正常应为 3.3 V。**4.86 V ≈ 5 V 减去一个二极管压降**，最可能的解释是：
**SWD 座上那根标着 `3V3` 的针，实际接的是板子的 5V 轨**，电流反灌进转接板的 3.3V LDO 输出（LDO 被顶着）。

处理建议：
1. 用万用表量 SWD 座那根电源针对 `GND`：**是 3.3V 还是 5V**（若是 5V，别把它当 3V3 用）
2. 板子已经自己 24V 上电，**SWD 侧只需要 `GND` + `SWDIO` + `SWCLK`（+`NRST`）**，那根电源针建议直接不接，避免 5V 反灌转接板 LDO
3. SWDIO/SWCLK（PA13/PA14）是 5V 容忍脚，所以现在能正常烧，但长期这么用不合规

### 下次烧录只要三步
```powershell
cd d:\stm32_my_project\My_ROBOCON27_V.CMAKE_clone
# 1) 板子先上 24V（电源灯亮）  2) 确认 SWD 插好
powershell -ExecutionPolicy Bypass -File tools\flash_jlink.ps1 -Build
```

---

## 三、硬件排查清单（按命中概率排序）

| # | 检查项 | 具体怎么查 | 典型现象 |
|---|--------|-----------|----------|
| 1 | **SWDIO / SWCLK 接反** | 20 芯排线 1 脚（红边/VTref）对板子 1 脚；若用杜邦线飞，确认 `SWDIO↔SWDIO`、`SWCLK↔SWCLK`，不是交叉 | DAP 无响应 |
| 2 | **板子没真正上电** | VTref=3.29V 只说明排针上有 3.3V；确认板上电源灯亮、MCU 的 3.3V/1.2V(VCAP) 正常。有的板要接 24V/电池，**只插 J-Link 3.3V 不够** | DAP 无响应 |
| 3 | **SWD 排针没连到 MCU** | 查原理图：SWD 排针是否经跳线帽 / 0Ω 电阻 / 电平转换芯片接到 PA13(SWDIO)、PA14(SWCLK)。有的板子要短接帽 | DAP 无响应 |
| 4 | **GND 没共地** | J-Link 与板子必须共 GND（20 芯排线自带） | 时好时坏 / 完全连不上 |
| 5 | **NRST 悬空且芯片被代码锁住** | 接上 NRST 再试；或按住复位键点烧录，日志出现 "Connecting" 时松手 | 连不上 |
| 6 | **芯片开了读保护 RDP**（师兄的板卖给你前可能开过） | 只有 J-Link 解锁：`JLink.exe -device STM32H723ZG -if SWD -speed 1000`，交互里执行 `unlock`，会**擦掉整片 Flash** | DAP 无响应 / 拒绝访问 |
| 7 | **芯片型号不对**（板子是 H743/H750/H723 但封装/引脚不同） | 探测任务里的 `DEV_ID`：`0x483`=H72x/H73x，`0x450`=H74x/H75x，`0x480`=H7A3 | 连接成功但烧录报错 |
| 8 | **BOOT0 拉高** | BOOT0=1 也可连 SWD，但程序不跑；确认 BOOT0 接地 | 烧完不运行 |

> 排查顺序建议：先跑「探测 SWD 连线」任务 → 若还是 `Failed to initialize DAP`，从第 1 条往下查。

---

## 三·补：4-pin SWD 座 + 转接板专项排查（当前正在卡的点）

现象：`Failed to initialize DAP`，**速度 4000/1000/100/50 kHz 全试过，SWD/JTAG 都试过，全部失败**。
J-Link 报 `VTref=3.288V` 且每次读数几乎不动 —— **注意：很多 J-Link 转接板（OB 型/带 LDO 的转接小板）会用自己的 3.3V 供 VTref，此时 VTref 有 3.3V 并不能证明 H7 板已上电**。

### 1) 先用万用表确认（板子上电、J-Link 先拔掉）

| 测量点 | 正常值 | 异常含义 |
|--------|--------|----------|
| 板子 `3V3` 对 `GND` | 3.3V | 0V → 板子没上电（很多板要接 24V / 电池，光靠 J-Link 3.3V 带不动整板） |
| MCU `VCAP` 对 `GND` | ≈1.2V | 0V → 芯片核心电源不正常 / 芯片没焊好 |
| **`SWDIO` 对 `GND`** | **≈3.3V**（PA13 内部上拉） | 0V → SWDIO 断线、短路到地，或没接到 MCU 的 PA13 |
| **`SWCLK` 对 `GND`** | **≈0V**（PA14 内部下拉） | 3.3V → SWCLK 断线/接错到别的信号 |

> 这两条电压判据是最好用的：**SWDIO 应该是高、SWCLK 应该是低**。只要其中一个不对，就说明 4-pin 座到 MCU 之间的线有问题，怎么烧都烧不进去。

### 2) 断电测通断（万用表蜂鸣档）

从 **SWD 座引脚** 打到 **MCU 的 PA13 / PA14 引脚**（LQFP144 上一般是第 105 / 109 脚，**务必对照学院板原理图确认**）：

- 座 `SWDIO` ↔ MCU `PA13`：应导通（< 几欧）
- 座 `SWCLK` ↔ MCU `PA14`：应导通
- 座 `GND` ↔ 板子 GND：应导通

不通 = 座上这几个针没接到 MCU（有的板要插跳线帽 / 焊 0Ω 电阻 / 拨码开关）。

### 3) 4-pin SWD 座常见丝印顺序（极易接反）

```
A: [3V3] [SWDIO] [SWCLK] [GND]     ← 最常见（ST-Link 风格）
B: [GND] [SWCLK] [SWDIO] [3V3]     ← A 的 180° 反向
C: [3V3] [SWCLK] [SWDIO] [GND]     ← DIO/CLK 对调（接上也不通！）
D: [GND] [SWDIO] [SWCLK] [3V3]
```

**必须按丝印逐针接**，不要按"排线顺序"猜。若转接板是 A 而板子是 B，则 3V3 和 GND 会短在一起（危险），SWDIO/SWCLK 也接到反。
另外：**NRST 建议一定接上**（4-pin 座常只有 4 根线没 NRST，那就把 NRST 单独飞一根到板子复位键那一侧），这样 `connect under reset` 才能真正生效。

### 4) 转接板自身要确认

- 转接板是否要求"目标板供电"还是"由转接板给目标供电"（有 LDO 的那种会对外输出 3.3V）
- 转接板的 20-pin 侧插头有没有插反（1 脚 = VTref，通常红边/三角标记）
- 若转接板上有 `VTref` 选择跳线，选目标板电压那档

### 5) 最快的判定方法

拔掉 J-Link，只给板子上电，用万用表量 4-pin 座的 `SWDIO`：

- **量到 3.3V** → 座到 MCU 的连接是好的，问题在 J-Link/转接板那一侧（换转接板、换排线、确认 1 脚方向）
- **量到 0V** → 座到 MCU 断了，或者板子根本没上电，问题在板上

### 6) 你的板子实测针序（重要！）

学院板 4-pin SWD 座（丝印从左到右 / 1→4 针）：

```
板子： [1: GND] [2: SWCLK] [3: SWDIO] [4: 3V3]
```

而常见转接线 / ST-Link 风格是：

```
转接板： [1: 3V3] [2: SWDIO] [3: SWCLK] [4: GND]
```

⚠️ **这两种直接对插是错的**：1 针对 1 针时 `3V3 ↔ GND` 短路，`SWDIO ↔ SWCLK` 对调 → 现象就是 `Failed to initialize DAP`（甚至转接板 LDO 发烫）。

#### 正确接法（按信号名对，不按针序对）

| 板子 4-pin 座 | 接到转接板 | J-Link 20 芯排线原始脚位 |
|---------------|------------|---------------------------|
| 1 `GND`       | `GND`      | 4/6/8/…/20（GND） |
| 2 `SWCLK`     | `SWCLK`    | **9（TCK）** |
| 3 `SWDIO`     | `SWDIO`    | **7（TMS）** |
| 4 `3V3`       | `3V3/VTref`| 1（VTref） |
| （建议）`NRST`| `NRST`     | 15（RESET） |

两种改法，任选其一：

1. **把 4-pin 插头整体反 180° 插**（如果插头没有防呆卡扣）
   —— 反插后正好是 `[3V3][SWDIO][SWCLK][GND] → [GND][SWCLK][SWDIO][3V3]`，四个信号全部对上。
2. **插头有防呆/是一体成型线**：把 4 根杜邦线按上表重新排针序（或自己压一个新的反序头）。

> 接反时若已经上电过，先检查转接板有没有发烫、板子 3V3 有没有被拉低（有可能烧掉转接板的 LDO）。

改完立刻跑：

```powershell
cd d:\stm32_my_project\My_ROBOCON27_V.CMAKE_clone
powershell -ExecutionPolicy Bypass -File tools\flash_jlink.ps1 -ProbeOnly
```

看到 `DEV_ID = 0x483 → STM32H72x/H73x` + `FLASHSIZE = 1024 KB` 就说明通了。

### 7) 实测结论：VTref 不能当作"板子上电"的判据

2026-10-28 实测：**把 4-pin 插头从 H7 板上完全拔掉后，J-Link 仍报 `VTref = 3.285V`。**

结论：
- 这个 3.3V 来自 **转接板自身的 LDO**，不是板子给的
- 所以"VTref 有 3.3V"**完全不能证明 H7 板上电**
- 反过来也说明：**板子到底有没有电，现在还是未知数**，必须单独确认

要确认板子有电，只有两种可靠办法：

1. 看板上电源指示灯（接 24V / 电池后应亮）
2. 万用表量板子 `3V3` 对 `GND`，再量 MCU `VCAP` 对 `GND`（≈1.2V）

> 注意：很多比赛主控板**必须接 24V / 电池**才会生成 3.3V；只插 J-Link 靠转接板 LDO 那点电流，整板（LED、CAN 收发器、运放等）根本带不起来，MCU 会处于欠压/反复复位状态，现象就是 SWD 完全无应答。

### 8) 探头自检（怀疑 J-Link 本身时）

拿一块已知能用的板子（比如你自己的 F407 探索者板，接 20-pin JTAG 座）试：

```powershell
& 'D:\JLink_V988\JLink.exe' -device STM32F407ZG -if SWD -speed 1000 -autoconnect 1
```

- 能连上 → 探头 + 排线没问题，问题全在 H7 板侧（供电 / SWD 座 / 芯片）
- 连不上 → 探头或排线有问题（克隆 J-Link 的 SWD 输出坏掉是存在的）

---

## 四、能连上以后：第一次烧录建议

1. 跑 `flash: J-Link`（会自动编译 + 烧录 + `r` 复位 + `g` 运行）
2. 看到 `================ 烧录成功 ================` 后再看板子表现
3. 若提示 `DAP 已连上但 loadfile 失败`：多半是芯片型号/Flash 选项字节不匹配，用「擦除整片再烧录」

---

## 五、这个工程烧到"别人的 H7 板"上要注意的软件层前提

工程是按师兄那块板子的硬件参数生成的，换板子时这些"隐形假设"必须先对上，否则会出现"烧进去了但跑不起来 / 外设全哑"：

| 项目 | 工程里的设定 | 在哪看 | 换板子要做什么 |
|------|--------------|--------|----------------|
| 芯片 | `STM32H723ZGT6`，LQFP144，1MB Flash | `ROBOCON26.ioc` 的 `Mcu.CPN` | 型号不同必须回 CubeMX 重生成，不能只改 `-Device` |
| 外部晶振 | **25 MHz**（HSE），`PLLM=2 PLLN=44 PLLP=1` → **SYSCLK = 550 MHz** | `Core/Src/main.c:150-203`、`stm32h7xx_hal_conf.h` 的 `HSE_VALUE` | 板上不是 25MHz 晶振 → 时钟树全错（CAN 波特率、串口全乱） |
| 供电方式 | `HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY)` + `VOS0`（550MHz 必须 VOS0） | `main.c:157-161` | 板子用 SMPS 供电（H723 有 SMPS 型号）时要改成 `PWR_SMPS_SUPPLY` 或 `PWR_SMPS_1V8_SUPPLIES_LDO` |
| RAM 布局 | .data/.bss 在 DTCM(0x20000000,128K)，DMA 缓冲在 D2(0x30000000)，PID 热代码从 ITCM 运行 | `STM32H723XG_FLASH.ld` | 与板子无关，但 H7 的 **DTCM 不能被 DMA 访问**，自己加 DMA 缓冲别放 DTCM |
| CAN | FDCAN1 PD0/PD1、FDCAN2 PB12/PB13、FDCAN3 PF6/PF7，时钟源 HSE=25MHz | `Core/Src/fdcan.c`、`ARCHITECTURE.md` | 引脚/收发器与师兄板不同则要改；无收发器时 CAN 完全不通 |
| 串口 | USART1/2/3/4/5/6/7/8/9/10 全开 | `Core/Src/usart.c` | 引脚不同会和外设冲突 |
| USB | USB_DEVICE CDC 初始化放在 `osKernelInitialize()` 前后有讲究 | `main.c:97,120` | 见注释，不要随意挪位置 |

> 如果新板子和师兄的板子**不是同一版 PCB**，最稳的做法是：拿 `.ioc` 在 CubeMX 里按新板子改引脚/时钟 → 重新生成 `Core/` → 再编译烧录。只改 `-Device` 解决不了外设引脚问题。

---

## 六、常用 J-Link 命令备忘（不用脚本时）

```powershell
# 图形化（推荐新手看内存/寄存器）
& 'D:\JLink_V988\JLink.exe'

# 只连一下，看能不能连上
& 'D:\JLink_V988\JLink.exe' -device STM32H723ZG -if SWD -speed 1000 -autoconnect 1

# 连上后手动烧（在交互界面里敲）
#   r           复位
#   h           停机
#   loadfile d:/stm32_my_project/My_ROBOCON27_V.CMAKE_clone/build/Debug/ROBOCON27_V.CMAKE.elf
#   r           复位
#   g           运行
#   mem32 0x5C001000 1    读 DBGMCU_IDCODE（看芯片型号）
#   unlock                解锁 RDP（会擦全片！）
```

---

## 七、清理与重建（空间不够时怎么做）

| 想清什么 | 怎么做 | 说明 |
|----------|--------|------|
| 编译中间产物 `build/`（约 123 MB） | `Remove-Item build -Recurse -Force` | 可随时重建，源码不受影响 |
| 依赖下载缓存（pip） | `python -m pip cache purge` | 本次实测清出 **1056 MB** |
| 依赖下载缓存（npm） | `npm cache clean --force` | 本次实测清出 **约 411 MB** |
| 安装包残留 | 删 `%TEMP%\JLink_Windows.exe` | J-Link 装完后就不再需要 |

清完后重建（**必须先 configure**）：

```powershell
cd d:\stm32_my_project\My_ROBOCON27_V.CMAKE_clone
cmake --preset Debug          # 只生成构建文件，约 6 秒
cmake --build --preset Debug  # 真正编译
```

> `tools/flash_jlink.ps1 -Build` 与 VSCode 任务 `build: Debug` 都已内置"发现没 configure 就自动 configure"，直接用也不会报 `not configured`。

