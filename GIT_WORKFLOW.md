# Git 操作手册

> 适用：本仓库 `ROBOCON27_V.CMAKE`
> 环境：Windows + PowerShell（VS Code 集成终端）
> 更新：2026-11-14
>
> 原则：**先编译，再提交；先备份，再动历史。**
> 凡是会改写已推送提交的操作（amend / rebase / reset），动手前先打个 tag。

---

## 熟练后只看这段（TL;DR）

```powershell
git status --short                                              # 1. 看改了什么
cmake --build build/Debug 2>&1 | Select-String 'error:|FAILED'  # 2. 编译必须过
git add -A                                                      # 3. 暂存
git commit -m "feat(模块): 说明"                                 # 4. 提交
git fetch origin; git rebase origin/main                        # 5. 同步远端
git push origin HEAD                                            # 6. 推送
```

改了上一次提交用 `git commit --amend --no-edit`，然后
`git push --force-with-lease origin HEAD`。

卡住了看第 6 章；特性分支迁移到重构后的 main，看第 8 章（含完整实战记录）。

> 📌 本文件已随 `d5c6b02` 提交入库，以后正常 `git add GIT_WORKFLOW.md` 即可。

---

## 0. 一次配好，以后不用管

在 PowerShell 里执行一次即可，全局生效：

```powershell
# 用 VS Code 当提交信息编辑器，避免卡在 vim 里出不来
git config --global core.editor "code --wait"

# 统一用 UTF-8，防止中文提交信息变乱码
git config --global i18n.commitEncoding utf-8
git config --global i18n.logOutputEncoding utf-8

# 让 git 不会弹 less 分页器卡住终端
git config --global core.pager ""

# 换行符：Windows 上也统一存 LF，避免整文件 diff
git config --global core.autocrlf input

# 让 pull 默认 rebase（本仓库单人开发 + PR，需要线性历史）
git config --global pull.rebase true
```

检查是否生效：

```powershell
git config --global --list
```

**中文乱码怎么办**（`git log` 显示 `鍦ㄩ娆′笂浼犵増鏈` 这种）：

```powershell
[Console]::OutputEncoding = [System.Text.Encoding]::UTF8
```

只在当前终端窗口有效。想永久生效加进 PowerShell 配置文件（`notepad $PROFILE`）。

---

## 1. 日常六步：改完代码 → 推上去

这是 95% 的情况，照抄即可。

### 第 1 步：看现在有什么改动

```powershell
git status --short
git diff --stat
```

`M` = 改过，`??` = 新文件，`A` = 已暂存。

### 第 2 步：编译通过再往下走（**别跳**）

```powershell
cmake --build build/Debug 2>&1 | Select-String -Pattern 'error:|warning:|FAILED' | Out-String
```

输出为空 = 0 错 0 警。有 `error:` 就先修，**不要提交编译不过的代码**。

### 第 3 步：暂存

```powershell
# 全部改动
git add -A

# 或者只挑某几个文件
git add APP/robot_com/spi_can_config.cpp BSP/Spi_Can/Mcp2518fd.cpp
```

看一眼暂存了什么：

```powershell
git status --short          # 第一列有字符 = 已暂存
git diff --cached --stat
```

### 第 4 步：提交

```powershell
git commit -m "fix(can4): 修正 calcTiming 位宽截断与过滤器掩码语义"
```

**提交信息规范**（对齐仓库现有历史）：

| 前缀 | 用途 | 例子 |
|---|---|---|
| `feat(模块):` | 新功能 | `feat(can4): 接入第 4 路 CAN` |
| `fix(模块):` | 修 bug | `fix(can4): 修正波特率被截断` |
| `refactor(模块):` | 重构，不改行为 | `refactor(can4): 取消独立设备层` |
| `docs:` / `chore:` | 文档 / 杂项 | `chore: 指定 JLink 路径` |

- 标题一行，**不超过 50 字**
- 用动词开头，说清"改了什么"，不是"改了某文件"

### 第 5 步：拉远端（防止 push 被拒）

```powershell
git fetch origin
git rebase origin/main
```

详见第 3 章。如果远端没新东西，这两条会立刻返回，不痛。

### 第 6 步：推送

```powershell
git push origin HEAD
```

第一次推新分支要建立跟踪：

```powershell
git push -u origin feat/your-branch
```

---

## 2. 只改上一次提交（amend）

**场景**：刚 commit 完，发现漏了一个文件 / 注释写错了，不想多一个"补丁提交"。

```powershell
# 把漏的文件加进上一次提交，提交信息不变
git add 漏掉的文件
git commit --amend --no-edit

# 想顺便改提交信息
git commit --amend -m "新的提交信息"
```

**只改提交信息、不动文件**：

```powershell
git commit --amend --no-edit
```

### ⚠️ amend 之后必须强推

amend 会生成一个**新的** commit hash，本地和远端就不一致了：

```powershell
git push --force-with-lease origin feat/your-branch
```

- `--force-with-lease` **不是** `-f`。它会先检查远端分支是否还停在你上次 fetch 的位置；如果别人往这个分支推过东西，会**拒绝**覆盖，救你一命。
- `-f` 会无脑覆盖，把别人的提交冲掉。**永远别用 `-f`**。

### amend 前先备份

```powershell
git tag backup/before-amend-20261114
```

出事了能回来：

```powershell
git reset --hard backup/before-amend-20261114
```

---

## 3. pull：把远端的改动拉进来

### 3.1 两种方式，选哪个

| 方式 | 命令 | 历史形状 | 什么时候用 |
|---|---|---|---|
| **rebase**（推荐） | `git pull --rebase` | 线性，你的提交排在别人后面 | 自己分支还没给别人用 / 想让 PR 干净 |
| merge | `git pull --no-rebase` | 多个 merge 提交 | 分支已被别人拉走 / 团队约定用 merge |

本仓库约定用 **rebase**（`pull.rebase=true` 已经配好）。

### 3.2 标准三步

```powershell
# 1. 先把远端的最新状态下载下来（不合并，安全）
git fetch origin

# 2. 看看落后多少、领先多少
git rev-list --left-right --count origin/main...HEAD
#   输出 "15  2" = 远端有 15 个你没有的提交，你有 2 个远端没有的

# 3. 把自己的提交"搬"到远端最新之上
git rebase origin/main
```

**注意 `fetch` 和 `pull` 的区别**：
- `git fetch` = 只下载，不动你的工作区。**安全，随时可以跑。**
- `git pull` = `fetch` + `merge/rebase`，会动历史。**可能进冲突。**

养成习惯：**先 `fetch` 看一眼，再决定要不要 rebase。**

### 3.3 rebase 中途停下来要你解冲突

会看到：

```
CONFLICT (content): Merge conflict in APP/robot_task/robot_task.cpp
error: could not apply 3f2a1b9... feat(can4): ...
```

别慌，看第 4 章。解完继续：

```powershell
git add 冲突文件
git rebase --continue
```

想**整个放弃**，回到 rebase 之前：

```powershell
git rebase --abort
```

### 3.4 工作区有未提交改动时不能 rebase

rebase 会先报：

```
error: cannot rebase: You have unstaged changes.
```

先处理掉，二选一：

```powershell
# 方案 A：先提交（推荐）
git add -A
git commit -m "wip: 临时保存"

# 方案 B：暂时收起来
git stash push -m "rebase 前暂存"
# ... rebase 完成后 ...
git stash pop
```

---

## 4. 冲突怎么解

### 4.1 先看清楚哪几个文件冲突

```powershell
git status --short
```

冲突文件前面是 `UU`（both modified）或 `AA`。

### 4.2 打开文件，找这三种标记

```cpp
<<<<<<< HEAD
// 远端 main 上的版本
int a = 1;
=======
// 你本地的版本
int a = 2;
>>>>>>> feat(can4): 接入第 4 路 CAN
```

**处理方式：把 `<<<<<<<`、`=======`、`>>>>>>>` 三行删掉，中间只留你要的代码。**

不要两个都留，也不要把标记留在文件里。

### 4.3 有冲突标记的文件一定编译不过

解完先搜一遍有没有漏掉的：

```powershell
Select-String -Path (git diff --name-only --diff-filter=U) -Pattern '^(<<<<<<<|=======|>>>>>>>)'
```

输出为空 = 干净了。

### 4.4 标记冲突已解决 → 继续

```powershell
git add 冲突文件
git rebase --continue
```

如果所有改动都已经在暂存区，rebase 会问你"要不要把剩余改动也提交"，一般直接回车确认即可。

### 4.5 解完必须重新编译

冲突解错很常见，**一定要重编**：

```powershell
cmake --build build/Debug 2>&1 | Select-String -Pattern 'error:|warning:|FAILED' | Out-String
```

### 4.6 不想解了，退回去

```powershell
git rebase --abort
```

工作区和 HEAD 都回到 rebase 开始的那一刻，**不会有任何损失**。

---

## 5. push：三种情况

### 5.1 普通推送（分支已存在，历史没被改写）

```powershell
git push origin feat/your-branch
```

### 5.2 第一次推新分支

```powershell
git push -u origin feat/your-branch
```

`-u` = 建立跟踪关系，以后直接 `git push` 就行。

### 5.3 amend / rebase 之后的强推

```powershell
git push --force-with-lease origin feat/your-branch
```

| 命令 | 行为 | 用不用 |
|---|---|---|
| `git push` | 被拒（non-fast-forward） | — |
| `git push -f` | **无条件覆盖远端**，可能冲掉别人的提交 | ❌ 禁用 |
| `git push --force-with-lease` | 远端还等于我上次看到的状态才覆盖 | ✅ 用这个 |

`--force-with-lease` 报这个错时：

```
! [rejected] ... (stale info)
```

说明**别人往这个分支推过东西**，你的本地信息过期了。这时：

```powershell
git fetch origin
git log --oneline HEAD..origin/feat/your-branch    # 看别人加了什么
```

有别人的提交 → **别再强推**，先 rebase 把它们拉进来，或者问一下对方。

### 5.4 推送前最后检查

```powershell
git status                    # 工作区应该是 clean
git log --oneline -5          # 确认提交信息对不对
git diff origin/main --stat   # 确认只改了你想要的
```

---

## 6. 出事了怎么救

### 6.1 万能后悔药：`reflog`

`git reflog` 记录本地 HEAD 的**每一次**移动（切分支、提交、rebase、reset），默认保留 90 天。
**只要提交过，就一定能找回来。**

```powershell
git reflog --date=short
```

输出形如：

```
a1b2c3d HEAD@{2026-11-14 10:30:12}: rebase (finish): returning to refs/heads/feat/xxx
9f8e7d6 HEAD@{2026-11-14 10:28:44}: rebase (start): checkout origin/main
1234abc HEAD@{2026-11-14 10:20:01}: commit: fix(can4): 上机前缺陷修复
```

找错了就回到某个状态：

```powershell
git reset --hard 1234abc
```

### 6.2 撤销刚做完的 rebase / merge

```powershell
git reset --hard ORIG_HEAD
```

`ORIG_HEAD` 是 git 在做 rebase/merge 前自动存的"上一个 HEAD"。

### 6.3 提交了不该提交的文件

```powershell
# 从这次提交里撤下，文件内容保留在工作区
git rm --cached 路径/文件

# 或者从暂存区撤下（不动工作区文件）
git restore --staged 路径/文件

# 同时把文件内容也还原
git restore 路径/文件

# 然后 amend 掉上一次提交
git commit --amend --no-edit
```

### 6.4 提交到错误的分支（还没 push）

```powershell
git reset --soft HEAD~1        # 撤销提交，改动全留在暂存区
git switch 正确的分支
git commit -m "原来的信息"
```

### 6.5 已经 push 出去才发现错了

**不要在公共分支上改写历史**（rebase / reset），用 `revert` 生成一个反向提交：

```powershell
git revert <出事的commit>      # 会打开编辑器让你确认信息
git push origin feat/your-branch
```

好处：别人的本地仓库不会因为你改写历史而坏掉。

### 6.6 当前状态彻底看不懂了

三件事按顺序做：

```powershell
git status                      # 你处在什么状态（rebase 中途？merge 中途？）
git log --oneline --graph -10   # 历史长什么样
git reflog -10                  # 刚才干了什么
```

如果正卡在 rebase 中途，`git rebase --abort` 先回到干净状态再分析。

---

## 7. 本仓库分支现状（2026-11-14 实测）

```
origin/main  a86b653  Merge pull request #10          ← 远端主线，最新
     │
     │  ... 15 个提交（PR#4 ~ PR#10：robot_init 重构、多轴控制优化、舵轮控制）...
     │
     08b0977  Merge pull request #3                    ← 分叉点 (merge-base)
     ├── 1345514  fix(can4)
     │      └── fe01be9  feat(can4): 在首次上传版本上接入第 4 路 CAN
     │             ↑ 分支 feat/spi-can-mcp2518fd-prepull（迁移前的原始位置，未推过）
     │
     └── 86ec0f9  Merge pull request #5
            └── c89e820 → aee0426 → eaec165
                    ↑ origin/feat/spi-can-mcp2518fd（PR#6 的头，也是 backup/pre-unpull）
```

**迁移完成后（2026-11-14 实测）：**

```
origin/main  a86b653
     └── ba1590d  fix(can4)
            └── 2b9371c  feat(can4): 在首次上传版本上接入第 4 路 CAN
                   └── d5c6b02  fix(can4): 修正发送路径 + Git 工作流文档
                          └── ……  docs(git): 第 8 章实战记录（即本节内容）
                                 ↑ 分支 feat/can4-on-main（代码 3 提交 + 文档 1 提交）
```

### 关键事实

| 项目 | 迁移前 | 迁移后 |
|---|---|---|
| 当前分支 | `feat/spi-can-mcp2518fd-prepull` | **`feat/can4-on-main`** |
| HEAD | `fe01be9`（2 个提交） | **`d5c6b02`（代码 3 个提交）** + 文档提交 1 个 |
| 有 upstream 吗 | **没有**，远端不存在这个分支 | **没有**（第一次 push 时用 `-u` 自动设） |
| 落后 origin/main | 15 个提交 | **0**（已 rebase 到 `a86b653` 之上） |
| 领先 origin/main | 2 个提交 | **3 个提交** |
| 远端 PR#6 分支 | `origin/feat/spi-can-mcp2518fd` @ `eaec165` | **仍是 `eaec165`，未动** |
| 与 origin/main 的差异 | 37 个文件 | **36 个文件 +12855 / −78** |
| 编译 | 0 error / 0 warning | **0 error / 0 warning** |
| DTCMRAM | 107920 B (82.34%) | **116432 B (88.83%)** ⚠️ |
| FLASH | 151860 B (14.48%) | **167860 B (16.01%)** |

> ⚠️ DTCMRAM 涨到 88.83%，只剩约 14.6 KB。main 新增的 `Encoder_1~4` + `SwerveDrive1~4` + `SwerveSteer1~4` 共 12 个设备实例吃掉了 8512 B。后续加功能要看这条线。

### 已有的备份（别删）

```
分支 feat/can4-on-main     = d5c6b02   （迁移完成的分支，可直接上机）
分支 feat/spi-can-mcp2518fd-prepull = fe01be9 （迁移前的原始位置）
分支 backup/pre-unpull     = eaec165   （PR#6 的头）
分支 backup/wip            = 058ada2   （早期 wip）
tag  backup/can4-step1                 （rebase 前一刻：fe01be9 + 9 改动 + GIT_WORKFLOW.md）
tag  backup/can4-worktree-20261114     （同上内容的提交快照 4609210）
tag  backup/can4-rebased-20261114      （第一次 rebase 的产物 78c034f，编译失败，仅作对照）
tag  backup/can4-before-migrate-20261114 （= fe01be9）
tag  backup/can4-before-reset          （重置前的完整状态）
tag  backup/can4-unified               （统一层方案）
```

### 为什么 36 个差异文件里只有 1 个真冲突

CAN4 的代码 **90% 是纯新增文件**，main 上根本没有，所以不会冲突：

| 类型 | 文件数 | rebase 表现 |
|---|---|---|
| 纯新增 | 19 个（`BSP/Spi_Can/*` 6 个、`APP/robot_com/spi_can_config.*` 2 个、HAL SPI 驱动 5 个、`spi.c/spi.h`、文档 4 个） | 文件级直接放入，**零冲突** |
| CubeMX 自动生成 | 8 个（`.ioc`、`.mxproject`、`gpio.c`、`stm32h7xx_it.c/.h`、`main.h`、`stm32h7xx_hal_conf.h`、`cmake/stm32cubemx/CMakeLists.txt`） | 自动合并 |
| 手写插桩 | 3 个（`main.c` 2 行、`robot_task.cpp` 9 行、`CMakeLists.txt` 3 行） | `main.c`、`robot_task.cpp` **自动合并**；`CMakeLists.txt` 自动合并 |
| **真冲突** | **1 个**：`APP/robot_com/com_config.cpp` | **手工解**（见 8.4） |

> 也就是说：**手写的插桩代码总共只有 14 行**，其中 11 行 git 自己能合，只有 `com_config.cpp` 需要人工介入。

而且 `CMakeLists.txt:67` 用的是：

```cmake
file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS "APP/*.cpp" "BSP/*.cpp" ...)
```

**新增的 .cpp 会自动纳入编译**，不用手工改 CMake 的源文件列表。

---

## 8. CAN4 分支迁移：实战记录（2026-11-14 已完成）

> 这一章**不是预演**，是真实跑过一遍的完整过程 + 踩过的坑，附全部真实输出。
> 以后再遇到"把特性分支迁到重构后的 main"这类活，直接照抄。日常提交只看第 1 章。

### 8.1 五个认知陷阱（先看这个，能省两小时）

| # | 陷阱 | 真相 |
|---|---|---|
| 1 | **rebase 的 `ours`/`theirs` 和 merge 是反的** | `--ours` = **正在 rebase 到的基底**（origin/main 的新版）；`--theirs` = 你自己的提交（旧版） |
| 2 | 以为要解 3 个文件 | 实际只有 `com_config.cpp` **1 个**；`robot_task.cpp`、`main.c` 自动合并 |
| 3 | 以为要先 squash 成 1 个提交 | **不需要**。3 个提交直接 rebase，1/3 干净通过，只在 2/3 停一次 |
| 4 | 以为设备还在 `com_config.cpp` | main 把设备实例搬到 `robot_devices.cpp/.hpp`，把 `.init()` 搬到 `robot_setup.cpp` |
| 5 | 以为 include 还是 `.h` | `com_config.h` → `.hpp`、`robot.h` → `.hpp`、`pid_controller.h` → `.hpp` |

> **第 1 条最致命。** 第一次尝试就是因为没验证方向、拿了旧版本，导致 `com_config.cpp` 里全是 `arm2006_motor` 而 main 已经删掉了这些定义，编译全红。

### 8.2 为什么不能简单 `git pull`

1. **落后 15 个提交**，`origin/main` 重构了 `APP/robot_init/`（设备实例全搬走）、`BSP/Canbus.hpp`、`Module/motor/Motor.hpp`、`Module/Axis/Axis.hpp`。
2. 当前分支**没有 upstream**，`git pull` 直接报错；且默认行为是 merge，会造出 merge commit。
3. 工作区有未提交改动时 rebase 会被拒绝。

### 8.3 实际执行的命令（已跑通，照抄）

```powershell
# ── 步骤 0：开新分支 + 提交 + 备份（保护原分支）
git checkout -b feat/can4-on-main
git add -A
git commit -m "fix(can4): 修正 CAN4 发送路径并补充 Git 工作流文档"
git tag backup/can4-step1

# ── 步骤 1：拉主线
git fetch origin
git rebase origin/main
#   实际输出：Rebasing (1/3) 干净通过 → (2/3) 在 com_config.cpp 冲突 → 停下
#   只有这一个文件会停

# ── 步骤 2：解冲突（详见 8.4）
git checkout --ours -- APP/robot_com/com_config.cpp   # ⚠️ --ours 拿到的才是 main 版
git add APP/robot_com/com_config.cpp

# ── 步骤 3：继续（8.5 的 6 行挂接已完成）
git add APP/robot_init/robot_devices.cpp APP/robot_init/robot_devices.hpp APP/robot_init/robot_setup.cpp
git -c core.editor=true rebase --continue

# ── 步骤 4：完整重编
cmake --build build/Debug --clean-first 2>&1 | Tee-Object -FilePath build_clean.txt
Select-String -Path build_clean.txt -Pattern 'error:|warning:|FAILED|DTCMRAM|FLASH'

# ── 步骤 5：推送（见 8.7）
```

**不需要 squash。** 3 个提交直接 rebase，git 的 cherry-pick 会自己处理掉互相抵消的改动（1/3 会干净通过）。

**`git -c core.editor=true`** 是为了 `--continue` 时不弹编辑器卡住。

### 8.4 唯一的冲突：`com_config.cpp` 怎么解

rebase 停在这里时，**先别急着 checkout，一定要先验证方向**：

```powershell
# 查看 ours（:2:）和 theirs（:3:）各是什么版本
git show :2:APP/robot_com/com_config.cpp | Select-String 'DJI2006_motor1|arm2006_motor'
git show :3:APP/robot_com/com_config.cpp | Select-String 'DJI2006_motor1|arm2006_motor'
```

2026-11-14 的实测结果：

| 引用 | 内容 | 是什么 |
|---|---|---|
| `:2:` **ours** | `DJI2006_motor1`、`U8_motor1` | ✅ **origin/main 的新版** |
| `:3:` **theirs** | `arm2006_motor`、`U8_1` | ❌ 我们的旧版 |

所以：

```powershell
git checkout --ours -- APP/robot_com/com_config.cpp   # 拿 main 新版
git add APP/robot_com/com_config.cpp
```

> **记住这条**：`git merge` 时 `ours` = 当前分支；`git rebase` 时 `ours` = **你正在 rebase 到的那条分支**。两者相反，每次都验证一次最稳。

拿完后快速确认一下：

```powershell
Select-String -Path APP/robot_com/com_config.cpp -Pattern 'com_config.hpp|CAN4_Send_TaskHandle|arm2006_motor'
# 预期：有 com_config.hpp，无 CAN4_Send_TaskHandle，无 arm2006_motor
```

### 8.5 6 行挂接：CAN4 在新架构下的位置

main 把原来 `com_config.cpp` 里的 4 行拆成了 **3 个文件的 6 行**：

| 文件 | 位置 | 内容 |
|---|---|---|
| `APP/robot_init/robot_devices.hpp` | `extern LinearAxis lift3508_axis;` 之后 | `extern C610Motor can4_motor1;` + 2 行注释 |
| `APP/robot_init/robot_devices.cpp` | `LinearAxis lift3508_axis(...)` 之后 | `C610Motor can4_motor1(nullptr, 0x205, 0, 0x1FF, 0);` + 1 行注释 |
| `APP/robot_init/robot_setup.cpp` | `RobotDevices_Init()` 里 `lift3508_motor.init();` 之后 | `can4_motor1.init();` |
| `APP/robot_com/com_config.cpp` | include 区（`robot_devices.hpp` 之后） | `#include "spi_can_config.hpp"` |
| `APP/robot_com/com_config.cpp` | `CAN3_Send_TaskHandle` 之后 | `osThreadId_t CAN4_Send_TaskHandle;` |
| `APP/robot_com/com_config.cpp` | `comServiceInit()` 里 `registerDevice(&lift3508_motor);` 之后 | `SpiCanApp::can4Attach(&can4_motor1);` |

**为什么 `init()` 在 `robot_setup.cpp` 而不在 `com_config.cpp`？**

main 把"设备定义"和"设备初始化"分开了：

```
robot_devices.cpp   定义所有设备实例（全局对象）
robot_setup.cpp     RobotDevices_Init()：总线 init + 设备 init + PID 参数
com_config.cpp      comServiceInit()：只做 registerDevice（挂到 CanBus）
```

调用顺序（`robot.cpp`）：

```cpp
void Robot_Init() {
    RobotDevices_Init();          // 31 行：can4_motor1.init() 在这里
    if (comServiceInit() != 0) {  // 33 行：can4Attach(&can4_motor1) 在这里
        ...
    }
}
```

`init()` 必须在 `attach()` 之前 —— 上面的顺序天然满足。

> ✅ `robot_task.cpp`（`CAN4_Send_TaskHandle` 任务创建）和 `main.c`（`MX_SPI4_Init()`）**自动合并，不用管**。
> 合并后确认一下就行：
> ```powershell
> Select-String -Path APP/robot_task/robot_task.cpp -Pattern 'can4'
> Select-String -Path Core/Src/main.c -Pattern 'SPI4'
> ```

> ⚠️ `Core/Src/spi.c`、`Core/Inc/spi.h`、HAL 的 SPI 驱动 **main 上没有，是纯新增，不会冲突**，会自动加入。

### 8.6 验证清单（含 2026-11-14 真实输出）

```powershell
# 1. 完整编译 0 错 0 警（89 个目标）
cmake --build build/Debug --clean-first 2>&1 | Tee-Object -FilePath build_clean.txt
Select-String -Path build_clean.txt -Pattern 'error:|warning:|FAILED' | Out-String
# 实际：无输出 = 通过

# 2. 内存占用
Select-String -Path build_clean.txt -Pattern 'DTCMRAM|RAM_D|FLASH'

# 3. 新文件确实进了编译
Get-ChildItem build/Debug/CMakeFiles/ROBOCON27_V.CMAKE.dir/BSP/Spi_Can/ -Name
# 预期：Mcp2518fd.cpp.obj、SpiCanTransport.cpp.obj、SpiCanBus.cpp.obj ...

# 4. 提交数与改动量
git log --oneline origin/main..HEAD
git diff --shortstat origin/main

# 5. 工作区干净
git status --short
```

实际结果：

| 检查项 | 结果 |
|---|---|
| 编译目标 | **89 个全部成功** |
| error / warning | **0 / 0** |
| 提交数 | **代码 3 个**（`ba1590d` / `2b9371c` / `d5c6b02`）+ 文档 1 个 |
| 改动量 | **36 文件 +12855 / −78** |
| 工作区 | 干净 |
| 产物 | `build/Debug/ROBOCON27_V.CMAKE.elf` |

内存占用：

| 区域 | 大小 | 占比 |
|---|---|---|
| DTCMRAM | 116432 B / 128 KB | **88.83%** ⚠️ |
| RAM_D1 | 0 B / 320 KB | 0.00% |
| RAM_D2 | 1216 B / 32 KB | 3.71% |
| RAM_D3 | 0 B / 16 KB | 0.00% |
| ITCMRAM | 1048 B / 64 KB | 1.60% |
| FLASH | 167860 B / 1 MB | 16.01% |

### 8.7 推送

**推荐：推到新分支，PR#6 保持不动**

```powershell
git push -u origin HEAD:refs/heads/feat/can4-on-main
```

- ✅ PR#6 的原始历史（`c89e820 → aee0426 → eaec165`）完整保留
- ✅ 师兄看到的是"基于最新 main 的干净 3 提交 diff"
- ✅ `-u` 自动设 upstream，以后直接 `git push` 即可
- ❌ 多一个 PR 要关

**备选：覆盖 PR#6（需明确批准）**

```powershell
git push --force-with-lease origin HEAD:refs/heads/feat/spi-can-mcp2518fd
```

- ✅ PR#6 原地更新，不用重开
- ❌ 远端 3 个旧提交从分支上消失（`backup/pre-unpull` 里还在，但 PR 页面看不到旧代码）
- ❌ 已有 review comment 的行号会全部错位

**建议：先走推荐路径，确认编译和上机都 OK 再决定要不要覆盖。**

### 8.8 迁移前后对比

| 项目 | 迁移前（`fe01be9`） | 迁移后（`d5c6b02`） |
|---|---|---|
| 基线 | `08b0977`（落后 15 个提交） | `a86b653`（同步 main） |
| 提交数 | 2 | 代码 3（+ 文档 1） |
| 差异文件 | 37 | 36 |
| DTCMRAM | 107920 B (82.34%) | 116432 B (**88.83%**) |
| FLASH | 151860 B (14.48%) | 167860 B (16.01%) |
| 编译 | 0 / 0 | 0 / 0 |
| 设备实例定义位置 | `com_config.cpp` | `robot_devices.cpp` |
| `.init()` 位置 | `comServiceInit()` | `RobotDevices_Init()` |
| 设备命名 | `arm2006_motor` / `U8_1` | `DJI2006_motor1` / `U8_motor1` |
| 头文件后缀 | `.h` | `.hpp` |

**DTCMRAM 涨了 8512 B**（+6.5%），来自 main 新增的 `Encoder_1~4`、`SwerveDrive1~4`、`SwerveSteer1~4` 共 12 个设备实例（`can4_motor1` 本身只占几十字节）。**这 88.83% 是当前最大的技术债**，后续加功能要盯着这条线。

---

## 9. 禁止清单

| ❌ 不要做 | 后果 | 该怎么做 |
|---|---|---|
| `git push -f` | 无脑覆盖，冲掉别人的提交 | `--force-with-lease` |
| 直接在 `main` 上 commit | 打乱 PR 流程 | 先 `git switch -c feat/xxx` |
| 提交 `build/` 目录 | 仓库体积爆炸 | 已在 `.gitignore`，别 `git add -f` |
| 提交编译不过的代码 | 队友拉下来就红 | 第 1 章第 2 步 |
| `git add .` 不先看 | 把临时文件/日志带进去 | 先 `git status --short` |
| rebase 已经共享的分支 | 队友本地仓库坏掉 | 用 `git revert` 代替 |
| 解 rebase 冲突时想当然用 `--theirs` 当 main 版 | rebase 里 `theirs` 是**你自己的旧版**，编译全红 | 先 `git show :2:文件` / `git show :3:文件` 验证方向 |
| 删掉 `backup/*` 分支和 tag | 出事没得救 | 至少留一个月 |
| 在 vim 里乱按 | 卡住出不来 | `core.editor "code --wait"` |

### 卡在 vim 里怎么出来

```
Esc  →  输入 :wq  →  回车     （保存并退出）
Esc  →  输入 :q!  →  回车     （不保存退出）
```

### 一个提交该装多少东西

**一个提交只做一件事。**

- ✅ `fix(can4): 修正 calcTiming 位宽截断`
- ❌ `fix(can4): 改时序 + 改注释 + 调栈大小 + 改 launch.json`

例外：本次"上机前缺陷修复"是一整套强相关的小修（同一次上机前准备），合成一个提交可以接受；和功能无关的杂项（改 `core.editor`、改工具脚本）单独用 `chore:` 提交。

> ⚠️ **本仓库 `.gitignore` 里有 `build`、`.vscode`、`.settings`、`*.bat`**，
> 所以 `.vscode/launch.json` 的改动**不会出现在 git 里**（`git ls-files .vscode` 输出为空 = 未跟踪）。
> 改了就只对自己本机生效，不需要、也没法提交。

---

## 10. 常用命令速查

```powershell
# 状态（每天最先跑的三条）
git status --short
git log --oneline --graph -10
git diff --stat

# 分支
git branch -vv                  # 本地分支 + upstream + 落后/领先
git switch -c feat/xxx          # 新建并切换
git switch main                 # 切到 main

# 同步
git fetch origin
git rev-list --left-right --count origin/main...HEAD
git rebase origin/main

# 提交
git add -A
git commit -m "feat(模块): 说明"
git commit --amend --no-edit

# 推送
git push -u origin feat/xxx
git push --force-with-lease origin feat/xxx

# 撤销
git restore --staged 文件
git restore 文件
git reset --soft HEAD~1
git reset --hard ORIG_HEAD
git rebase --abort

# 找回
git reflog --date=short
```

---

> 这份文档放在仓库根目录，流程变了就顺手更新它。
> 卡住时先跑这三条，能解释 90% 的情况：
>
> ```powershell
> git status
> git log --oneline --graph -10
> git reflog -10
> ```
