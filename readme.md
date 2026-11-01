待完成：底盘速度外环pid，底盘位置外环pid
3508电机pid参数（粗调）：

转子端速度环：P:150,I:20,D:0 位置式

输出端位置环：P:400,I:80,D:0 位置式

目前是本工具链是生产elf通过ozone进行调试
现在能用jlink，也能用无线daplink，但是无线调试不能看曲线
怎么用无线dap呢：
https://bbs.robomaster.com/article/1638268?source=1

目前数据内存默认都存储在DTCMRAM里，其他SRAM基本没用上，如果以后 DTCM 接近满了，可以考虑把不敏感的大数组/缓存挪到 D1/D2，具体怎么改到时再看看
除此之外，我（fzh）把pid计算放在了ITCMRAM里，我也不知道这能够优化多少
上述说的这俩RAM的优势是什么呢
DTCMRAM
Data TCM RAM，紧耦合数据内存，速度非常快，通常给实时性高的数据/栈
ITCMRAM
Instruction TCM RAM，紧耦合指令内存，通常放极高实时性的代码

后续的开发流程非常之明了啊，
要添加外设的话：在module层调用BSP层的固件接口（目前只有串口，而且默认是开dma的）->在module层写功能函数以及封装函数接口->APP成调用你的功能函数

要添加机器人执行逻辑：在APP层调用所需的功能函数->debug任务书写功能逻辑、状态机->测试->新建个任务文件把内容装里面
<img width="850" height="857" alt="6324839ae149e8f52d0411ab866e990b" src="https://github.com/user-attachments/assets/525838a6-2a47-48ca-a4d0-c90de74be154" />


## Windows / Linux 来回开发

两边继续选择同名的 `Debug` 或 `Release` CMake 预设。`CMakePresets.json`
根据 `${hostSystemName}` 自动隔离构建目录：

| 系统 | Debug | Release |
| --- | --- | --- |
| Windows | `build/Windows/Debug` | `build/Windows/Release` |
| Linux | `build/Linux/Debug` | `build/Linux/Release` |

切换系统后，在 VS Code 执行 **CMake: 选择配置预设**，选择 `Debug` 或 `Release`，
再执行 **CMake: 配置** 和 **CMake: 生成**。两边都需要安装各自平台的 STM32Cube
工具链、CMake 和 Ninja；不要把一边的编译器绝对路径写进共享预设。
旧的 `build/Debug`、`build/Release` 不再使用，可以保留作备份。
如果在同一个系统上移动了项目目录或更换了工具链，执行 **CMake: 删除缓存并重新配置**。

VS Code 的 CMake Tools 每次配置后将当前平台、当前预设的编译数据库复制到
`build/compile_commands.json`，供 clangd 和 C/C++ 扩展读取。切换系统后先配置，
如补全未刷新，再重启语言服务器或重新加载窗口。
`.vscode` 和 `.clangd` 当前被 Git 忽略；若通过 Git 在另一份检出中同步项目，
请同步以下本地设置（同一共享目录无需重复设置）：

```json
{
    "cmake.useCMakePresets": "always",
    "cmake.copyCompileCommands": "${workspaceFolder}/build/compile_commands.json"
}
```

在 `.clangd` 的 `CompileFlags` 下设置 `CompilationDatabase: build`；
如使用 `c_cpp_properties.json`，将 `compileCommands` 设置为
`${workspaceFolder}/build/compile_commands.json`。

在已加载 STM32 工具链环境的终端中也可构建：

```sh
cmake --preset Debug
cmake --build --preset Debug
```

终端直接运行 CMake 不会执行 VS Code 的数据库复制；如需刷新编辑器补全，
回到 VS Code 执行一次 **CMake: 配置**。使用 Ozone 等外部调试器时，
将 ELF 路径指向当前系统构建目录中的 `ROBOCON27_V.CMAKE.elf`。

## 底部小三角打开 Ozone 新建项目向导

此电脑的 `stm32` VS Code 配置文件已安装本地扩展 **CMake Ozone Launcher (Local)**。
重新加载窗口后，底部原位置的小三角会提示“构建所选 CMake 目标并打开 Ozone 新建项目向导”。
点击后先构建当前目标；失败或取消时不启动 Ozone。成功后向 Ozone 传入当前系统、
当前预设生成的 ELF 路径，再由用户在向导中选择芯片、连接方式及烧录器。
扩展不会预设 J-Link USB，也不会生成 `.jdebug`。

扩展源码和设置说明在 `tools/ozone-launcher/`。另一台电脑或 Windows 需要安装同一份 VSIX
才会显示替换后的按钮。如果 Windows 继续使用原生文件关联而不装扩展，需将
`cmake.options.advanced.launch.statusBarVisibility` 恢复为 `inherit`。
