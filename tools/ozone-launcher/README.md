# CMake Ozone Launcher（本地扩展）

将此项目底部原有的 CMake 启动小三角替换为同位置的 Ozone 小三角。
点击后通过 CMake Tools 保存、配置和构建当前选择的目标；只有成功生成 ELF 后才启动 Ozone。
Debug / Release 及 Windows / Linux 的路径均由当前 CMake 目标决定。

## Windows / Ubuntu 分开使用（2026-10-25）

Windows：直接打开项目文件夹，或原来的 `../ROBOCON27_V.CMAKE.code-workspace`。
使用原生 CMake 三角按钮，构建 ELF 后沿用 Windows 的 Ozone 文件关联。
Windows 不需要安装本扩展。

Ubuntu：使用已经安装本扩展的 VS Code 配置文件（Profile），通过“文件 → 从文件打开工作区”
打开项目目录旁的 `../ROBOCON27_V.CMAKE.Ubuntu.code-workspace`。
该工作区启用 Ozone 扩展按钮并隐藏原生按钮。以后从“最近打开”进入这个 Ubuntu 工作区即可。
本扩展的安装位置和 Ozone Linux 启动路径保持原样。

共享的 `.vscode/settings.json` 不再设置 `ozoneLauncher.enabled` 或隐藏 CMake 启动按钮，
避免 Ubuntu 的界面设置影响 Windows。不要在 Windows 打开 Ubuntu 专用工作区。

Ubuntu 专用工作区已包含以下设置，无需再写入共享 `.vscode/settings.json`：

```json
{
  "ozoneLauncher.enabled": true,
  "cmake.buildBeforeRun": true,
  "cmake.options": {
    "advanced": {
      "launch": { "statusBarVisibility": "hidden" }
    }
  }
}
```

重新加载 VS Code 窗口。鼠标放到三角上应显示“构建所选 CMake 目标并打开 Ozone 新建项目向导”。
其他项目默认不启用；CMake 侧栏的原始运行按钮保持原本行为。

Ozone 从 PATH、`OZONE_EXECUTABLE` 环境变量或 SEGGER 默认安装目录查找。
非默认安装位置可在每台电脑的 VS Code 用户设置中设置 `ozoneLauncher.executable`。
启动参数仅包含 `-programfile` 和当前 ELF 路径。Ozone 会打开新建项目向导；
ELF 路径由按钮填入，芯片、连接方式和烧录器由用户在向导中选择。
按钮不会预先生成 `.jdebug` 项目，也不会替用户选择 J-Link USB。

Windows 上已有的原生文件关联方式可以继续使用，无需安装此扩展。
仅打开普通项目文件夹时不会启用本扩展；Ubuntu 请使用上述专用工作区。

卸载时在 Ubuntu 工作区中删除 `ozoneLauncher.enabled` 设置，并将原按钮可见性恢复为 `inherit`。
