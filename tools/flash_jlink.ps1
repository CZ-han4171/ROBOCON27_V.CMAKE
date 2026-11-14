<#
.SYNOPSIS
    用 SEGGER J-Link 烧录 / 探测 STM32H723 工程（ROBOCON27_V.CMAKE）。

.DESCRIPTION
    默认烧录 build\Debug\ROBOCON27_V.CMAKE.elf。
    -ProbeOnly 只做 SWD 连接自检（读 DBGMCU_IDCODE / FLASHSIZE），用于排查连线。

.EXAMPLE
    # 只探测连线（不烧录）
    powershell -ExecutionPolicy Bypass -File tools\flash_jlink.ps1 -ProbeOnly

.EXAMPLE
    # 先编译再烧录
    powershell -ExecutionPolicy Bypass -File tools\flash_jlink.ps1 -Build

.EXAMPLE
    # 整片擦除后烧录（芯片被写坏/需要干净状态时用）
    powershell -ExecutionPolicy Bypass -File tools\flash_jlink.ps1 -Erase
#>
[CmdletBinding()]
param(
    [string]$Elf,
    [string]$JLinkExe,
    [string]$Device = 'STM32H723ZG',
    [ValidateSet('SWD', 'JTAG')][string]$Interface = 'SWD',
    [int]$Speed = 4000,
    [switch]$Build,
    [switch]$Erase,
    [switch]$ProbeOnly
)

$ErrorActionPreference = 'Stop'
$root = $PSScriptRoot | Split-Path -Parent

function Find-JLinkExe {
    param([string]$Explicit)
    if ($Explicit) {
        if (Test-Path -LiteralPath $Explicit) { return (Resolve-Path -LiteralPath $Explicit).Path }
        throw "找不到指定的 JLinkExe：$Explicit"
    }
    # 注意：D:\java\bin\jlink.exe 是 JDK 的 jlink 工具，不是 SEGGER 的，不能用
    $candidates = @(
        'D:\JLink_V988\JLink.exe',
        'C:\Program Files\SEGGER\JLink\JLink.exe',
        'C:\Program Files (x86)\SEGGER\JLink\JLink.exe',
        'D:\Keil_v5\ARM\Segger\JLink.exe'
    )
    foreach ($c in $candidates) {
        if (Test-Path -LiteralPath $c) { return $c }
    }
    foreach ($drive in @('C:\', 'D:\')) {
        $dirs = Get-ChildItem -LiteralPath $drive -Directory -Filter 'JLink*' -ErrorAction SilentlyContinue
        foreach ($d in $dirs) {
            $p = Join-Path $d.FullName 'JLink.exe'
            if (Test-Path -LiteralPath $p) { return $p }
        }
    }
    throw '没找到 SEGGER JLink.exe，请用 -JLinkExe 手动指定安装路径。'
}

function Get-DevIdName {
    param([UInt32]$DevId)
    switch ($DevId) {
        0x450 { 'STM32H74x/H75x (H743/H750 等)' }
        0x483 { 'STM32H72x/H73x (H723/H725/H730/H733 等)' }
        0x480 { 'STM32H7A3/H7B3' }
        0x484 { 'STM32H7B0' }
        default { '未知 DEV_ID，请核对板子丝印' }
    }
}

$jlink = Find-JLinkExe -Explicit $JLinkExe
Write-Host "J-Link  : $jlink"
Write-Host "目标器件: $Device   接口: $Interface   速度: $Speed kHz"

if ($Build) {
    Write-Host '[1/2] 编译中 ...'
    Push-Location -LiteralPath $root
    try {
        # build 目录被清理过时，先自动 configure（否则 --build 会报 not configured）
        $cache = Join-Path $root 'build\Debug\CMakeCache.txt'
        if (-not (Test-Path -LiteralPath $cache)) {
            Write-Host '      build\Debug 未配置，先运行 cmake --preset Debug ...'
            & cmake --preset Debug
            if ($LASTEXITCODE -ne 0) { throw "configure 失败（exit=$LASTEXITCODE）" }
        }
        & cmake --build --preset Debug
        if ($LASTEXITCODE -ne 0) { throw "编译失败（exit=$LASTEXITCODE）" }
    }
    finally { Pop-Location }
}

if (-not $Elf) { $Elf = Join-Path $root 'build\Debug\ROBOCON27_V.CMAKE.elf' }
if (-not $ProbeOnly) {
    if (-not (Test-Path -LiteralPath $Elf)) {
        throw "找不到 ELF：$Elf`n先编译：cmake --build --preset Debug  或者加 -Build 参数。"
    }
    $Elf = (Resolve-Path -LiteralPath $Elf).Path
    $elfUnix = $Elf.Replace('\', '/')
    Write-Host "ELF     : $Elf ($([math]::Round((Get-Item -LiteralPath $Elf).Length / 1KB)) KB)"
}

function Invoke-JLinkScript {
    param([string[]]$Lines)
    $path = Join-Path $env:TEMP ("robocon_jlink_{0}.jlink" -f $PID)
    Set-Content -LiteralPath $path -Value $Lines -Encoding ASCII
    try {
        & $jlink -device $Device -if $Interface -speed $Speed -autoconnect 1 -CommandFile $path 2>&1 |
            ForEach-Object { "$_" }
    }
    finally {
        Remove-Item -LiteralPath $path -ErrorAction SilentlyContinue
    }
}

# 第一步：先单独验证 SWD 能不能连上（连不上就别往下浪费时间）
$output = Invoke-JLinkScript -Lines @("si $Interface", "speed $Speed", 'connect', 'q')
$output | ForEach-Object { Write-Host $_ }

$errors = @($output | Where-Object {
        $_ -match 'Error occurred|Could not connect to the target|Failed to initialize DAP|Failed to attach|No J-Link device found|Unknown device'
    })

if ($errors.Count -gt 0) {
    Write-Host ''
    Write-Host '================ 连接失败 ================' -ForegroundColor Red
    Write-Host 'SWD 连不到芯片（DAP 无响应），不是代码问题。按顺序检查：' -ForegroundColor Yellow
    Write-Host '  1) 板子是否真的上电：VTref=3.3V 可能是转接板自带 LDO，不代表板子有电'
    Write-Host '  2) 4-pin SWD 座必须按丝印逐针接（3V3/SWDIO/SWCLK/GND 顺序因板而异，DIO 与 CLK 对调也会失败）'
    Write-Host '  3) 万用表判据（只给板子上电、拔掉 J-Link）：SWDIO 应 ~3.3V（内部上拉），SWCLK 应 ~0V（内部下拉）'
    Write-Host '  4) 断电测通断：座 SWDIO <-> MCU PA13、座 SWCLK <-> PA14 是否导通（可能要插跳线帽 / 焊 0R）'
    Write-Host '  5) NRST 接上再试；芯片若被开过 RDP 需 J-Link unlock（会擦掉整片 Flash）'
    Write-Host ''
    Write-Host '详细排查清单：FLASH_JLINK.md' -ForegroundColor Cyan
    exit 1
}

if ($ProbeOnly) {
    # 连接已通过，再单独读一次芯片 ID / Flash 容量
    $out2 = Invoke-JLinkScript -Lines @(
        "si $Interface", "speed $Speed", 'connect', 'halt',
        'mem32 0x5C001000 1',   # DBGMCU_IDCODE
        'mem32 0x1FF1E880 1',   # FLASHSIZE（H7 在 0x1FF1E880，不是 DBGMCU 里）
        'q')
    $out2 | ForEach-Object { Write-Host $_ }

    $idLine = $out2 | Where-Object { $_ -match '5C001000\s*=\s*([0-9A-Fa-f]{8})' } | Select-Object -First 1
    $fsLine = $out2 | Where-Object { $_ -match '1FF1E880\s*=\s*([0-9A-Fa-f]{8})' } | Select-Object -First 1
    Write-Host ''
    Write-Host '================ 探测成功 ================' -ForegroundColor Green
    if ($idLine -and $idLine -match '=\s*([0-9A-Fa-f]{8})') {
        $raw = [Convert]::ToUInt32($matches[1], 16)
        $dev = $raw -band 0xFFF
        Write-Host ("DBGMCU_IDCODE = 0x{0:X8}  DEV_ID = 0x{1:X3}  ->  {2}" -f $raw, $dev, (Get-DevIdName -DevId $dev))
    }
    if ($fsLine -and $fsLine -match '=\s*([0-9A-Fa-f]{8})') {
        $kb = [Convert]::ToUInt32($matches[1], 16) -band 0xFFFF
        Write-Host ("FLASHSIZE     = {0} KB" -f $kb)
    }
    exit 0
}

# 第二步：连接可用，开始烧录
$flashLines = @("si $Interface", "speed $Speed", 'connect', 'r', 'h')
if ($Erase) { $flashLines += 'erase' }
$flashLines += "loadfile `"$elfUnix`""
$flashLines += 'r'
$flashLines += 'g'
$flashLines += 'q'

Write-Host ''
Write-Host '开始烧录 ...'
$out2 = Invoke-JLinkScript -Lines $flashLines
$out2 | ForEach-Object { Write-Host $_ }

$errors2 = @($out2 | Where-Object {
        $_ -match 'Error|failed|Failed|Could not'
    })
$downloaded = @($out2 | Where-Object { $_ -match 'O\.K\.' }).Count -gt 0

if ($downloaded -and $errors2.Count -eq 0) {
    Write-Host ''
    Write-Host '================ 烧录成功 ================' -ForegroundColor Green
    Write-Host '已复位并从 Flash 启动（r / g）。看板子 LED / 串口 / J-Link RTT 是否按预期动作。'
    exit 0
}

Write-Host ''
Write-Host '================ 烧录失败 ================' -ForegroundColor Red
if ($errors2.Count -gt 0) {
    Write-Host '关键错误行：' -ForegroundColor Yellow
    $errors2 | Select-Object -First 6 | ForEach-Object { Write-Host "  $_" }
}
Write-Host ''
Write-Host '常见原因：' -ForegroundColor Yellow
Write-Host '  - Flash 被写保护 / RDP 打开   -> 用「flash: 擦除整片再烧录」或 J-Link unlock'
Write-Host '  - 板子实际是 H743/H750 等型号 -> 换 -Device 并回 CubeMX 重生成'
Write-Host '  - 烧录中掉线/复位            -> 接上 NRST，降低速度 -Speed 1000'
Write-Host '详细排查清单：FLASH_JLINK.md' -ForegroundColor Cyan
exit 1
