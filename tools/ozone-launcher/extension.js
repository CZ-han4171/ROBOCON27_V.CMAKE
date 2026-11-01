'use strict';

const vscode = require('vscode');
const fs = require('node:fs/promises');
const path = require('node:path');
const { spawn } = require('node:child_process');

async function isFile(file) {
  try { return (await fs.stat(file)).isFile(); } catch { return false; }
}

async function findOzone(configured, platform = process.platform, env = process.env) {
  const explicit = configured || env.OZONE_EXECUTABLE;
  if (explicit) {
    if (await isFile(explicit)) return explicit;
    throw new Error(`找不到 Ozone：${explicit}`);
  }
  const windows = platform === 'win32';
  const names = windows ? ['Ozone.exe'] : ['ozone', 'Ozone'];
  const candidates = (env.PATH || '').split(windows ? ';' : ':')
    .filter(Boolean).flatMap(dir => names.map(name => path.join(dir, name)));
  const roots = windows
    ? [env.ProgramFiles, env['ProgramFiles(x86)']].filter(Boolean).map(dir => path.join(dir, 'SEGGER'))
    : ['/opt/SEGGER'];
  for (const root of roots) {
    try {
      const dirs = (await fs.readdir(root)).filter(name => /^Ozone/i.test(name))
        .sort((a, b) => b.localeCompare(a, undefined, { numeric: true }));
      candidates.push(...dirs.map(dir => path.join(root, dir, windows ? 'Ozone.exe' : 'Ozone')));
    } catch { /* Other installation roots may still be available. */ }
  }
  for (const candidate of candidates) if (await isFile(candidate)) return candidate;
  throw new Error('未找到 Ozone。请安装它，或在设置 ozoneLauncher.executable 中填写完整路径。');
}

async function makeLaunchPlan(folder, config, elf) {
  if (!elf || path.extname(elf).toLowerCase() !== '.elf' || !await isFile(elf)) {
    throw new Error('当前目标没有生成有效的 .elf 文件，请检查 CMake 构建结果。');
  }
  const executable = await findOzone(config.get('executable', ''));
  // An ELF alone makes Ozone open the New Project Wizard with this program file.
  // Passing -project, -device, or -select would skip choices the user makes there.
  return { executable, args: ['-programfile', elf], cwd: folder.uri.fsPath };
}

function activate(context) {
  const output = vscode.window.createOutputChannel('Ozone Launcher');
  // Match the original CMake Tools launch button's position, immediately after Debug.
  const button = vscode.window.createStatusBarItem('ozoneLauncher.launch', vscode.StatusBarAlignment.Left, 3.21);
  button.name = 'Ozone 启动';
  button.text = '$(play)';
  button.tooltip = '构建所选 CMake 目标并打开 Ozone 新建项目向导';
  button.command = 'ozoneLauncher.open';
  let busy = false;
  const folders = () => (vscode.workspace.workspaceFolders || [])
    .filter(folder => vscode.workspace.getConfiguration('ozoneLauncher', folder.uri).get('enabled', false));
  const refresh = () => folders().length ? button.show() : button.hide();
  const command = vscode.commands.registerCommand('ozoneLauncher.open', async () => {
    if (busy) return;
    let folder;
    const enabled = folders();
    if (enabled.length === 1) folder = enabled[0];
    else if (enabled.length > 1) {
      const item = await vscode.window.showQuickPick(enabled.map(value => ({ label: value.name, folder: value })), { placeHolder: '选择要用 Ozone 打开的项目' });
      folder = item && item.folder;
    }
    if (!folder || !vscode.workspace.isTrusted) return;
    busy = true;
    button.text = '$(loading~spin)';
    try {
      const config = vscode.workspace.getConfiguration('ozoneLauncher', folder.uri);
      await findOzone(config.get('executable', ''));
      if (!vscode.workspace.getConfiguration('cmake', folder.uri).get('buildBeforeRun', true)) {
        throw new Error('请启用 cmake.buildBeforeRun，确保打开的是本次构建成功的 ELF。');
      }
      // CMake Tools saves files, configures if needed, builds, and returns null on failure/cancellation.
      const elf = await vscode.commands.executeCommand('cmake.launchTargetPath', folder);
      if (!elf) {
        output.appendLine('构建未成功或目标选择已取消，未启动 Ozone。');
        return;
      }
      const plan = await makeLaunchPlan(folder, config, elf);
      output.appendLine(`打开 ELF：${elf}`);
      output.appendLine(`Ozone：${plan.executable}`);
      output.appendLine('进入 Ozone 新建项目向导，芯片和调试器由用户选择。');
      const child = spawn(plan.executable, plan.args, { cwd: plan.cwd, detached: true, stdio: 'ignore', shell: false });
      await new Promise((resolve, reject) => { child.once('spawn', resolve); child.once('error', reject); });
      child.unref();
    } catch (error) {
      output.appendLine(`错误：${error.message}`);
      output.show(true);
      void vscode.window.showErrorMessage(`Ozone：${error.message}`);
    } finally {
      busy = false;
      button.text = '$(play)';
    }
  });
  context.subscriptions.push(output, button, command,
    vscode.workspace.onDidChangeConfiguration(refresh), vscode.workspace.onDidChangeWorkspaceFolders(refresh));
  refresh();
}

module.exports = { activate, findOzone, makeLaunchPlan };
