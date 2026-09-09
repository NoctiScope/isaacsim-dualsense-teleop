# Isaac Sim DualSense Teleop

English | [简体中文](README_zh.md)

Control JetBot and UR10 in NVIDIA Isaac Sim with a Sony PlayStation 5 DualSense controller.

Isaac Sim's OmniGraph `Read Gamepad State` workflow is primarily oriented around Xbox-style controller mappings. This project reads a DualSense through `pygame` and feeds its input into an already running Isaac Sim scene without modifying the official robot assets.

![Isaac Sim DualSense Teleop Demo](media/example_isaacsim_dualsense_teleop.gif)

> **Tested environment, not a broad support claim**
>
> - Windows
> - Isaac Sim 6.0.1
> - pygame 2.6.1
> - Controller name: `DualSense Wireless Controller`
>
> Linux, other Isaac Sim versions, other controllers, and every wired/wireless connection mode have not been tested. The UR10 example is experimental.

## Features

- `examples/jetbot_teleop.py`: differential-drive JetBot teleoperation and the stable introductory example.
- `examples/ur10_teleop.py`: Cartesian TCP control for UR10, with singularity scaling, soft joint-limit protection, and target leashes; currently **experimental**.
- `input/dualsense_input.py`: reusable pygame-based DualSense input with deadzones, normalized triggers, and button-edge detection.
- `config/dualsense_mapping.json`: controller axis and button index mapping.

## How the execution environment works

There are two related but different Python entry points:

- `<ISAAC_SIM_DIR>\python.bat` is Isaac Sim's bundled Python launcher. Use it to install and verify dependencies, or to run standalone Isaac Sim programs.
- **Isaac Sim VS Code Edition > Run** sends the current editor file to the Python Server inside an already running Isaac Sim application. The JetBot and UR10 examples in this repository require this interactive execution path because they use the currently open Stage and timeline.

The `.vscode/settings.json` shipped with Isaac Sim configures VS Code analysis and its interpreter. It does not install packages and does not replace the Python Server connection.

## Installation and connection

### 1. Prerequisites

Install Isaac Sim 6.0.1 and verify that it starts correctly. This repository does not include Isaac Sim or third-party JetBot/UR10 models, meshes, materials, or textures.

Clone this repository, then edit `PROJECT_ROOT` near the top of both `examples/jetbot_teleop.py` and `examples/ur10_teleop.py`:

```python
# Replace this with the directory where you cloned this repository.
PROJECT_ROOT = Path(r"D:\isaacsim_dualsense_teleop")
```

The path is intentionally explicit because Isaac Sim's Python Server executes submitted source as `"<string>"`; the original file path may not be available through `__file__`. If the configured directory is wrong, the script reports which `PROJECT_ROOT` needs to be changed.

### 2. Install pygame into Isaac Sim's Python

Installing `pygame` only into system Python or an ordinary virtual environment is not sufficient. In PowerShell, replace both paths below with your own locations:

```powershell
& "C:\path\to\isaacsim\python.bat" -m pip install -r "D:\path\to\isaacsim-dualsense-teleop\requirements.txt"
```

Alternatively, install only the tested version:

```powershell
& "C:\path\to\isaacsim\python.bat" -m pip install pygame==2.6.1
```

Verify it:

```powershell
& "C:\path\to\isaacsim\python.bat" -c "import pygame; print(pygame.version.ver)"
```

Restart Isaac Sim after installation. `ModuleNotFoundError: pygame` usually means the package was installed into a different Python environment.

### 3. Connect VS Code to Isaac Sim

These examples are interactive scripts sent to an **already running Isaac Sim** instance. They are not ordinary standalone Python programs.

1. In Isaac Sim's Extension Manager, find and enable `isaacsim.code_editor.vscode`. It also enables the Python Server dependency.
2. Install NVIDIA's **Isaac Sim VS Code Edition** extension in VS Code.
3. In Isaac Sim, choose **Window > VS Code**. This opens your own Isaac Sim installation folder and uses the `.vscode` configuration shipped with that installation.
4. Open a teleop file with **File > Open File...**. Adding this repository to the workspace is optional.

The `.vscode` directory in an Isaac Sim installation normally contains:

- `settings.json`: analysis paths for Isaac Sim/Omniverse modules and the bundled Python interpreter.
- `launch.json`: standalone and attach-debug configurations supplied by Isaac Sim.
- `tasks.json`: helper tasks shipped with that Isaac Sim installation.

Use the settings supplied by **your own Isaac Sim installation**; this repository deliberately does not provide a settings template. Do not commit machine-specific settings containing installation paths or user names.

### 4. Optional repository task

If you open this repository as a VS Code workspace, `.vscode/tasks.json` provides **Run with Isaac Sim Python**. It defaults to `C:\isaacsim\python.bat`; change the `command` in that file if Isaac Sim is installed elsewhere. Its working directory is `${workspaceFolder}`.

This task runs the current file as a standalone Python process. It is retained as a convenience for other scripts, but it is **not** the launch method for the two teleop examples. The official documentation states that standalone `Python: Current File` execution should not be used for extension/interactive code. Run `jetbot_teleop.py` and `ur10_teleop.py` through **Run** in the Isaac Sim extension panel.

Reference: [Isaac Sim 6.0.1 — Visual Studio Code](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/development_tools/vscode.html)

## Run the examples

Connect the DualSense before starting and close any other application that may take exclusive control of it.

### JetBot

1. Open `examples/jetbot_scene.usd` in Isaac Sim.
2. Wait for the official JetBot asset to load and confirm that `/World/jetbot` exists in the Stage.
3. Click **Play** in Isaac Sim.
4. Open `examples/jetbot_teleop.py` in VS Code without selecting only part of the file.
5. Open the Isaac Sim panel in the VS Code Activity Bar and click **Run**.
6. Confirm that the Isaac Sim VS Code Edition output panel shows `DualSense JetBot Teleop ACTIVE`.

| DualSense input | JetBot action |
| --- | --- |
| Left stick up / down | Forward / reverse |
| Left stick left / right | Turn left / right |
| Cross (×) | Emergency stop |
| Triangle (△) | Re-enable after an emergency stop |

### UR10 (experimental)

1. Open `examples/ur10_scene.usd` in Isaac Sim.
2. Wait for the official UR10 asset to load and confirm that `/World/ur10` and `/World/ur10/ee_link/tcp` exist in the Stage.
3. Click **Play** in Isaac Sim.
4. Open the complete `examples/ur10_teleop.py` file in VS Code, then click **Run** in the Isaac Sim panel.
5. Confirm that the output panel shows `DualSense UR10 TCP Teleop ACTIVE`.

| Mode | DualSense input | UR10 action |
| --- | --- | --- |
| Default | Left stick left / right | TCP world X− / X+ |
| Default | Left stick up / down | TCP world Y+ / Y− |
| Default | Right stick up / down | TCP world Z+ / Z− |
| Default | Right stick left / right | Roll around world X |
| Hold R2 | Left stick left / right | Pitch around world Y |
| Hold R2 | Left stick up / down | Yaw around world Z |
| Any | Cross (×) | Emergency stop and hold the current joint target |
| Any | Triangle (△) | Resynchronize the current pose and re-enable |

While R2 is held, the left stick changes from XY translation to Pitch/Yaw and translation commands pause. When the Isaac Sim timeline stops, the example attempts to restore the UR10 to the scene's `pose_1`. Joint-limit and singularity protection are not substitutes for complete motion planning or collision avoidance.

### Switching examples

Stop the timeline and open the other scene, then repeat **Play → Run**. Each script cleans up callbacks from a previously running JetBot or UR10 example, but the examples are not intended to control both scenes simultaneously.

## Scenes and assets

The two small USD files in this repository are scene configuration layers only. They load JetBot and UR10 from the Isaac Sim 6.0 asset server through remote `payload` references; robot meshes, materials, and textures are not included or redistributed. Opening a scene for the first time requires network access. For offline use, configure the official Isaac Sim asset packs according to NVIDIA's documentation instead of committing third-party assets here.

References: [Isaac Sim Robot Assets](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/assets/usd_assets_robots.html) · [Isaac Sim License FAQ](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/common/license-faq.html)

## Controller mapping and adjustment

The current mapping comes from the tested Windows environment above. SDL/pygame may report different axis and button indices with another operating system, driver, or connection mode. If controls do not match, edit `config/dualsense_mapping.json` instead of the robot control logic, then rerun the active teleop script.

Current low-level indices:

| Name | Type | pygame index |
| --- | --- | ---: |
| `left_x`, `left_y` | axis | 0, 1 |
| `right_x`, `right_y` | axis | 2, 3 |
| `l2`, `r2` | axis | 4, 5 |
| `cross`, `circle`, `square`, `triangle` | button | 0, 1, 2, 3 |
| `l1`, `r1` | button | 9, 10 |
| D-pad up, down, left, right | button | 11, 12, 13, 14 |

## Troubleshooting

- **`No gamepad detected.`** Connect the controller before running the script. Confirm that Windows Game Controllers and pygame can see it.
- **`ModuleNotFoundError: pygame`** Reinstall with `python.bat` from the Isaac Sim installation, not system `python`.
- **`dualsense_input.py` or the mapping JSON cannot be found** Correct `PROJECT_ROOT` near the top of both teleop files and keep the repository layout intact.
- **`/World/jetbot`, `/World/ur10`, or the TCP cannot be found** Open the matching repository scene and wait for its remote asset to load before clicking Play.
- **The robot model does not load** Check network access and the Isaac Sim asset service. This repository does not provide a copy of the official asset.
- **VS Code Run does nothing** Enable the extension on both the Isaac Sim and VS Code sides, and launch the workspace from Isaac Sim through **Window > VS Code**.
- **Mapping/API errors on Linux or another release** Only Windows with Isaac Sim 6.0.1 has been tested. When opening an issue, include the complete version, controller name, and pygame axis/button output.

## Safety and disclaimer

This is an unofficial community tool and is not affiliated with or endorsed by NVIDIA, Sony Interactive Entertainment, or Universal Robots. All trademarks belong to their respective owners.

This project is an Isaac Sim teleoperation demo only and is not warranted for real-robot control. Do not connect it to physical hardware without an independent safety assessment, hardware emergency stop, collision checking, and workspace guarding. The software is provided “as is” under the MIT License, without warranty.

## License

Original code in this repository is available under the [MIT License](LICENSE). Isaac Sim and remotely loaded robot assets remain under their respective licenses and are outside this repository's MIT grant.
