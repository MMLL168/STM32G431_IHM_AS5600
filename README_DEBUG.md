# CMake Debug Flow

## Build

Use the helper script if `cmake`, `ninja`, or `arm-none-eabi-gcc` are not in `PATH`:

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\cmake-debug.ps1 -Action build -Preset Debug
```

On this machine the helper script is also the recommended path because it avoids a local `cmake --build` hang by invoking `ninja` directly after configure.

Note:

The `Debug` configuration uses `-O2 -g3` on purpose. For this STM32G431 motor-control project, `-O0` is too slow for the 30 kHz FOC loop and can trigger `FOC Duration` faults as soon as the motor starts.

To rebuild from a clean CMake cache:

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\cmake-debug.ps1 -Action rebuild -Preset Debug
```

To flash the debug ELF with STM32CubeProgrammer:

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\cmake-debug.ps1 -Action flash -Preset Debug
```

The helper defaults to the verified `NUCLEO-G431RB` ST-LINK serial `003500383137511233333639`, and you can still override it with `-ProbeSerial` or the `STM32_STLINK_SN` environment variable.

## Realtime Monitoring

In `Debug` builds the firmware exposes a global snapshot named `g_debug_monitor`.

Watch this symbol in your debugger to inspect:

- `state`
- `command_state`
- `current_faults`
- `occurred_faults`
- `speed_ref_rpm`
- `speed_avg_rpm`
- `power_w`
- `iq_a`
- `id_a`
- `iq_ref_a`
- `id_ref_a`

The snapshot is refreshed every 50 ms inside the main loop and is intended for non-intrusive live watch during debugging.

The workspace launch configuration now enables Cortex-Debug `liveWatch` directly in `launch.json`, so the `CORTEX LIVE WATCH` panel can sample `g_debug_monitor` without extra manual setup.

## Existing UART Monitoring

The project already has Motor Control Protocol logging enabled on `USART2`, so ST Motor Pilot can still be used for richer runtime monitoring over the existing UART channel.

## VSCode Debug

The workspace now includes `.vscode/launch.json`, `.vscode/tasks.json`, and `.vscode/settings.json` for `Cortex Debug`.

Use the launch configuration `STM32G431RB Debug (Cortex Debug)` to debug the ELF built at `build/Debug/ST_FOC_G431_IMH.elf`.

The default pre-launch task is `CMake: Build Debug`, which calls `tools/cmake-debug.ps1`.

Recommended Live Watch expressions:

- `g_debug_monitor`
- `g_debug_monitor.speed_avg_rpm`
- `g_debug_monitor.speed_ref_rpm`
- `g_debug_monitor.iq_a`
- `g_debug_monitor.id_a`
- `g_debug_monitor.current_faults`
- `g_debug_monitor.occurred_faults`
- `g_debug_monitor.knob_online`
- `g_debug_monitor.knob_raw`
- `g_debug_monitor.knob_angle_deg10`
- `g_debug_monitor.knob_status`
- `g_debug_monitor.knob_magnet_detected`

Important:

`Cortex Debug` in `stlink` mode requires `ST-LINK_gdbserver.exe`.

This workspace is configured to use the `ST-LINK_gdbserver.exe` that ships with the local STM32CubeIDE installation. If VSCode reports that it cannot start the ST-LINK GDB server on another machine, install `STM32CubeIDE` or `STM32CubeCLT` with the ST-LINK GDB Server component and update the workspace paths if needed.

If multiple ST-LINK debuggers are connected, this workspace already pins the intended G431 probe in `launch.json`. You can still override flashing with `STM32_STLINK_SN` when needed.

According to the Cortex-Debug project, Live Watch is configured through a `liveWatch` object in `launch.json`, while the screen refresh rate is controlled by a workspace/user setting. This workspace already sets `cortex-debug.liveWatchRefreshRate`, and `launch.json` now enables `liveWatch` for the STM32G431RB debug configurations.

For this project the verified `NUCLEO-G431RB` debugger is:

- `COM17`
- `ST-LINK SN = 003500383137511233333639`

The flash helper script uses that same ST-LINK serial by default to avoid programming the wrong board when multiple probes are connected.

If the GDB server starts but reports `ST-LINK firmware upgrade required`, update the G431 board's ST-LINK firmware from STM32CubeProgrammer before retrying VSCode debug.

If Motor Pilot shows `FAULT_OVER` with only `FOC Duration` active, rebuild and reflash the optimized `Debug` firmware first. That fault is typically caused by an unoptimized real-time build rather than by UART or ST-LINK configuration.

## AS5600 Pin Ownership

The AS5600 integration is intentionally user-owned instead of CubeMX-owned:

- `PB7` is reserved for `AS5600 SDA`
- `PB8` is reserved for `AS5600 SCL`

Keep both pins unassigned in CubeMX / Motor Control Workbench so future regeneration does not conflict with `Src/as5600_knob.c`.
