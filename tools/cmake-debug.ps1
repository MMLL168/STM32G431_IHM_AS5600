param(
    [ValidateSet("configure", "build", "rebuild", "flash")]
    [string]$Action = "build",

    [ValidateSet("Debug", "Release")]
    [string]$Preset = "Debug",

    [string]$ProbeSerial = $(if ($env:STM32_STLINK_SN) { $env:STM32_STLINK_SN } else { "003500383137511233333639" })
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

function Get-FirstExistingPath {
    param(
        [Parameter(Mandatory = $true)]
        [string[]]$Candidates
    )

    foreach ($candidate in $Candidates) {
        if ([string]::IsNullOrWhiteSpace($candidate)) {
            continue
        }

        if (Test-Path -LiteralPath $candidate) {
            return $candidate
        }
    }

    return $null
}

function Add-ToolToPath {
    param(
        [Parameter(Mandatory = $true)]
        [string]$ExecutablePath
    )

    $toolDir = Split-Path -Path $ExecutablePath -Parent
    if (-not ($env:Path.Split(';') -contains $toolDir)) {
        $env:Path = "$toolDir;$env:Path"
    }
}

function Repair-CMakeConfigureOutputs {
    param(
        [Parameter(Mandatory = $true)]
        [string]$BuildDir
    )

    $expectedFiles = @(
        "CMakeCache.txt"
        "build.ninja"
        "cmake_install.cmake"
        "compile_commands.json"
        "CMakeFiles\clean_additional.cmake"
        "CMakeFiles\rules.ninja"
        "CMakeFiles\TargetDirectories.txt"
    )

    $compilerDir = Join-Path -Path $BuildDir -ChildPath "CMakeFiles\4.3.1"
    if (Test-Path -LiteralPath $compilerDir) {
        $expectedFiles += @(
            "CMakeFiles\4.3.1\CMakeASMCompiler.cmake"
            "CMakeFiles\4.3.1\CMakeCCompiler.cmake"
            "CMakeFiles\4.3.1\CMakeCXXCompiler.cmake"
            "CMakeFiles\4.3.1\CMakeSystem.cmake"
        )
    }

    $subdirInstall = Join-Path -Path $BuildDir -ChildPath "cmake\stm32cubemx"
    if (Test-Path -LiteralPath $subdirInstall) {
        $expectedFiles += "cmake\stm32cubemx\cmake_install.cmake"
    }

    foreach ($relativePath in $expectedFiles) {
        $finalPath = Join-Path -Path $BuildDir -ChildPath $relativePath
        if (Test-Path -LiteralPath $finalPath) {
            continue
        }

        $directory = Split-Path -Path $finalPath -Parent
        $name = Split-Path -Path $finalPath -Leaf
        if (-not (Test-Path -LiteralPath $directory)) {
            continue
        }

        $tempCandidate = Get-ChildItem -LiteralPath $directory -Filter "$name.tmp*" -File -ErrorAction SilentlyContinue |
            Sort-Object LastWriteTimeUtc -Descending |
            Select-Object -First 1

        if ($tempCandidate) {
            Copy-Item -LiteralPath $tempCandidate.FullName -Destination $finalPath -Force
        }
    }
}

function Test-CMakeCacheMatchesSource {
    param(
        [Parameter(Mandatory = $true)]
        [string]$BuildDir,

        [Parameter(Mandatory = $true)]
        [string]$SourceDir
    )

    $cachePath = Join-Path -Path $BuildDir -ChildPath 'CMakeCache.txt'
    if (-not (Test-Path -LiteralPath $cachePath)) {
        return $true
    }

    $homeDirectoryLine = Select-String -Path $cachePath -Pattern '^CMAKE_HOME_DIRECTORY:INTERNAL=(.+)$' -ErrorAction SilentlyContinue |
        Select-Object -First 1

    if (-not $homeDirectoryLine) {
        return $true
    }

    $cachedSourceDir = $homeDirectoryLine.Matches[0].Groups[1].Value
    $resolvedCachedSourceDir = [System.IO.Path]::GetFullPath($cachedSourceDir)
    $resolvedSourceDir = [System.IO.Path]::GetFullPath($SourceDir)

    return [string]::Equals($resolvedCachedSourceDir, $resolvedSourceDir, [System.StringComparison]::OrdinalIgnoreCase)
}

$cmakeExe = Get-FirstExistingPath @(
    "C:\Espressif\tools\cmake\3.30.2\bin\cmake.exe"
    "C:\Program Files\CMake\bin\cmake.exe"
    "C:\Program Files\Microsoft Visual Studio\18\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
)

$ninjaExe = Get-FirstExistingPath @(
    "C:\Espressif\tools\ninja\1.12.1\ninja.exe"
    "C:\Program Files\Microsoft Visual Studio\18\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\Ninja\ninja.exe"
    "C:\Program Files\Microsoft Visual Studio\18\Insiders\Common7\IDE\CommonExtensions\Microsoft\CMake\Ninja\ninja.exe"
    "C:\Program Files (x86)\Microsoft Visual Studio\2019\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\Ninja\ninja.exe"
)

$armGccExe = Get-FirstExistingPath @(
    "C:\ST\STM32CubeIDE_1.18.1\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.win32_1.0.100.202602081740\tools\bin\arm-none-eabi-gcc.exe"
    "C:\ST\STM32CubeIDE_1.19.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.win32_1.0.0.202411081344\tools\bin\arm-none-eabi-gcc.exe"
    "C:\Program Files (x86)\Arm GNU Toolchain arm-none-eabi\14.2 rel1\bin\arm-none-eabi-gcc.exe"
    "C:\Program Files (x86)\Arm GNU Toolchain arm-none-eabi\13.3 rel1\bin\arm-none-eabi-gcc.exe"
)

$programmerCli = Get-FirstExistingPath @(
    "C:\ST\STM32CubeIDE_1.18.1\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.400.202601091506\tools\bin\STM32_Programmer_CLI.exe"
    "C:\ST\STM32CubeIDE_1.19.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.200.202503041107\tools\bin\STM32_Programmer_CLI.exe"
    "C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
)

$projectRoot = (Resolve-Path -LiteralPath (Join-Path -Path $PSScriptRoot -ChildPath '..')).Path
$presetBuildDir = Join-Path -Path $projectRoot -ChildPath "build\$Preset"
$needsFreshConfigure = -not (Test-CMakeCacheMatchesSource -BuildDir $presetBuildDir -SourceDir $projectRoot)

if (-not $cmakeExe) {
    throw "cmake.exe not found. Install CMake or update tools/cmake-debug.ps1."
}

if (-not $ninjaExe) {
    throw "ninja.exe not found. Install Ninja/Visual Studio CMake tools or update tools/cmake-debug.ps1."
}

if (-not $armGccExe) {
    throw "arm-none-eabi-gcc.exe not found. Install Arm GNU Toolchain or update tools/cmake-debug.ps1."
}

Add-ToolToPath -ExecutablePath $cmakeExe
Add-ToolToPath -ExecutablePath $ninjaExe
Add-ToolToPath -ExecutablePath $armGccExe

if ($Action -eq "configure") {
    if ($needsFreshConfigure) {
        Write-Host "Detected moved CMake cache in $presetBuildDir. Reconfiguring with --fresh."
        & $cmakeExe --fresh --preset $Preset
    }
    else {
        & $cmakeExe --preset $Preset
    }

    Repair-CMakeConfigureOutputs -BuildDir $presetBuildDir
    exit $LASTEXITCODE
}

if ($Action -eq "rebuild" -or $needsFreshConfigure) {
    if ($needsFreshConfigure -and $Action -ne "rebuild") {
        Write-Host "Detected moved CMake cache in $presetBuildDir. Reconfiguring with --fresh."
    }

    & $cmakeExe --fresh --preset $Preset
    if ($LASTEXITCODE -ne 0) {
        exit $LASTEXITCODE
    }
} else {
    & $cmakeExe --preset $Preset
    if ($LASTEXITCODE -ne 0) {
        exit $LASTEXITCODE
    }
}

Repair-CMakeConfigureOutputs -BuildDir $presetBuildDir

$buildDir = (Resolve-Path -LiteralPath $presetBuildDir).Path

& $ninjaExe -C $buildDir
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}

if ($Action -eq "flash") {
    if (-not $programmerCli) {
        throw "STM32_Programmer_CLI.exe not found. Install STM32CubeProgrammer or update tools/cmake-debug.ps1."
    }

    $elfPath = Join-Path -Path $PSScriptRoot -ChildPath "..\build\$Preset\ST_FOC_G431_IMH.elf"
    $elfPath = (Resolve-Path -LiteralPath $elfPath).Path

    $connectArgs = @(
        "-c"
        "port=SWD"
        "mode=UR"
        "freq=4000"
    )

    if (-not [string]::IsNullOrWhiteSpace($ProbeSerial)) {
        $connectArgs += "sn=$ProbeSerial"
    }
    else {
        Write-Host "No ST-LINK serial specified. Flash will use the first available probe."
    }

    # Fail fast on the selected target before any write operation so we never
    # fall back to the first enumerated ST-LINK when multiple probes are connected.
    & $programmerCli @connectArgs -score
    if ($LASTEXITCODE -ne 0) {
        exit $LASTEXITCODE
    }

    & $programmerCli @connectArgs -w $elfPath -v -rst
    exit $LASTEXITCODE
}
