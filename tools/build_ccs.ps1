param(
    [string]$ProjectDir = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path,
    [string]$SdkDir = "D:\Ti\mspm0_sdk_2_10_00_04",
    [string]$CcsDir = "D:\Ti\ccs",
    [string]$BuildDir = "",
    [string]$Profile = "",
    [string]$Board = "Tianmeng",
    [string]$BluetoothRole = "Disabled",
    [string]$Jy61 = "",
    [string]$Log = "",
    [string[]]$Defines = @(),
    [switch]$Clean
)

$ErrorActionPreference = "Stop"

function Assert-FileExists {
    param([string]$Path)

    if (-not (Test-Path -LiteralPath $Path -PathType Leaf)) {
        throw "File not found: $Path"
    }
}

function Assert-DirExists {
    param([string]$Path)

    if (-not (Test-Path -LiteralPath $Path -PathType Container)) {
        throw "Directory not found: $Path"
    }
}

function Get-ObjectName {
    param(
        [string]$SourcePath,
        [string]$ResolvedProjectDir,
        [string]$ResolvedSdkDir
    )

    $resolvedSource = (Resolve-Path -LiteralPath $SourcePath).Path
    if ($resolvedSource.StartsWith($ResolvedProjectDir, [System.StringComparison]::OrdinalIgnoreCase)) {
        $name = $resolvedSource.Substring($ResolvedProjectDir.Length).TrimStart("\")
    }
    elseif ($resolvedSource.StartsWith($ResolvedSdkDir, [System.StringComparison]::OrdinalIgnoreCase)) {
        $name = "sdk\" + $resolvedSource.Substring($ResolvedSdkDir.Length).TrimStart("\")
    }
    else {
        $name = $resolvedSource
    }

    return (($name -replace "[:\\/ ]", "_") + ".o")
}

$ProjectDir = (Resolve-Path -LiteralPath $ProjectDir).Path
$SdkDir = (Resolve-Path -LiteralPath $SdkDir).Path
$CcsDir = (Resolve-Path -LiteralPath $CcsDir).Path

$legacyProfile = $null
foreach ($define in $Defines) {
    if ($define -match '^CAR_LIBRARY_GMR_CONFIG_ENABLED=(0|1)(?:U)?$') {
        $legacyProfile = if ($Matches[1] -eq '1') { 'Gmr' } else { 'Full' }
    }
    if ($define -match '^CAR_ACTIVE_PROFILE=') {
        throw "Use -Profile Gmr or -Profile Full instead of defining CAR_ACTIVE_PROFILE"
    }
    if ($define -match '^CAR_BLUETOOTH_ROLE=') {
        throw "Use -BluetoothRole Disabled, Master, or Slave instead of defining CAR_BLUETOOTH_ROLE"
    }
    if ($define -match '^CAR_LIBRARY_BOARD_PROFILE=') {
        throw "Use -Board Tianmeng or -Board Dimeng instead of defining CAR_LIBRARY_BOARD_PROFILE"
    }
    if ($define -match '^CAR_JY61P_ENABLED=') {
        throw "Use -Jy61 Enabled or -Jy61 Disabled instead of defining CAR_JY61P_ENABLED"
    }
    if ($define -match '^CAR_ENABLE_LOG_UART(?:_RX)?=') {
        throw "Use -Log Enabled or -Log Disabled instead of defining CAR_ENABLE_LOG_UART"
    }
}
if ([string]::IsNullOrWhiteSpace($Profile)) {
    $Profile = if ($null -ne $legacyProfile) { $legacyProfile } else { 'Gmr' }
}
if (($Profile -ne 'Gmr') -and ($Profile -ne 'Full')) {
    throw "Profile must be Gmr or Full"
}
if (($null -ne $legacyProfile) -and ($legacyProfile -ne $Profile)) {
    throw "-Profile $Profile conflicts with legacy define selecting $legacyProfile"
}
$profileValue = if ($Profile -eq 'Gmr') { 1 } else { 0 }
$profileSlug = $Profile.ToLowerInvariant()

if (($Board -ne 'Tianmeng') -and ($Board -ne 'Dimeng')) {
    throw "Board must be Tianmeng or Dimeng"
}
$boardValue = if ($Board -eq 'Dimeng') { 1 } else { 2 }
$boardSlug = $Board.ToLowerInvariant()

if (($BluetoothRole -ne 'Disabled') -and ($BluetoothRole -ne 'Master') -and
    ($BluetoothRole -ne 'Slave')) {
    throw "BluetoothRole must be Disabled, Master, or Slave"
}
$bluetoothRoleValue = if ($BluetoothRole -eq 'Master') {
    1
}
elseif ($BluetoothRole -eq 'Slave') {
    2
}
else {
    0
}
$bluetoothRoleSlug = $BluetoothRole.ToLowerInvariant()

if ([string]::IsNullOrWhiteSpace($Jy61)) {
    $Jy61 = if ($Profile -eq 'Gmr') { 'Disabled' } else { 'Enabled' }
}
if (($Jy61 -ne 'Enabled') -and ($Jy61 -ne 'Disabled')) {
    throw "Jy61 must be Enabled or Disabled"
}
$jy61Value = if ($Jy61 -eq 'Enabled') { 1 } else { 0 }

if ([string]::IsNullOrWhiteSpace($Log)) {
    $Log = if ($Profile -eq 'Gmr') { 'Disabled' } else { 'Enabled' }
}
if (($Log -ne 'Enabled') -and ($Log -ne 'Disabled')) {
    throw "Log must be Enabled or Disabled"
}
$logValue = if ($Log -eq 'Enabled') { 1 } else { 0 }

if (($Profile -eq 'Full') -and ($Board -eq 'Dimeng') -and
    ($BluetoothRole -ne 'Disabled')) {
    throw "Full/Dimeng cannot enable Bluetooth because K230 owns UART3 PB2/PB3"
}
if (($Profile -eq 'Gmr') -and ($Board -ne 'Tianmeng')) {
    throw "Competition Gmr profile requires -Board Tianmeng"
}
if (($Profile -eq 'Gmr') -and ($BluetoothRole -ne 'Disabled')) {
    throw "Competition Gmr reserves UART2 for H7; Bluetooth must be disabled"
}

if ([string]::IsNullOrWhiteSpace($BuildDir)) {
    $buildName = "profile-$profileSlug-$boardSlug"
    if ($bluetoothRoleValue -ne 0) {
        $buildName += "-bt-$bluetoothRoleSlug"
    }
    if ($jy61Value -eq 0) {
        $buildName += "-no-jy61"
    }
    if ($logValue -eq 0) {
        $buildName += "-no-log"
    }
    $BuildDir = Join-Path $ProjectDir "Debug\$buildName"
}

$ToolRoot = Join-Path $CcsDir "tools\compiler\ti-cgt-armllvm_4.0.4.LTS"
$Compiler = Join-Path $ToolRoot "bin\tiarmclang.exe"
$SdkStartup = Join-Path $SdkDir "source\ti\devices\msp\m0p\startup_system_files\ticlang\startup_mspm0g350x_ticlang.c"
$ProjectStartup = Join-Path $ProjectDir "generated\startup_mspm0g350x_ticlang.c"
$LinkerCmd = Join-Path $SdkDir "source\ti\devices\msp\m0p\linker_files\ticlang\mspm0g3507.cmd"

Assert-DirExists $ProjectDir
Assert-DirExists $SdkDir
Assert-DirExists $ToolRoot
Assert-FileExists $Compiler
Assert-FileExists $LinkerCmd

$sourceDirs = @("app", "hardware", "system", "generated") | ForEach-Object {
    Join-Path $ProjectDir $_
}

$FreeRtosDir = Join-Path $ProjectDir "Middlewares\FreeRTOS-Kernel"
$FreeRtosPortDir = Join-Path $FreeRtosDir "portable\GCC\ARM_CM0"
Assert-DirExists $FreeRtosDir
Assert-DirExists $FreeRtosPortDir
foreach ($dir in $sourceDirs) {
    Assert-DirExists $dir
}

if ($Clean -and (Test-Path -LiteralPath $BuildDir)) {
    $resolvedBuild = (Resolve-Path -LiteralPath $BuildDir).Path
    if (-not $resolvedBuild.StartsWith($ProjectDir, [System.StringComparison]::OrdinalIgnoreCase)) {
        throw "Refuse to clean outside project: $resolvedBuild"
    }

    Remove-Item -LiteralPath $resolvedBuild -Recurse -Force
}

$ObjDir = Join-Path $BuildDir "obj"
New-Item -ItemType Directory -Force -Path $ObjDir | Out-Null

$includeDirs = @(
    $ProjectDir,
    (Join-Path $ProjectDir "app"),
    (Join-Path $ProjectDir "hardware"),
    (Join-Path $ProjectDir "system"),
    (Join-Path $ProjectDir "config"),
    (Join-Path $ProjectDir "generated"),
    (Join-Path $FreeRtosDir "include"),
    $FreeRtosPortDir,
    (Join-Path $SdkDir "source"),
    (Join-Path $SdkDir "source\third_party\CMSIS\Core\Include")
)

$compileArgs = @(
    "-c",
    "-march=thumbv6m",
    "-mcpu=cortex-m0plus",
    "-mfloat-abi=soft",
    "-mlittle-endian",
    "-mthumb",
    "-O2",
    "-gdwarf-3",
    "-D__MSPM0G3507__",
    "-D__USE_SYSCONFIG__",
    "-DCAR_ACTIVE_PROFILE=$profileValue",
    "-DCAR_LIBRARY_BOARD_PROFILE=$boardValue",
    "-DCAR_BLUETOOTH_ROLE=$bluetoothRoleValue",
    "-DCAR_JY61P_ENABLED=$jy61Value",
    "-DCAR_ENABLE_LOG_UART=$logValue"
)
if ($logValue -eq 0) {
    $compileArgs += "-DCAR_ENABLE_LOG_UART_RX=0"
}
foreach ($define in $Defines) {
    if (-not [string]::IsNullOrWhiteSpace($define)) {
        $compileArgs += "-D$define"
    }
}
foreach ($includeDir in $includeDirs) {
    $compileArgs += @("-I", $includeDir)
}

$sources = @()
foreach ($dir in $sourceDirs) {
    $sources += Get-ChildItem -LiteralPath $dir -Filter *.c -File
}
$gmrOnlySources = @(
    "app\gmr_app.c",
    "app\gmr_menu.c",
    "app\gmr_state_machine.c",
    "hardware\gmr_motor.c"
)
$legacyGmrSources = @(
    "app\gmr_bluetooth_mission.c",
    "app\gmr_tuning_console.c",
    "app\gmr_yaw_control.c"
)
$gmrUnusedSources = @(
    "app\bluetooth_service.c",
    "app\motor_no_yaw.c",
    "hardware\bluetooth_link.c",
    "hardware\bluetooth_protocol.c",
    "hardware\bluetooth_uart.c"
)
$fullOnlySources = @(
    "app\app.c",
    "app\body_motion.c",
    "app\gimbal.c",
    "app\gimbal_attitude.c",
    "app\menu.c",
    "app\state_machine.c",
    "app\staticconfig.c",
    "app\tuning_console.c",
    "app\vision.c",
    "hardware\link.c",
    "hardware\motor.c",
    "hardware\stepper_pulse.c"
)
$excludedSources = if ($Profile -eq 'Gmr') {
    $fullOnlySources + $legacyGmrSources + $gmrUnusedSources
} else {
    $gmrOnlySources + $legacyGmrSources
}
$sources = $sources | Where-Object {
    $relativePath = $_.FullName.Substring($ProjectDir.Length).TrimStart('\')
    $excludedSources -notcontains $relativePath
}
$sources += @(
    (Get-Item -LiteralPath (Join-Path $FreeRtosDir "tasks.c"))
    (Get-Item -LiteralPath (Join-Path $FreeRtosDir "list.c"))
    (Get-Item -LiteralPath (Join-Path $FreeRtosDir "queue.c"))
    (Get-Item -LiteralPath (Join-Path $FreeRtosPortDir "port.c"))
    (Get-Item -LiteralPath (Join-Path $FreeRtosPortDir "portasm.c"))
)
if (-not (Test-Path -LiteralPath $ProjectStartup -PathType Leaf)) {
    Assert-FileExists $SdkStartup
    $sources += Get-Item -LiteralPath $SdkStartup
}
$sources = $sources | Sort-Object FullName

$objects = @()
foreach ($source in $sources) {
    $objectPath = Join-Path $ObjDir (Get-ObjectName -SourcePath $source.FullName -ResolvedProjectDir $ProjectDir -ResolvedSdkDir $SdkDir)
    $objects += $objectPath

    & $Compiler @compileArgs -o $objectPath $source.FullName
    if ($LASTEXITCODE -ne 0) {
        throw "Compile failed: $($source.FullName)"
    }
}

$output = Join-Path $BuildDir "m0-light-rtos.out"
$map = Join-Path $BuildDir "m0-light-rtos.map"
$linkInfo = Join-Path $BuildDir "m0-light-rtos_linkInfo.xml"

$linkArgs = @(
    "-march=thumbv6m",
    "-mcpu=cortex-m0plus",
    "-mfloat-abi=soft",
    "-mlittle-endian",
    "-mthumb",
    "-O2",
    "-gdwarf-3",
    "-Wl,-m$map",
    "-Wl,-i$($SdkDir)\source",
    "-Wl,-i$($ToolRoot)\lib",
    "-Wl,--diag_wrap=off",
    "-Wl,--display_error_number",
    "-Wl,--warn_sections",
    "-Wl,--xml_link_info=$linkInfo",
    "-Wl,--rom_model",
    "-o",
    $output
)
$linkArgs += $objects
$linkArgs += @(
    "-Wl,-l$LinkerCmd",
    "-Wl,-lti/driverlib/lib/ticlang/m0p/mspm0g1x0x_g3x0x/driverlib.a",
    "-Wl,-llibc.a"
)

& $Compiler @linkArgs
if ($LASTEXITCODE -ne 0) {
    throw "Link failed"
}

Write-Host "Build OK [$Profile/$Board/$BluetoothRole/JY61=$Jy61/Log=$Log]: $output"
