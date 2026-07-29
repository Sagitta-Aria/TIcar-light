param(
    [string]$ProjectDir = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path,
    [string]$CcsDir = "D:\Ti\ccs",
    [string]$Profile = "Gmr",
    [string]$Board = "Tianmeng",
    [string]$BluetoothRole = "Disabled",
    [string]$Jy61 = "Enabled",
    [string]$Log = "Enabled",
    [switch]$SkipBuild
)

$ErrorActionPreference = "Stop"

$ProjectDir = (Resolve-Path -LiteralPath $ProjectDir).Path
$CcsDir = (Resolve-Path -LiteralPath $CcsDir).Path
$BuildScript = Join-Path $ProjectDir "tools\build_ccs.ps1"
$Config = Join-Path $ProjectDir "targetConfigs\MSPM0G3507_XDS110.ccxml"
$Dslite = Join-Path $CcsDir "ccs_base\DebugServer\bin\DSLite.exe"

if (($Profile -ne "Gmr") -and ($Profile -ne "Full")) {
    throw "Profile must be Gmr or Full"
}
if (($BluetoothRole -ne "Disabled") -and ($BluetoothRole -ne "Master") -and
    ($BluetoothRole -ne "Slave")) {
    throw "BluetoothRole must be Disabled, Master, or Slave"
}
if (($Board -ne "Tianmeng") -and ($Board -ne "Dimeng")) {
    throw "Board must be Tianmeng or Dimeng"
}
if (($Jy61 -ne "Enabled") -and ($Jy61 -ne "Disabled")) {
    throw "Jy61 must be Enabled or Disabled"
}
if (($Log -ne "Enabled") -and ($Log -ne "Disabled")) {
    throw "Log must be Enabled or Disabled"
}
if (($Profile -eq "Full") -and ($Board -eq "Dimeng") -and
    ($BluetoothRole -ne "Disabled")) {
    throw "Full/Dimeng cannot enable Bluetooth because K230 owns UART3 PB2/PB3"
}
$profileSlug = $Profile.ToLowerInvariant()
$boardSlug = $Board.ToLowerInvariant()
$buildName = "profile-$profileSlug-$boardSlug"
if ($BluetoothRole -ne "Disabled") {
    $buildName += "-bt-$($BluetoothRole.ToLowerInvariant())"
}
if ($Jy61 -eq "Disabled") {
    $buildName += "-no-jy61"
}
if ($Log -eq "Disabled") {
    $buildName += "-no-log"
}
$Output = Join-Path $ProjectDir "Debug\$buildName\m0-light-rtos.out"

if (-not (Test-Path -LiteralPath $BuildScript -PathType Leaf)) {
    throw "Build script not found: $BuildScript"
}
if (-not (Test-Path -LiteralPath $Config -PathType Leaf)) {
    throw "XDS110 target config not found: $Config"
}
if (-not (Test-Path -LiteralPath $Dslite -PathType Leaf)) {
    throw "DSLite not found: $Dslite"
}

if (-not $SkipBuild) {
    & $BuildScript -Profile $Profile -Board $Board `
        -BluetoothRole $BluetoothRole -Jy61 $Jy61 -Log $Log -Clean
    if ($LASTEXITCODE -ne 0) {
        throw "Build failed"
    }
}

if (-not (Test-Path -LiteralPath $Output -PathType Leaf)) {
    throw "Output file not found: $Output"
}

Write-Host "准备使用 XDS110 下载 $Profile/$Board/$BluetoothRole/JY61=$Jy61/Log=$Log MAIN 程序，不执行 Factory Reset，不写 NONMAIN。"
Write-Host "如果失败并出现 Error -260，请先完全关闭 CCS Theia 或结束当前 Debug Session。"

& $Dslite flash --config="$Config" --run --verbose "$Output"
if ($LASTEXITCODE -ne 0) {
    throw "DSLite flash failed"
}
