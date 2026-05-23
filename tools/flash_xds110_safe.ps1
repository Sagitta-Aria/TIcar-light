param(
    [string]$ProjectDir = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path,
    [string]$CcsDir = "D:\Ti\ccs",
    [switch]$SkipBuild
)

$ErrorActionPreference = "Stop"

$ProjectDir = (Resolve-Path -LiteralPath $ProjectDir).Path
$CcsDir = (Resolve-Path -LiteralPath $CcsDir).Path
$BuildScript = Join-Path $ProjectDir "tools\build_ccs.ps1"
$Config = Join-Path $ProjectDir "targetConfigs\MSPM0G3507_XDS110.ccxml"
$Output = Join-Path $ProjectDir "Debug\codex-build\light-car-ccs1.2.out"
$Dslite = Join-Path $CcsDir "ccs_base\DebugServer\bin\DSLite.exe"

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
    & $BuildScript -Clean
    if ($LASTEXITCODE -ne 0) {
        throw "Build failed"
    }
}

if (-not (Test-Path -LiteralPath $Output -PathType Leaf)) {
    throw "Output file not found: $Output"
}

Write-Host "准备使用 XDS110 下载 MAIN 程序，不执行 Factory Reset，不写 NONMAIN。"
Write-Host "如果失败并出现 Error -260，请先完全关闭 CCS Theia 或结束当前 Debug Session。"

& $Dslite flash --config="$Config" --run --verbose "$Output"
if ($LASTEXITCODE -ne 0) {
    throw "DSLite flash failed"
}
