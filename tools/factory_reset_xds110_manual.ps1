param(
    [string]$ProjectDir = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path,
    [string]$CcsDir = "D:\Ti\ccs",
    [switch]$ConfirmFactoryReset,
    [switch]$Slow
)

$ErrorActionPreference = "Stop"

if (-not $ConfirmFactoryReset) {
    throw "DSSM Factory Reset changes NONMAIN. Add -ConfirmFactoryReset only after explicit confirmation."
}

$ProjectDir = (Resolve-Path -LiteralPath $ProjectDir).Path
$CcsDir = (Resolve-Path -LiteralPath $CcsDir).Path
$Script = Join-Path $ProjectDir "tools\factory_reset_xds110_manual.js"
$Dss = Join-Path $CcsDir "ccs_base\scripting\bin\dss.bat"

if (-not (Test-Path -LiteralPath $Script -PathType Leaf)) {
    throw "Manual Factory Reset DSS script not found: $Script"
}
if (-not (Test-Path -LiteralPath $Dss -PathType Leaf)) {
    throw "DSS not found: $Dss"
}

Write-Host "Running MANUAL DSSM Factory Reset."
Write-Host "When you see 'Press the reset button...', press and release board RESET once."
$oldValue = $env:MSPM0_ALLOW_FACTORY_RESET
$oldConfig = $env:MSPM0_XDS110_CONFIG
$env:MSPM0_ALLOW_FACTORY_RESET = "YES"
if ($Slow) {
    $slowConfig = Join-Path $ProjectDir "targetConfigs\MSPM0G3507_XDS110_SLOW.ccxml"
    if (-not (Test-Path -LiteralPath $slowConfig -PathType Leaf)) {
        throw "Slow XDS110 target config not found: $slowConfig"
    }
    $env:MSPM0_XDS110_CONFIG = $slowConfig
    Write-Host "Using slow XDS110 target config: $slowConfig"
}
try {
    $oldActionPreference = $ErrorActionPreference
    $ErrorActionPreference = "Continue"
    $output = @()
    & $Dss $Script 2>&1 | ForEach-Object {
        $line = ($_ | Out-String).TrimEnd()
        $output += $line
        Write-Host $line
    }
    $exitCode = $LASTEXITCODE
    $ErrorActionPreference = $oldActionPreference
    $text = $output -join "`n"
    if ($exitCode -ne 0) {
        throw "Manual DSSM Factory Reset failed"
    }
    if (($text -match "Factory Reset aborted") -or
        ($text -match "Factory Reset failed") -or
        ($text -match "Command execution failed") -or
        ($text -notmatch "Command execution completed")) {
        throw "Manual DSSM Factory Reset did not report successful completion"
    }
}
finally {
    if ($null -ne $oldActionPreference) {
        $ErrorActionPreference = $oldActionPreference
    }
    if ($null -eq $oldValue) {
        Remove-Item Env:\MSPM0_ALLOW_FACTORY_RESET -ErrorAction SilentlyContinue
    } else {
        $env:MSPM0_ALLOW_FACTORY_RESET = $oldValue
    }
    if ($null -eq $oldConfig) {
        Remove-Item Env:\MSPM0_XDS110_CONFIG -ErrorAction SilentlyContinue
    } else {
        $env:MSPM0_XDS110_CONFIG = $oldConfig
    }
}
