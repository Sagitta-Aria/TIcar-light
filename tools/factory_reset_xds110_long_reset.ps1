param(
    [string]$ProjectDir = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path,
    [string]$CcsDir = "D:\Ti\ccs",
    [switch]$ConfirmFactoryReset
)

$ErrorActionPreference = "Stop"

if (-not $ConfirmFactoryReset) {
    throw "DSSM Factory Reset changes NONMAIN. Add -ConfirmFactoryReset only after explicit confirmation."
}

$ProjectDir = (Resolve-Path -LiteralPath $ProjectDir).Path
$CcsDir = (Resolve-Path -LiteralPath $CcsDir).Path
$Script = Join-Path $ProjectDir "tools\factory_reset_xds110_long_reset.js"
$Config = Join-Path $ProjectDir "targetConfigs\MSPM0G3507_XDS110_SLOW.ccxml"
$Dss = Join-Path $CcsDir "ccs_base\scripting\bin\dss.bat"

foreach ($path in @($Script, $Config, $Dss)) {
    if (-not (Test-Path -LiteralPath $path -PathType Leaf)) {
        throw "Required file not found: $path"
    }
}

Write-Host "Running long-reset DSSM Factory Reset with slow XDS110 config."
$oldAllow = $env:MSPM0_ALLOW_FACTORY_RESET
$oldConfig = $env:MSPM0_XDS110_CONFIG
$env:MSPM0_ALLOW_FACTORY_RESET = "YES"
$env:MSPM0_XDS110_CONFIG = $Config
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
        throw "Long-reset DSSM Factory Reset failed"
    }
    if (($text -match "Factory Reset aborted") -or
        ($text -match "Factory Reset failed") -or
        ($text -match "Command execution failed") -or
        ($text -notmatch "Command execution completed")) {
        throw "Long-reset DSSM Factory Reset did not report successful completion"
    }
}
finally {
    if ($null -ne $oldActionPreference) {
        $ErrorActionPreference = $oldActionPreference
    }
    if ($null -eq $oldAllow) {
        Remove-Item Env:\MSPM0_ALLOW_FACTORY_RESET -ErrorAction SilentlyContinue
    } else {
        $env:MSPM0_ALLOW_FACTORY_RESET = $oldAllow
    }
    if ($null -eq $oldConfig) {
        Remove-Item Env:\MSPM0_XDS110_CONFIG -ErrorAction SilentlyContinue
    } else {
        $env:MSPM0_XDS110_CONFIG = $oldConfig
    }
}
