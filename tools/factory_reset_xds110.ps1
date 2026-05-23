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
$Script = Join-Path $ProjectDir "tools\factory_reset_xds110.js"
$Dss = Join-Path $CcsDir "ccs_base\scripting\bin\dss.bat"

if (-not (Test-Path -LiteralPath $Script -PathType Leaf)) {
    throw "Factory Reset DSS script not found: $Script"
}
if (-not (Test-Path -LiteralPath $Dss -PathType Leaf)) {
    throw "DSS not found: $Dss"
}

Write-Host "Running DSSM Factory Reset. Do not unplug XDS110 or power during this step."
$oldValue = $env:MSPM0_ALLOW_FACTORY_RESET
$env:MSPM0_ALLOW_FACTORY_RESET = "YES"
try {
    $output = & $Dss $Script 2>&1
    $exitCode = $LASTEXITCODE
    $output | ForEach-Object { Write-Host $_ }
    $text = $output | Out-String
    if ($exitCode -ne 0) {
        throw "DSSM Factory Reset failed"
    }
    if (($text -match "Factory Reset aborted") -or
        ($text -match "Factory Reset failed") -or
        ($text -match "Command execution failed") -or
        ($text -notmatch "Command execution completed")) {
        throw "DSSM Factory Reset did not report successful completion"
    }
}
finally {
    if ($null -eq $oldValue) {
        Remove-Item Env:\MSPM0_ALLOW_FACTORY_RESET -ErrorAction SilentlyContinue
    } else {
        $env:MSPM0_ALLOW_FACTORY_RESET = $oldValue
    }
}
