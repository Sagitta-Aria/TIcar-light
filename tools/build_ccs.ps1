param(
    [string]$ProjectDir = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path,
    [string]$SdkDir = "D:\Ti\mspm0_sdk_2_10_00_04",
    [string]$CcsDir = "D:\Ti\ccs",
    [string]$BuildDir = "",
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

if ([string]::IsNullOrWhiteSpace($BuildDir)) {
    $BuildDir = Join-Path $ProjectDir "Debug\codex-build"
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
    "-D__USE_SYSCONFIG__"
)
foreach ($includeDir in $includeDirs) {
    $compileArgs += @("-I", $includeDir)
}

$sources = @()
foreach ($dir in $sourceDirs) {
    $sources += Get-ChildItem -LiteralPath $dir -Filter *.c -File
}
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

$output = Join-Path $BuildDir "light-car-ccs1.2.out"
$map = Join-Path $BuildDir "light-car-ccs1.2.map"
$linkInfo = Join-Path $BuildDir "light-car-ccs1.2_linkInfo.xml"

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

Write-Host "Build OK: $output"
