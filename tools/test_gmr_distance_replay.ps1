$ErrorActionPreference = "Stop"

function Assert-Equal {
    param($Actual, $Expected, [string]$Name)

    if ($Actual -ne $Expected) {
        throw "$Name failed: expected '$Expected', got '$Actual'"
    }
}

function Get-Uint16Bits {
    param([int]$Value)

    if (($Value -lt -32768) -or ($Value -gt 32767)) {
        throw "signed 16-bit delta out of range: $Value"
    }
    if ($Value -lt 0) {
        return [uint32](65536 + $Value)
    }
    return [uint32]$Value
}

function New-PackedDelta {
    param([int]$Left, [int]$Right)

    [uint32]$leftBits = Get-Uint16Bits $Left
    [uint32]$rightBits = Get-Uint16Bits $Right
    return [uint32]($leftBits -bor ($rightBits -shl 16))
}

function ConvertFrom-Int16Bits {
    param([int]$Bits)

    if ($Bits -ge 0x8000) {
        return $Bits - 0x10000
    }
    return $Bits
}

function Get-PositionSpeed {
    param([long]$ErrorCounts)

    $tolerance = 4
    $kpQ1024 = 512
    $minimum = 6
    $maximum = 35
    $magnitude = [Math]::Abs($ErrorCounts)
    if ($magnitude -le $tolerance) {
        return 0
    }
    $speed = [Math]::Floor(($magnitude * $kpQ1024) / 1024)
    $speed = [Math]::Max($minimum, [Math]::Min($maximum, $speed))
    if ($ErrorCounts -lt 0) {
        return -$speed
    }
    return $speed
}

$vectors = @(
    [pscustomobject]@{ Left = 60; Right = 58 },
    [pscustomobject]@{ Left = 62; Right = 64 },
    [pscustomobject]@{ Left = -40; Right = 40 },
    [pscustomobject]@{ Left = -32768; Right = 32767 }
)

foreach ($vector in $vectors) {
    [uint32]$packed = New-PackedDelta $vector.Left $vector.Right
    $left = ConvertFrom-Int16Bits ([int]($packed -band 0xFFFF))
    $right = ConvertFrom-Int16Bits ([int](($packed -shr 16) -band 0xFFFF))
    Assert-Equal $left $vector.Left "left delta round trip"
    Assert-Equal $right $vector.Right "right delta round trip"
}

$targetLeft = 0
$targetRight = 0
foreach ($vector in $vectors[0..2]) {
    $targetLeft += $vector.Left
    $targetRight += $vector.Right
}
Assert-Equal $targetLeft 82 "cumulative left position"
Assert-Equal $targetRight 162 "cumulative right position"

Assert-Equal (Get-PositionSpeed 0) 0 "zero error speed"
Assert-Equal (Get-PositionSpeed 4) 0 "position tolerance"
Assert-Equal (Get-PositionSpeed 5) 6 "minimum positive speed"
Assert-Equal (Get-PositionSpeed 60) 30 "proportional speed"
Assert-Equal (Get-PositionSpeed -60) -30 "signed proportional speed"
Assert-Equal (Get-PositionSpeed 1000) 35 "maximum speed clamp"

$mission = Get-Content -Raw (Join-Path $PSScriptRoot `
    "..\app\gmr_bluetooth_mission.c")
$encoder = Get-Content -Raw (Join-Path $PSScriptRoot `
    "..\hardware\encoder_motor.c")
$config = Get-Content -Raw (Join-Path $PSScriptRoot `
    "..\config\gmr_control_config.h")

foreach ($required in @(
    "GMR_BT_PROTOCOL_VERSION         (2U)",
    "GMR_BT_SIGNAL_ENCODER_DELTA",
    "EncoderMotor_GetTotalCounts",
    "GMR_BLUETOOTH_MISSION_ERROR_REPLAY_TIMEOUT"
)) {
    if (-not $mission.Contains($required)) {
        throw "distance mission source is missing: $required"
    }
}
if ($mission.Contains("EncoderMotor_GetTarget(")) {
    throw "Task 6 must not transmit the count/s target getter"
}
if (-not $encoder.Contains("void EncoderMotor_GetTotalCounts(")) {
    throw "encoder pair snapshot API is missing"
}
foreach ($required in @(
    "GMR_BLUETOOTH_DISTANCE_TOLERANCE_COUNTS (4U)",
    "GMR_BLUETOOTH_DISTANCE_KP_Q1024          (512L)",
    "GMR_BLUETOOTH_DISTANCE_MAX_SPEED_COUNTS_PER_PERIOD (35U)",
    "GMR_BLUETOOTH_DISTANCE_SEGMENT_TIMEOUT_MS (2000U)"
)) {
    if (-not $config.Contains($required)) {
        throw "distance replay config is missing: $required"
    }
}

Write-Host "GMR distance replay vectors OK"
