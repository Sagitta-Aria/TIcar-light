$ErrorActionPreference = "Stop"

function Get-Crc16Modbus {
    param([byte[]]$Data)

    [uint16]$crc = 0xFFFF
    foreach ($byte in $Data) {
        $crc = [uint16]($crc -bxor $byte)
        for ($bit = 0; $bit -lt 8; $bit++) {
            if (($crc -band 1) -ne 0) {
                $crc = [uint16](($crc -shr 1) -bxor 0xA001)
            }
            else {
                $crc = [uint16]($crc -shr 1)
            }
        }
    }
    return $crc
}

function New-TelemetryFrame {
    param(
        [uint16]$Sequence,
        [single]$AngleDeg,
        [single]$GyroDps
    )

    [byte[]]$payload = @(
        [byte]($Sequence -band 0xFF),
        [byte](($Sequence -shr 8) -band 0xFF)
    ) + [BitConverter]::GetBytes($AngleDeg) +
        [BitConverter]::GetBytes($GyroDps)
    [uint16]$crc = Get-Crc16Modbus $payload
    return [byte[]](@(0xAA, 0x55) + $payload + @(
        [byte]($crc -band 0xFF),
        [byte](($crc -shr 8) -band 0xFF),
        0x55,
        0xAA
    ))
}

function Test-TelemetryFrame {
    param([byte[]]$Frame)

    if (($Frame.Count -ne 16) -or ($Frame[0] -ne 0xAA) -or
        ($Frame[1] -ne 0x55) -or ($Frame[14] -ne 0x55) -or
        ($Frame[15] -ne 0xAA)) {
        return $false
    }
    [byte[]]$payload = $Frame[2..11]
    [uint16]$received = [uint16]$Frame[12] -bor
        ([uint16]$Frame[13] -shl 8)
    return (Get-Crc16Modbus $payload) -eq $received
}

function New-Set100HzCommand {
    param([byte]$Sequence)

    [byte[]]$payload = @(0x03, 0x04, $Sequence)
    [uint16]$crc = Get-Crc16Modbus $payload
    return [byte[]](@(0xA5, 0x5A) + $payload + @(
        [byte]($crc -band 0xFF),
        [byte](($crc -shr 8) -band 0xFF),
        0x5A
    ))
}

function Assert-Equal {
    param($Actual, $Expected, [string]$Name)

    if ($Actual -ne $Expected) {
        throw "$Name failed: expected '$Expected', got '$Actual'"
    }
}

[byte[]]$documented = @(
    0xAA, 0x55, 0x01, 0x00, 0x00, 0x00, 0x80, 0x3F,
    0x00, 0x00, 0x00, 0x00, 0x2A, 0x07, 0x55, 0xAA
)
Assert-Equal (Test-TelemetryFrame $documented) $true "Documented frame"

$multiTurn = New-TelemetryFrame 7 720.25 -12.5
Assert-Equal (Test-TelemetryFrame $multiTurn) $true "Multi-turn frame"
Assert-Equal ([BitConverter]::ToSingle($multiTurn, 4)) ([single]720.25) `
    "Continuous angle"

$corrupted = [byte[]]$multiTurn.Clone()
$corrupted[8] = $corrupted[8] -bxor 0x40
Assert-Equal (Test-TelemetryFrame $corrupted) $false "Corrupted CRC"

$command = New-Set100HzCommand 0
$commandHex = ($command | ForEach-Object { $_.ToString("X2") }) -join " "
Assert-Equal $commandHex "A5 5A 03 04 00 83 00 5A" "100 Hz command"

$linkSource = Get-Content -Raw (Join-Path $PSScriptRoot `
    "..\hardware\m0_attitude_link.c")
$uartSource = Get-Content -Raw (Join-Path $PSScriptRoot `
    "..\hardware\m0_attitude_uart.c")
if ($linkSource.Contains("M0_ATTITUDE_LINK_HALF_TURN_X100")) {
    throw "Parser still assumes a +/-180 degree wrapped angle"
}
foreach ($required in @(
    "M0_ATTITUDE_LINK_MAX_ANGLE_X100",
    "M0AttitudeLink_BuildSetReportRate100Hz"
)) {
    if (-not $linkSource.Contains($required)) {
        throw "Attitude link is missing required implementation: $required"
    }
}
foreach ($required in @(
    "M0_ATTITUDE_UART_STARTUP_DELAY_MS",
    "M0AttitudeUart_Task"
)) {
    if (-not $uartSource.Contains($required)) {
        throw "Attitude UART is missing required implementation: $required"
    }
}

Write-Host "M0 attitude protocol vectors OK"
Write-Host "100 Hz command: $commandHex"
