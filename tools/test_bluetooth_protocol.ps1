$ErrorActionPreference = "Stop"

function Get-Crc16Ccitt {
    param([byte[]]$Data)

    [uint16]$crc = 0xFFFF
    foreach ($byte in $Data) {
        $crc = [uint16]($crc -bxor ([uint16]$byte -shl 8))
        for ($bit = 0; $bit -lt 8; $bit++) {
            if (($crc -band 0x8000) -ne 0) {
                $crc = [uint16]((([uint32]$crc -shl 1) -bxor 0x1021) -band
                    0xFFFF)
            }
            else {
                $crc = [uint16](([uint32]$crc -shl 1) -band 0xFFFF)
            }
        }
    }
    return $crc
}

function New-BluetoothFrame {
    param(
        [byte]$Type,
        [byte]$Sequence,
        [byte]$Source,
        [byte]$Target,
        [byte[]]$Payload
    )

    if ($Payload.Count -gt 16) {
        throw "Payload exceeds protocol limit"
    }
    [byte[]]$body = @(
        0x01,
        $Type,
        $Sequence,
        $Source,
        $Target,
        [byte]$Payload.Count
    ) + $Payload
    [uint16]$crc = Get-Crc16Ccitt $body
    return [byte[]](@(0xA5, 0x5A) + $body + @(
        [byte]($crc -band 0xFF),
        [byte](($crc -shr 8) -band 0xFF)
    ))
}

function Test-BluetoothFrame {
    param([byte[]]$Frame)

    if (($Frame.Count -lt 10) -or ($Frame[0] -ne 0xA5) -or
        ($Frame[1] -ne 0x5A) -or ($Frame[2] -ne 0x01)) {
        return $false
    }
    $payloadLength = $Frame[7]
    if (($payloadLength -gt 16) -or
        ($Frame.Count -ne (10 + $payloadLength))) {
        return $false
    }
    [byte[]]$body = $Frame[2..(7 + $payloadLength)]
    [uint16]$expected = [uint16]$Frame[8 + $payloadLength]
    $expected = [uint16]($expected -bor
        ([uint16]$Frame[9 + $payloadLength] -shl 8))
    return (Get-Crc16Ccitt $body) -eq $expected
}

function Assert-Equal {
    param($Actual, $Expected, [string]$Name)

    if ($Actual -ne $Expected) {
        throw "$Name failed: expected '$Expected', got '$Actual'"
    }
}

[byte[]]$standardVector = [Text.Encoding]::ASCII.GetBytes("123456789")
Assert-Equal (Get-Crc16Ccitt $standardVector) 0x29B1 "CRC standard vector"

[byte[]]$masterHelloPayload = @(
    0x01,
    0x00, 0x25, 0x06, 0x01, 0x0D, 0xD3,
    0x98, 0xDA, 0x50, 0x03, 0x01, 0x77
)
[byte[]]$slaveHelloPayload = @(
    0x02,
    0x98, 0xDA, 0x50, 0x03, 0x01, 0x77,
    0x00, 0x25, 0x06, 0x01, 0x0D, 0xD3
)
[byte[]]$signalPayload = @(
    0x34, 0x12,
    0xC0, 0x1D, 0xFE, 0xFF,
    0x05
)

$masterHello = New-BluetoothFrame 0x01 0x00 0x01 0x02 $masterHelloPayload
$slaveHello = New-BluetoothFrame 0x01 0x00 0x02 0x01 $slaveHelloPayload
$signal = New-BluetoothFrame 0x03 0x2A 0x01 0x02 $signalPayload

Assert-Equal (Test-BluetoothFrame $masterHello) $true "Master HELLO"
Assert-Equal (Test-BluetoothFrame $slaveHello) $true "Slave HELLO"
Assert-Equal (Test-BluetoothFrame $signal) $true "SIGNAL"
Assert-Equal ($masterHello.Count) 23 "HELLO frame length"
Assert-Equal ($signal.Count) 17 "SIGNAL frame length"

$corrupted = [byte[]]$signal.Clone()
$corrupted[10] = $corrupted[10] -bxor 0x80
Assert-Equal (Test-BluetoothFrame $corrupted) $false "Corrupted CRC rejection"

$config = Get-Content -Raw (Join-Path $PSScriptRoot "..\config\bluetooth_config.h")
foreach ($required in @("002506010DD3", "98DA50030177", "98da,50,030177")) {
    if (-not $config.Contains($required)) {
        throw "Bluetooth config is missing verified identity: $required"
    }
}

Write-Host "Bluetooth protocol vectors OK"
Write-Host ("Master HELLO: " + (($masterHello | ForEach-Object { $_.ToString("X2") }) -join " "))
Write-Host ("Slave HELLO:  " + (($slaveHello | ForEach-Object { $_.ToString("X2") }) -join " "))
Write-Host ("SIGNAL:       " + (($signal | ForEach-Object { $_.ToString("X2") }) -join " "))
