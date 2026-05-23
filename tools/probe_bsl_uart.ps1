param(
    [string]$Port = "COM17",
    [string]$CcsDir = "D:\Ti\ccs",
    [ValidateSet("None", "LaunchPad", "Standalone")]
    [string]$InvokeMode = "None"
)

$ErrorActionPreference = "Stop"

function Get-BslCrc32([byte[]]$Data) {
    [uint32]$crc = [uint32]::MaxValue
    [uint32]$poly = 3988292384
    foreach ($b in $Data) {
        $crc = $crc -bxor [uint32]$b
        for ($i = 0; $i -lt 8; $i++) {
            if (($crc -band 1) -ne 0) {
                $crc = ($crc -shr 1) -bxor $poly
            } else {
                $crc = $crc -shr 1
            }
        }
    }
    return $crc
}

function New-BslPacket([byte]$Cmd, [byte[]]$Payload = @()) {
    [byte[]]$body = @($Cmd) + $Payload
    [uint16]$len = $body.Length
    [uint32]$crc = Get-BslCrc32 $body
    [byte[]]$packet = @(0x80, [byte]($len -band 0xFF), [byte](($len -shr 8) -band 0xFF))
    $packet += $body
    $packet += [BitConverter]::GetBytes($crc)
    return $packet
}

function ConvertTo-Hex([byte[]]$Data) {
    if ($Data.Length -eq 0) {
        return "<none>"
    }
    return (($Data | ForEach-Object { $_.ToString("X2") }) -join " ")
}

function Read-SerialBytes($SerialPort, [int]$TimeoutMs) {
    $deadline = [DateTime]::UtcNow.AddMilliseconds($TimeoutMs)
    $bytes = New-Object System.Collections.Generic.List[byte]
    while ([DateTime]::UtcNow -lt $deadline) {
        while ($SerialPort.BytesToRead -gt 0) {
            $bytes.Add([byte]$SerialPort.ReadByte())
        }
        Start-Sleep -Milliseconds 20
    }
    return $bytes.ToArray()
}

function Invoke-BslEntry {
    param(
        [string]$Mode,
        [string]$CcsRoot
    )

    if ($Mode -eq "None") {
        return
    }

    $dbgJtag = Join-Path $CcsRoot "ccs_base\common\uscif\dbgjtag.exe"
    $xdsReset = Join-Path $CcsRoot "ccs_base\common\uscif\xds110\xds110reset.exe"

    if (-not (Test-Path -LiteralPath $dbgJtag -PathType Leaf)) {
        throw "dbgjtag not found: $dbgJtag"
    }
    if (-not (Test-Path -LiteralPath $xdsReset -PathType Leaf)) {
        throw "xds110reset not found: $xdsReset"
    }

    if ($Mode -eq "LaunchPad") {
        & $dbgJtag -f "@xds110" -Y "gpiopins,config=0x1,write=0x1" | Out-Host
        & $xdsReset -d 1400 | Out-Host
        return
    }

    & $dbgJtag -f "@xds110" -Y "gpiopins,config=0x3,write=0x02" | Out-Host
    Start-Sleep -Milliseconds 1400
    & $dbgJtag -f "@xds110" -Y "gpiopins,config=0x3,write=0x03" | Out-Host
}

function Clear-BslEntry {
    param(
        [string]$Mode,
        [string]$CcsRoot
    )

    if ($Mode -eq "None") {
        return
    }

    $dbgJtag = Join-Path $CcsRoot "ccs_base\common\uscif\dbgjtag.exe"
    if ($Mode -eq "LaunchPad") {
        & $dbgJtag -f "@xds110" -Y "gpiopins,config=0x1,write=0x0" | Out-Host
        return
    }
    & $dbgJtag -f "@xds110" -Y "gpiopins,config=0x3,write=0x01" | Out-Host
}

$CcsDir = (Resolve-Path -LiteralPath $CcsDir).Path
Invoke-BslEntry -Mode $InvokeMode -CcsRoot $CcsDir

$serial = [System.IO.Ports.SerialPort]::new($Port, 9600, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
$serial.ReadTimeout = 1000
$serial.WriteTimeout = 2000

try {
    $serial.Open()
    $serial.DiscardInBuffer()
    $serial.DiscardOutBuffer()

    $connect = New-BslPacket 0x12
    Write-Host "Send BSL connection: $(ConvertTo-Hex $connect)"
    $serial.Write($connect, 0, $connect.Length)
    $connectResponse = Read-SerialBytes $serial 1200
    Write-Host "Connection response: $(ConvertTo-Hex $connectResponse)"

    $getId = New-BslPacket 0x19
    Write-Host "Send BSL GET_ID: $(ConvertTo-Hex $getId)"
    $serial.Write($getId, 0, $getId.Length)
    $idResponse = Read-SerialBytes $serial 2000
    Write-Host "GET_ID response: $(ConvertTo-Hex $idResponse)"

    $connectHex = ConvertTo-Hex $connectResponse
    $idHex = ConvertTo-Hex $idResponse
    if (($connectHex -eq (ConvertTo-Hex $connect)) -or ($idHex -match "^32 49")) {
        throw "BSL probe saw UART echo/loopback instead of a valid MSPM0 BSL response."
    }
    if (($connectResponse.Length -eq 0) -and ($idResponse.Length -eq 0)) {
        throw "BSL probe timed out. The target is not in BSL mode or UART wiring is not connected."
    }
    if ($idResponse.Length -lt 9) {
        throw "BSL probe did not receive a complete GET_ID response. Check PA18 BSL invoke and UART TX/RX wiring."
    }
}
finally {
    if ($serial.IsOpen) {
        $serial.Close()
    }
    Clear-BslEntry -Mode $InvokeMode -CcsRoot $CcsDir
}
