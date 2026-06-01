param(
    [string]$Port = "COM20",
    [int]$BaudRate = 115200,
    [string]$Text = "",
    [int]$TimeoutMs = 3000
)

$ErrorActionPreference = "Stop"

function Convert-ToHexText {
    param([byte[]]$Bytes)

    if ($null -eq $Bytes -or $Bytes.Length -eq 0) {
        return ""
    }

    return (($Bytes | ForEach-Object { $_.ToString("X2") }) -join " ")
}

function Read-LoopbackBytes {
    param(
        [System.IO.Ports.SerialPort]$Serial,
        [int]$ExpectedLength,
        [int]$TimeoutMs
    )

    $buffer = New-Object System.Collections.Generic.List[byte]
    $deadline = [DateTime]::UtcNow.AddMilliseconds($TimeoutMs)

    while ([DateTime]::UtcNow -lt $deadline) {
        $count = $Serial.BytesToRead
        if ($count -gt 0) {
            $chunk = New-Object byte[] $count
            $read = $Serial.Read($chunk, 0, $count)
            for ($i = 0; $i -lt $read; $i++) {
                $buffer.Add($chunk[$i])
            }

            if ($buffer.Count -ge $ExpectedLength) {
                break
            }
        }

        Start-Sleep -Milliseconds 10
    }

    return ,([byte[]]$buffer.ToArray())
}

if ([string]::IsNullOrWhiteSpace($Text)) {
    $Text = "WIRELESS_LOOPBACK_TEST_" + (Get-Date -Format "HHmmss")
}

$encoding = [System.Text.Encoding]::ASCII
$txBytes = $encoding.GetBytes($Text)
$serial = [System.IO.Ports.SerialPort]::new($Port, $BaudRate)

$serial.Parity = [System.IO.Ports.Parity]::None
$serial.DataBits = 8
$serial.StopBits = [System.IO.Ports.StopBits]::One
$serial.Handshake = [System.IO.Ports.Handshake]::None
$serial.DtrEnable = $false
$serial.RtsEnable = $false
$serial.ReadTimeout = 50
$serial.WriteTimeout = 1000

try {
    Write-Host "Wireless loopback test"
    Write-Host "Port: $Port, baud: $BaudRate, timeout: ${TimeoutMs}ms"
    Write-Host "请确认无线接收端已供电，并且接收端排针 TX 与 RX 已短接。"
    Write-Host ""

    $serial.Open()
    Start-Sleep -Milliseconds 200
    $serial.DiscardInBuffer()
    $serial.DiscardOutBuffer()

    Write-Host "TX text: $Text"
    Write-Host ("TX hex : " + (Convert-ToHexText $txBytes))
    $serial.Write($Text)

    $rxBytes = Read-LoopbackBytes -Serial $serial `
        -ExpectedLength $txBytes.Length -TimeoutMs $TimeoutMs
    if ($null -eq $rxBytes) {
        $rxBytes = [byte[]]::new(0)
    } else {
        $rxBytes = [byte[]]$rxBytes
    }
    $rxText = $encoding.GetString($rxBytes)

    Write-Host ""
    Write-Host "RX text: $rxText"
    Write-Host ("RX hex : " + (Convert-ToHexText $rxBytes))

    if ($rxText.Contains($Text)) {
        Write-Host ""
        Write-Host "PASS: 收到了自己发出的内容，无线串口透传基本正常。"
        exit 0
    }

    Write-Host ""
    if ($rxBytes.Length -eq 0) {
        Write-Host "FAIL: 超时没有收到任何回环数据。"
        Write-Host "建议检查：COM 口是否被串口助手占用、无线两端是否配对、RX 端是否供电、TX/RX 是否真的短接。"
    } else {
        Write-Host "FAIL: 收到了数据，但不是原样回环。"
        Write-Host "建议检查：波特率、空中速率/信道配置、供电稳定性、是否有其它设备正在发送。"
    }
    exit 1
}
finally {
    if ($serial.IsOpen) {
        $serial.Close()
    }
    $serial.Dispose()
}
