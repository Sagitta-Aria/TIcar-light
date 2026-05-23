param(
    [string[]]$Ports = @("COM17", "COM18"),
    [int]$BaudRate = 115200,
    [int]$Seconds = 20
)

$ErrorActionPreference = "Stop"

foreach ($portName in $Ports) {
    Write-Host "监听 $portName, $BaudRate baud, $Seconds 秒..."
    $port = New-Object System.IO.Ports.SerialPort $portName, $BaudRate, "None", 8, "One"
    $port.ReadTimeout = 200

    try {
        $port.Open()
        $deadline = (Get-Date).AddSeconds($Seconds)
        while ((Get-Date) -lt $deadline) {
            $text = $port.ReadExisting()
            if ($text.Length -gt 0) {
                Write-Host "[$portName] $text" -NoNewline
            }
            Start-Sleep -Milliseconds 100
        }
    }
    finally {
        if ($port.IsOpen) {
            $port.Close()
        }
    }
}
