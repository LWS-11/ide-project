param(
    [string]$PortName = "COM4",
    [string]$OutputDirectory = ".\snapshot",
    [int]$BaudRate = 115200,
    [switch]$SaveNow
)

$ErrorActionPreference = "Stop"
New-Item -ItemType Directory -Force -Path $OutputDirectory | Out-Null
$timestamp = (Get-Date).ToString("yyyyMMdd_HHmmss_fff")
$baseName = "receiver_snapshot_$timestamp"
$csvPath = Join-Path $OutputDirectory "$baseName.csv"
$bmpPath = Join-Path $OutputDirectory "$baseName.bmp"
$latestCsvPath = Join-Path $OutputDirectory "最新数据.csv"
$latestBmpPath = Join-Path $OutputDirectory "最新图片.bmp"

$port = New-Object System.IO.Ports.SerialPort(
    $PortName,
    $BaudRate,
    [System.IO.Ports.Parity]::None,
    8,
    [System.IO.Ports.StopBits]::One
)
$port.NewLine = "`r`n"
$port.Encoding = [System.Text.Encoding]::ASCII
$port.ReadTimeout = 5000
$port.WriteTimeout = 5000
$port.DtrEnable = $false
$port.RtsEnable = $false

function Wait-ForPrefix {
    param(
        [string]$Prefix,
        [int]$TimeoutMs = 5000
    )

    $deadline = [DateTime]::UtcNow.AddMilliseconds($TimeoutMs)
    while ([DateTime]::UtcNow -lt $deadline) {
        $line = $port.ReadLine()
        if ($line.StartsWith($Prefix)) {
            return $line
        }
    }
    throw "Timed out waiting for $Prefix"
}

try {
    $port.Open()
    Start-Sleep -Milliseconds 300

    if ($SaveNow) {
        $port.WriteLine("SAVE")
        $saveLine = Wait-ForPrefix "SNAPSHOT," 5000
        Write-Host $saveLine
    }

    $port.WriteLine("CSV")
    $csvBegin = Wait-ForPrefix "CSV_BEGIN," 5000
    Write-Host $csvBegin
    $csvLines = New-Object System.Collections.Generic.List[string]
    while ($true) {
        $line = $port.ReadLine()
        if ($line.StartsWith("CSV_END,")) {
            break
        }
        $csvLines.Add($line)
    }
    [System.IO.File]::WriteAllLines(
        $csvPath,
        $csvLines,
        (New-Object System.Text.UTF8Encoding($false))
    )

    $port.WriteLine("BMP")
    $bmpBegin = Wait-ForPrefix "BMP_BEGIN,SIZE=" 5000
    $sizeText = $bmpBegin.Substring("BMP_BEGIN,SIZE=".Length)
    $bmpSize = [int]$sizeText
    $payload = New-Object byte[] $bmpSize
    $offset = 0
    while ($offset -lt $bmpSize) {
        $read = $port.Read($payload, $offset, $bmpSize - $offset)
        if ($read -le 0) {
            throw "Serial read returned no BMP data"
        }
        $offset += $read
    }
    [System.IO.File]::WriteAllBytes($bmpPath, $payload)
    Copy-Item -LiteralPath $csvPath -Destination $latestCsvPath -Force
    Copy-Item -LiteralPath $bmpPath -Destination $latestBmpPath -Force

    $bmpEnd = Wait-ForPrefix "BMP_END" 5000
    Write-Host $bmpEnd
    Write-Host "CSV: $csvPath"
    Write-Host "BMP: $bmpPath"
    Write-Host "最新CSV: $latestCsvPath"
    Write-Host "最新BMP: $latestBmpPath"
}
finally {
    if ($port.IsOpen) {
        $port.Close()
    }
    $port.Dispose()
}
