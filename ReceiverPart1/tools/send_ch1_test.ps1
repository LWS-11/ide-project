param(
    [string]$PortName = "COM3",
    [int]$FrameCount = 30,
    [ValidateSet("CH1", "CH2", "BOTH")]
    [string]$Channel = "BOTH",
    [int]$IntervalUs = 200,
    [switch]$CorruptFirst
)

$count = 500
$frameType = if ($Channel -eq "CH1") { 0x10 } elseif ($Channel -eq "CH2") { 0x20 } else { 0x30 }
$frameFlags = if ($Channel -eq "CH1") { 0x01 } elseif ($Channel -eq "CH2") { 0x02 } else { 0x03 }
$bytesPerSample = if ($Channel -eq "BOTH") { 4 } else { 2 }
$frameLength = 12 + ($bytesPerSample * $count) + 2
$port = [System.IO.Ports.SerialPort]::new(
    $PortName,
    460800,
    [System.IO.Ports.Parity]::None,
    8,
    [System.IO.Ports.StopBits]::One
)
$port.ReadTimeout = 100
$port.WriteTimeout = 1000

function Get-Crc16Ccitt([byte[]]$data, [int]$length) {
    [uint16]$crc = 0xFFFF
    for ($i = 0; $i -lt $length; $i++) {
        $crc = [uint16]($crc -bxor ([uint16]$data[$i] -shl 8))
        for ($bit = 0; $bit -lt 8; $bit++) {
            if (($crc -band 0x8000) -ne 0) {
                $crc = [uint16](($crc -shl 1) -bxor 0x1021)
            }
            else {
                $crc = [uint16]($crc -shl 1)
            }
        }
    }
    return $crc
}

$port.Open()
$port.DiscardInBuffer()
try {
    for ($sequence = 0; $sequence -lt $FrameCount; $sequence++) {
        [byte[]]$frame = New-Object byte[] $frameLength
        $frame[0] = 0xA5
        $frame[1] = 0x5A
        $frame[2] = 0x01
        $frame[3] = [byte]$frameType
        $frame[4] = [byte]($sequence -band 0xFF)
        $frame[5] = [byte](($sequence -shr 8) -band 0xFF)
        $frame[6] = [byte]($count -band 0xFF)
        $frame[7] = [byte](($count -shr 8) -band 0xFF)
        $frame[8] = [byte]($IntervalUs -band 0xFF)
        $frame[9] = [byte](($IntervalUs -shr 8) -band 0xFF)
        $frame[10] = [byte]$frameFlags
        $frame[11] = 0x00

        for ($i = 0; $i -lt $count; $i++) {
            $phase1 = (2.0 * [Math]::PI * $i / $count) - ($sequence * 0.12)
            $phase2 = (4.0 * [Math]::PI * $i / $count) + ($sequence * 0.08)
            $ch1 = [int]([Math]::Round(1500.0 + 1400.0 * [Math]::Sin($phase1)))
            $ch2 = [int]([Math]::Round(1500.0 + 1100.0 * [Math]::Sin($phase2)))
            $offset = 12 + ($bytesPerSample * $i)
            if (($Channel -eq "CH1") -or ($Channel -eq "BOTH")) {
                $frame[$offset] = [byte]($ch1 -band 0xFF)
                $frame[$offset + 1] = [byte](($ch1 -shr 8) -band 0xFF)
                $offset += 2
            }
            if (($Channel -eq "CH2") -or ($Channel -eq "BOTH")) {
                $frame[$offset] = [byte]($ch2 -band 0xFF)
                $frame[$offset + 1] = [byte](($ch2 -shr 8) -band 0xFF)
                $offset += 2
            }
        }

        $crc = Get-Crc16Ccitt $frame (12 + ($bytesPerSample * $count))
        $frame[$frameLength - 2] = [byte]($crc -band 0xFF)
        $frame[$frameLength - 1] = [byte](($crc -shr 8) -band 0xFF)

        if ($CorruptFirst -and ($sequence -eq 0)) {
            $frame[100] = $frame[100] -bxor 0xFF
        }

        $port.Write($frame, 0, $frame.Length)
        Start-Sleep -Milliseconds 100
        try {
            $response = $port.ReadLine().Trim()
            Write-Output "$sequence $response"
        }
        catch {
            Write-Output "$sequence NO_ACK"
        }
    }
}
finally {
    $port.Close()
}
