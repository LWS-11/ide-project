param(
    [string]$PortName = "COM3",
    [int]$BaudRate = 460800,
    [int]$DurationSeconds = 12,
    [int]$FrameDelayMs = 100
)

$count = 500
$frameLength = 12 + (4 * $count) + 2
$script:pendingAcks = [System.Collections.Queue]::new()
$script:controlEvents = [System.Collections.Queue]::new()
$script:asciiBytes = [System.Collections.Generic.List[byte]]::new()
$script:controlBytes = [byte[]]::new(9)
$script:controlLength = 0
$script:startupBanner = $null

$port = [System.IO.Ports.SerialPort]::new(
    $PortName,
    $BaudRate,
    [System.IO.Ports.Parity]::None,
    8,
    [System.IO.Ports.StopBits]::One
)
$port.ReadTimeout = 100
$port.WriteTimeout = 1000
$port.DtrEnable = $true
$port.RtsEnable = $false

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

function New-TestFrame([int]$sequence, [int]$intervalUs, [bool]$corrupt) {
    [byte[]]$frame = New-Object byte[] $frameLength
    $frame[0] = 0xA5
    $frame[1] = 0x5A
    $frame[2] = 0x01
    $frame[3] = 0x30
    $frame[4] = [byte]($sequence -band 0xFF)
    $frame[5] = [byte](($sequence -shr 8) -band 0xFF)
    $frame[6] = [byte]($count -band 0xFF)
    $frame[7] = [byte](($count -shr 8) -band 0xFF)
    $frame[8] = [byte]($intervalUs -band 0xFF)
    $frame[9] = [byte](($intervalUs -shr 8) -band 0xFF)
    $frame[10] = 0x03
    $frame[11] = 0x00

    for ($i = 0; $i -lt $count; $i++) {
        $phase1 = (2.0 * [Math]::PI * $i / $count) - ($sequence * 0.12)
        $phase2 = (4.0 * [Math]::PI * $i / $count) + ($sequence * 0.08)
        $ch1 = [int]([Math]::Round(1500.0 + 1200.0 * [Math]::Sin($phase1)))
        $ch2 = [int]([Math]::Round(1500.0 + 1000.0 * [Math]::Sin($phase2)))
        $offset = 12 + (4 * $i)
        $frame[$offset] = [byte]($ch1 -band 0xFF)
        $frame[$offset + 1] = [byte](($ch1 -shr 8) -band 0xFF)
        $frame[$offset + 2] = [byte]($ch2 -band 0xFF)
        $frame[$offset + 3] = [byte](($ch2 -shr 8) -band 0xFF)
    }

    $crc = Get-Crc16Ccitt $frame ($frameLength - 2)
    $frame[$frameLength - 2] = [byte]($crc -band 0xFF)
    $frame[$frameLength - 1] = [byte](($crc -shr 8) -band 0xFF)

    if ($corrupt) {
        $frame[100] = $frame[100] -bxor 0xFF
    }
    return $frame
}

function Complete-ControlPacket {
    if (($script:controlBytes[1] -ne 0x5C) -or
        ($script:controlBytes[2] -ne 0x01) -or
        ($script:controlBytes[3] -ne 0x01)) {
        return
    }

    $receivedCrc = [uint16]$script:controlBytes[7] -bor
        ([uint16]$script:controlBytes[8] -shl 8)
    $calculatedCrc = Get-Crc16Ccitt $script:controlBytes 7
    if ($receivedCrc -ne $calculatedCrc) {
        Write-Host "CTRL CRC ERROR" -ForegroundColor Red
        return
    }

    $event = [pscustomobject]@{
        Index = [int]$script:controlBytes[4]
        IntervalUs = [int]$script:controlBytes[5] -bor
            ([int]$script:controlBytes[6] -shl 8)
    }
    $script:controlEvents.Enqueue($event)
    Write-Host ("CTRL,TB={0},INTERVAL={1}us" -f
        $event.Index, $event.IntervalUs) -ForegroundColor Cyan
}

function Process-ReceivedByte([byte]$value) {
    if ($script:controlLength -gt 0) {
        if (($script:controlLength -eq 1) -and ($value -ne 0x5C)) {
            $script:controlLength = 0
        }
        else {
            $script:controlBytes[$script:controlLength] = $value
            $script:controlLength++
            if ($script:controlLength -eq $script:controlBytes.Length) {
                Complete-ControlPacket
                $script:controlLength = 0
            }
            return
        }
    }

    if ($value -eq 0xC5) {
        $script:controlBytes[0] = $value
        $script:controlLength = 1
        return
    }

    $script:asciiBytes.Add($value)
    if ($value -eq 10) {
        $line = [System.Text.Encoding]::ASCII.GetString(
            $script:asciiBytes.ToArray()).Trim()
        $script:asciiBytes.Clear()
        if ($line.StartsWith("R2 READY")) {
            $script:startupBanner = $line
            Write-Host $line -ForegroundColor Green
        }
        elseif ($line.StartsWith("ACK,")) {
            $fields = $line.Split(',')
            if ($fields.Length -eq 8) {
                $ack = [pscustomobject]@{
                    Status = $fields[1]
                    Sequence = [int]$fields[2]
                    Points = [int]$fields[3]
                    IntervalUs = [int]$fields[4]
                    ValidFrames = [int]$fields[5]
                    CrcErrors = [int]$fields[6]
                    RingDrops = [int]$fields[7]
                }
                $script:pendingAcks.Enqueue($ack)
                Write-Host $line -ForegroundColor DarkGray
            }
        }
        elseif ($line.Length -gt 0) {
            Write-Host $line -ForegroundColor DarkGray
        }
    }
}

function Receive-Serial([int]$timeoutMs) {
    $deadline = (Get-Date).AddMilliseconds($timeoutMs)
    do {
        while ($port.BytesToRead -gt 0) {
            $value = $port.ReadByte()
            if ($value -ge 0) {
                Process-ReceivedByte ([byte]$value)
            }
        }
        if ((Get-Date) -ge $deadline) {
            break
        }
        Start-Sleep -Milliseconds 5
    } while ($true)
}

function Wait-Ack([string]$status, [int]$sequence, [int]$timeoutMs) {
    $deadline = (Get-Date).AddMilliseconds($timeoutMs)
    do {
        Receive-Serial 20
        while ($script:pendingAcks.Count -gt 0) {
            $ack = $script:pendingAcks.Dequeue()
            if (($ack.Sequence -eq $sequence) -and ($ack.Status -eq $status)) {
                return $ack
            }
            Write-Host ("UNEXPECTED ACK: {0},{1}" -f
                $ack.Sequence, $ack.Status) -ForegroundColor Yellow
        }
    } while ((Get-Date) -lt $deadline)
    return $null
}

function Wait-Control([int]$index, [int]$intervalUs, [int]$timeoutMs) {
    $deadline = (Get-Date).AddMilliseconds($timeoutMs)
    do {
        Receive-Serial 20
        while ($script:controlEvents.Count -gt 0) {
            $event = $script:controlEvents.Dequeue()
            if (($event.Index -eq $index) -and
                ($event.IntervalUs -eq $intervalUs)) {
                return $event
            }
        }
    } while ((Get-Date) -lt $deadline)
    return $null
}

function Send-TestFrame([int]$sequence, [int]$intervalUs, [bool]$corrupt) {
    $frame = New-TestFrame $sequence $intervalUs $corrupt
    $port.Write($frame, 0, $frame.Length)
}

try {
    $port.Open()
    $port.DiscardInBuffer()
    Receive-Serial 100

    Write-Host "连接到串口 $PortName, $BaudRate-8-N-1"
    Write-Host "1/6 检查启动/首帧握手..."

    Send-TestFrame 0 200 $false
    $ack = Wait-Ack "OK" 0 1000
    if ($null -eq $ack) {
        throw "未收到首帧 ACK"
    }
    Write-Host "PASS: 正常帧 ACK" -ForegroundColor Green

    Write-Host "2/6 检查 CRC 错误帧不会进入显示..."
    Send-TestFrame 1 200 $true
    $ack = Wait-Ack "CRC" 1 1000
    if ($null -eq $ack) {
        throw "CRC 错误帧没有返回 ACK,CRC"
    }
    Write-Host "PASS: CRC 错帧被拒绝" -ForegroundColor Green

    Write-Host "3/6 检查格式错误帧..."
    [byte[]]$formatFrame = New-TestFrame 2 200 $false
    $formatFrame[10] = 0x01
    $port.Write($formatFrame, 0, $formatFrame.Length)
    $ack = Wait-Ack "FORMAT" 2 1000
    if ($null -eq $ack) {
        throw "格式错误帧没有返回 ACK,FORMAT"
    }
    Write-Host "PASS: 格式错帧被拒绝" -ForegroundColor Green

    Write-Host "4/6 检查 200/100/20/10/2 us 五档数据协议..."
    $sequence = 3
    $seenIntervals = [System.Collections.Generic.HashSet[int]]::new()
    foreach ($interval in 200, 100, 20, 10, 2) {
        Send-TestFrame $sequence $interval $false
        $ack = Wait-Ack "OK" $sequence 1000
        if (($null -eq $ack) -or ($ack.IntervalUs -ne $interval) -or
            ($ack.Points -ne 500)) {
            throw "采样间隔 $interval us 的 ACK 校验失败"
        }
        [void]$seenIntervals.Add($ack.IntervalUs)
        $sequence++
        Start-Sleep -Milliseconds $FrameDelayMs
    }
    if (($seenIntervals.Count -ne 5) -or
        -not $seenIntervals.Contains(200) -or
        -not $seenIntervals.Contains(100) -or
        -not $seenIntervals.Contains(20) -or
        -not $seenIntervals.Contains(10) -or
        -not $seenIntervals.Contains(2)) {
        throw "五档间隔未全部通过"
    }
    Write-Host "PASS: 10ms/5ms/1ms/500us/100us 对应 200/100/20/10/2 us" -ForegroundColor Green

    Write-Host "5/6 检查 PC 到接收端再到 PC 的换档控制闭环..."
    foreach ($test in @(
        [pscustomobject]@{ Index = 0; IntervalUs = 200 },
        [pscustomobject]@{ Index = 1; IntervalUs = 100 },
        [pscustomobject]@{ Index = 2; IntervalUs = 20 },
        [pscustomobject]@{ Index = 3; IntervalUs = 10 },
        [pscustomobject]@{ Index = 4; IntervalUs = 2 })) {
        $command = [System.Text.Encoding]::ASCII.GetBytes(
            ("TB={0}`r`n" -f $test.Index))
        $port.Write($command, 0, $command.Length)
        $event = Wait-Control $test.Index $test.IntervalUs 1000
        if ($null -eq $event) {
            throw ("未收到档位 {0} 的控制包" -f $test.Index)
        }

        Send-TestFrame $sequence $test.IntervalUs $false
        $ack = Wait-Ack "OK" $sequence 1000
        if (($null -eq $ack) -or
            ($ack.IntervalUs -ne $test.IntervalUs)) {
            throw ("档位 {0} 的控制切换未生效" -f $test.Index)
        }
        $sequence++
        Start-Sleep -Milliseconds $FrameDelayMs
    }
    Write-Host "PASS: C5 5C 控制包及采集端换档闭环" -ForegroundColor Green

    Write-Host ("6/6 持续联调 {0} 秒，请按触摸屏按钮或 User 键..." -f
        $DurationSeconds)
    $currentInterval = 200
    $deadline = (Get-Date).AddSeconds($DurationSeconds)
    $controlCount = 0
    $goodCount = 0
    while ((Get-Date) -lt $deadline) {
        Receive-Serial 5
        while ($script:controlEvents.Count -gt 0) {
            $event = $script:controlEvents.Dequeue()
            if (($event.IntervalUs -eq 200) -or
                ($event.IntervalUs -eq 100) -or
                ($event.IntervalUs -eq 20) -or
                ($event.IntervalUs -eq 10) -or
                ($event.IntervalUs -eq 2)) {
                $currentInterval = $event.IntervalUs
                $controlCount++
            }
        }

        Send-TestFrame $sequence $currentInterval $false
        $ack = Wait-Ack "OK" $sequence 1000
        if (($null -ne $ack) -and ($ack.IntervalUs -eq $currentInterval)) {
            $goodCount++
        }
        $sequence++
        Start-Sleep -Milliseconds $FrameDelayMs
    }

    Write-Host ("PASS: 持续联调完成，正常 ACK {0} 帧，控制包 {1} 个" -f
        $goodCount, $controlCount) -ForegroundColor Green
    if ($null -eq $script:startupBanner) {
        Write-Host "提示: 本次会话未捕获冷启动横幅；若需验证 TOUCH=81，请复位板卡后再运行。"
    }
    exit 0
}
catch {
    Write-Host ("FAIL: {0}" -f $_.Exception.Message) -ForegroundColor Red
    exit 1
}
finally {
    if ($port.IsOpen) {
        $port.Close()
    }
}
