# PC 上位机

## 环境

- Windows
- Python 3.10 或更新版本
- Tkinter（通常随 Windows Python 安装）
- pyserial 3.x

本工程已附带 `vendor/pyserial`，直接运行即可。若迁移或删除 vendor 目录，
可重新安装串口依赖：

```powershell
python -m pip install -r requirements.txt
```

上位机使用与队友 `ReceiverPart1` 一致的应用协议，串口固定为 `115200 8N1`。
板间正式通路为 UART7：`PF7/TX`、`PF6/RX`，使用 `1000000 8N1`。

## 运行

双击 `run_host.cmd`，或在 PowerShell 中运行 `.\run_host.ps1`。启动脚本会自动
寻找 Python，并处理当前 Codex 内置 Python 的 Tcl/Tk 路径。

需要启动后自动连接优先选中的 ST-Link 串口时，可运行：

```powershell
.\run_host.cmd --auto-connect
```

也可以使用自己的 Python 环境：

```powershell
python pc_host.py
```

连接开发板 CN14 ST-LINK USB 后：

1. 点击“刷新”，选择 `STMicroelectronics STLink Virtual COM Port` 对应的 COM 口。
2. 点击“连接”，程序会自动发送 10 ms 扫描命令。
3. 使用“10 ms”“5 ms”“1 ms”“500 us”“100 us”按钮切换扫描窗口。
4. 观察 CH1 电位器（ADC3_IN7 / PF9）和
   CH2 麦克风（ADC3_IN0 / PA0）波形、电压、帧率及丢帧统计。

## 协议测试

协议模块不依赖 pyserial，可直接运行：

```powershell
python -m unittest discover -s tests -v
```

串口协议细节见 `PROTOCOL.md`。

