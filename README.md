# STM32CubeIDE Projects

This repository contains two related STM32F746 projects and their PC-side
test tools.

## Projects

- `ReceiverPart1` - display-side firmware and UART integration test utilities.
  See `ReceiverPart1/README.md` for the frame format, UART configuration, and
  test procedure.
- `TEST01` - acquisition/test firmware and the Python PC host application.
  See `TEST01/PC_HOST/README.md` and `TEST01/PC_HOST/PROTOCOL.md` for the host
  application and protocol documentation.

## Open in STM32CubeIDE

1. Open STM32CubeIDE.
2. Select **File > Import > General > Existing Projects into Workspace**.
3. Select the project directory, for example
   `stm32-cubeide-projects/ReceiverPart1` or
   `stm32-cubeide-projects/TEST01`.
4. Build the imported project with its existing Debug configuration.

Generated build directories, Python virtual environments, runtime files, and
compiled firmware artifacts are intentionally excluded from version control.
They will be recreated by STM32CubeIDE or the provided host scripts.

## PC host

The reusable `pyserial` package is vendored under
`TEST01/PC_HOST/vendor`, so the PC host can run without installing additional
Python packages. Alternatively:

```powershell
cd TEST01\PC_HOST
python -m pip install -r requirements.txt
python pc_host.py
```
