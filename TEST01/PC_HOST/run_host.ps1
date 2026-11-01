param(
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]]$AppArgs
)

$ErrorActionPreference = "Stop"
$HostDir = Split-Path -Parent $MyInvocation.MyCommand.Path

$PythonExe = Get-ChildItem -Path "$env:USERPROFILE\.cache\codex-runtimes\*\dependencies\python\python.exe" `
    -File -ErrorAction SilentlyContinue |
    Select-Object -First 1 -ExpandProperty FullName

if ([string]::IsNullOrWhiteSpace($PythonExe)) {
    $Python = (Get-Command python -ErrorAction SilentlyContinue | Select-Object -First 1)
    if ($null -ne $Python) {
        $PythonExe = $Python.Source
    }
}

if ([string]::IsNullOrWhiteSpace($PythonExe)) {
    throw "Python 3 was not found. Install Python 3, Tkinter and pyserial first."
}

$PythonRoot = Split-Path -Parent $PythonExe
$TclSource = Join-Path $PythonRoot "tcl"
if (Test-Path -LiteralPath (Join-Path $TclSource "tcl8.6\init.tcl")) {
    $TclRuntime = Join-Path $HostDir "runtime\tcl"
    $TclTarget = Join-Path $TclRuntime "tcl8.6"
    $TkTarget = Join-Path $TclRuntime "tk8.6"
    $StampFile = Join-Path $TclRuntime "source.txt"
    $CurrentStamp = if (Test-Path -LiteralPath $StampFile) {
        Get-Content -LiteralPath $StampFile -Raw
    } else {
        ""
    }

    if (-not (Test-Path -LiteralPath (Join-Path $TclTarget "init.tcl")) -or
        $CurrentStamp.Trim() -ne $TclSource) {
        if (Test-Path -LiteralPath $TclRuntime) {
            Remove-Item -LiteralPath $TclRuntime -Recurse -Force
        }
        New-Item -ItemType Directory -Force -Path $TclRuntime | Out-Null
        Copy-Item -LiteralPath (Join-Path $TclSource "tcl8.6") -Destination $TclTarget -Recurse -Force
        Copy-Item -LiteralPath (Join-Path $TclSource "tk8.6") -Destination $TkTarget -Recurse -Force
        Set-Content -LiteralPath $StampFile -Value $TclSource -NoNewline
    }

    $env:TCL_LIBRARY = $TclTarget
    $env:TK_LIBRARY = $TkTarget
} else {
    Remove-Item Env:TCL_LIBRARY -ErrorAction SilentlyContinue
    Remove-Item Env:TK_LIBRARY -ErrorAction SilentlyContinue
}

Push-Location $HostDir
try {
    & $PythonExe (Join-Path $HostDir "pc_host.py") @AppArgs
} finally {
    Pop-Location
}

