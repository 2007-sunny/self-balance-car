param(
  [string]$Port = "",
  [int]$Baud = 9600,
  [int]$Window = 20
)

$Root = Split-Path -Parent $PSScriptRoot
$Python = Join-Path $Root ".venv\Scripts\python.exe"
$Script = Join-Path $PSScriptRoot "telemetry_plot.py"

if (-not (Test-Path $Python)) {
  Write-Error "Python virtual environment not found: $Python"
  exit 1
}

if ($Port -eq "") {
  & $Python $Script --list-ports
  Write-Host ""
  Write-Host "Run again with: .\tools\run_telemetry_plot.ps1 -Port COM5"
  exit 0
}

& $Python $Script --port $Port --baud $Baud --window $Window
