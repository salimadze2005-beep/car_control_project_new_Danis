param(
    [ValidateSet('lightweight','test','check','fsds','ros2-lightweight')][string]$Mode = 'lightweight',
    [string]$FsdsHost = 'localhost',
    [string]$Python = 'python',
    [switch]$Gui
)
$ErrorActionPreference = 'Stop'
$carRepo = Split-Path -Parent $PSScriptRoot
Push-Location $carRepo
try {
    switch ($Mode) {
        'lightweight' {
            if ($Gui) { & $Python tools/simulate.py --gui }
            else { & $Python tools/simulate.py }
        }
        'test' { & $Python -m unittest discover -s tests -v }
        'check' { & $Python tools/check_fsds.py --host $FsdsHost }
        'ros2-lightweight' { & ros2 launch car_control_sim lightweight.launch.py }
        'fsds' {
            # For a user-provided native build only. Our validated bridge build is WSL2.
            & $Python tools/check_fsds.py --host $FsdsHost
            if ($LASTEXITCODE -ne 0) { throw 'FSDS TCP check failed' }
            & ros2 launch car_control_sim fsds.launch.py "host:=$FsdsHost"
        }
    }
    if ($LASTEXITCODE -ne 0) { throw "Simulation command failed (exit $LASTEXITCODE)" }
} finally { Pop-Location }
