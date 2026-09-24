param(
    [ValidateSet('test_ground','figure_eight','slalom','turns')]
    [string]$Track = 'test_ground',
    [string]$FsdsDirectory = 'C:\FSDS',
    [string]$Distro = 'Ubuntu-22.04',
    [string]$BindAddress = ''
)
$ErrorActionPreference = 'Stop'
if (Get-Process -Name FSDS,Blocks -ErrorAction SilentlyContinue) {
    throw 'FSDS is already running. Close the current simulator before loading another map.'
}
$repository = Split-Path -Parent $PSScriptRoot
$mapPath = Join-Path $repository "simulation\tracks\$Track.csv"
$sourceSettings = Join-Path $repository 'simulation\fsds-settings.json'
if (!(Test-Path -LiteralPath $mapPath)) { throw "Missing generated map: $mapPath" }
if (!$BindAddress) {
    $route = & wsl.exe -d $Distro -- ip route show default
    if ($LASTEXITCODE -ne 0) { throw 'Cannot determine WSL gateway; pass -BindAddress explicitly.' }
    if ($route -match 'default via ([0-9.]+)') { $BindAddress = $Matches[1] }
    else { $BindAddress = '127.0.0.1' }
}
$exe = Join-Path $FsdsDirectory 'FSDS.exe'
if (!(Test-Path -LiteralPath $exe)) { throw "Missing executable: $exe" }
$runs = Join-Path $FsdsDirectory 'test_runs'
$run = Join-Path $runs (Get-Date -Format 'yyyyMMdd_HHmmss_ffff')
New-Item -ItemType Directory -Path $run | Out-Null
Copy-Item -LiteralPath $mapPath -Destination (Join-Path $run "$Track.csv")
$settings = Get-Content -Raw -LiteralPath $sourceSettings | ConvertFrom-Json
$settings.LocalHostIp = $BindAddress
$settingsPath = Join-Path $run 'settings.json'
$settings | ConvertTo-Json -Depth 20 | Set-Content -LiteralPath $settingsPath -Encoding utf8
$localMap = Join-Path $run "$Track.csv"
$argsList = @("-CustomMapPath=`"$localMap`"", '-settings', "`"$settingsPath`"", '-windowed', '-ResX=960', '-ResY=640', '-ini:Engine:[ConsoleVariables]:r.Vulkan.SubmitAfterEveryEndRenderPass=1')
$process = Start-Process -FilePath $exe -WorkingDirectory $FsdsDirectory -ArgumentList $argsList -PassThru
Write-Output "FSDS started: PID=$($process.Id), track=$Track, host=$BindAddress"
Write-Output "Run files: $run"
Write-Output "WSL: ros2 launch car_control_sim fsds_drive.launch.py host:=$BindAddress"
