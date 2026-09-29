. (Join-Path $PSScriptRoot 'single_uav_common.ps1')
Assert-SingleUavSources

Set-Location -LiteralPath $script:SingleUavRoot
Write-SingleUavLog 'starting the single-UAV PX4/Gazebo/MAVROS baseline'
Invoke-SingleUavCompose up --detach --no-build sim

$ready = $false
$timer = [Diagnostics.Stopwatch]::StartNew()
while ($timer.Elapsed.TotalSeconds -lt 90) {
    & $script:SingleUavDocker exec robocup-single-uav bash /workspace/scripts/container/check_single_uav_connected.sh *> $null
    if ($LASTEXITCODE -eq 0) { $ready = $true; break }
    Write-SingleUavLog "waiting for MAVROS connection ($([int]$timer.Elapsed.TotalSeconds)s elapsed)"
    Start-Sleep -Seconds 5
}

if (-not $ready) {
    & $script:SingleUavDocker logs --tail 120 robocup-single-uav
    throw 'Simulation connection wait exceeded its 90-second budget. See the log above.'
}

& (Join-Path $PSScriptRoot 'health_check_single_uav.ps1')
if ($LASTEXITCODE -ne 0) { throw 'Single-UAV health check failed.' }
