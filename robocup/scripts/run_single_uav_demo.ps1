param([string]$ConfigFile = '/workspace/src/robocup_navigation/config/single_uav_waypoints.yaml')

. (Join-Path $PSScriptRoot 'single_uav_common.ps1')
Assert-SingleUavDocker

Set-Location -LiteralPath $script:SingleUavRoot
$running = & $script:SingleUavDocker inspect --format '{{.State.Running}}' robocup-single-uav 2>$null
if ($LASTEXITCODE -ne 0 -or $running -ne 'true') {
    Write-SingleUavLog 'single-UAV container is not running; starting the existing baseline'
    & (Join-Path $PSScriptRoot 'start_single_uav.ps1')
}

& (Join-Path $PSScriptRoot 'health_check_single_uav.ps1')
if ($LASTEXITCODE -ne 0) { throw 'Single-UAV health check failed; demo was not started.' }

Write-SingleUavLog 'starting the single-UAV A -> B motion demo'
& $script:SingleUavDocker exec robocup-single-uav bash /workspace/scripts/container/run_single_uav_demo.sh $ConfigFile
if ($LASTEXITCODE -ne 0) { throw "single-UAV motion demo failed with exit code $LASTEXITCODE" }
