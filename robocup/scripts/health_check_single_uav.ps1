. (Join-Path $PSScriptRoot 'single_uav_common.ps1')
Assert-SingleUavDocker

$running = & $script:SingleUavDocker inspect --format '{{.State.Running}}' robocup-single-uav 2>$null
if ($LASTEXITCODE -ne 0 -or $running -ne 'true') {
    throw 'Container robocup-single-uav is not running. Run scripts\start_single_uav.ps1 first.'
}

Write-SingleUavLog 'checking ROS master, Gazebo model, clock, MAVROS state and local pose'
& $script:SingleUavDocker exec robocup-single-uav bash /workspace/scripts/container/health_check_single_uav.sh
if ($LASTEXITCODE -ne 0) { throw "single-UAV health check failed with exit code $LASTEXITCODE" }
Write-SingleUavLog 'all single-UAV checks passed'
