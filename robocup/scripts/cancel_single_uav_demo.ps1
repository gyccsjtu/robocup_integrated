. (Join-Path $PSScriptRoot 'single_uav_common.ps1')
Assert-SingleUavDocker
$command = @'
source /workspace/scripts/container/single_uav_env.sh >/dev/null
timeout 10 rostopic pub -1 /uav_navigation/cancel std_msgs/Empty '{}'
'@
& $script:SingleUavDocker exec robocup-single-uav bash -lc $command
if ($LASTEXITCODE -ne 0) { throw 'Could not publish cancellation; inspect controller status.' }
Write-SingleUavLog 'cancel requested; wait for LANDED / RESULT in the controller terminal before stopping the simulation'
