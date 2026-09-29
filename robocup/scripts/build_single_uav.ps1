param(
    [switch]$Clean
)

. (Join-Path $PSScriptRoot 'single_uav_common.ps1')
Assert-SingleUavSources

Set-Location -LiteralPath $script:SingleUavRoot
Write-SingleUavLog 'building the pinned Ubuntu 20.04 / ROS Noetic development image'
Invoke-SingleUavCompose build sim

if ($Clean) {
    $targets = @(
        (Join-Path $script:SingleUavRoot 'third_party\PX4-Autopilot\build\px4_sitl_default'),
        (Join-Path $script:SingleUavRoot 'build\xtdrone_catkin_ws')
    )
    foreach ($target in $targets) {
        $absolute = [System.IO.Path]::GetFullPath($target)
        if (-not $absolute.StartsWith($script:SingleUavRoot, [System.StringComparison]::OrdinalIgnoreCase)) {
            throw "Unsafe clean target: $absolute"
        }
        if (Test-Path -LiteralPath $absolute) {
            Write-SingleUavLog "removing generated build directory: $absolute"
            Remove-Item -LiteralPath $absolute -Recurse -Force
        }
    }
}

Write-SingleUavLog 'applying the pinned XTDrone overlay and compiling PX4 SITL/Gazebo'
$buildCommand = @'
set -Eeuo pipefail
bash /workspace/scripts/container/prepare_xtdrone_px4.sh
source /opt/ros/noetic/setup.bash
cd /workspace/third_party/PX4-Autopilot
DONT_RUN=1 make px4_sitl_default gazebo
bash /workspace/scripts/container/build_xtdrone_gazebo_ros.sh
'@
Invoke-SingleUavCompose run --rm --no-deps --entrypoint bash sim -lc $buildCommand
Write-SingleUavLog 'build completed successfully'
