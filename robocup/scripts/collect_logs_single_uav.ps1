. (Join-Path $PSScriptRoot 'single_uav_common.ps1')
Assert-SingleUavDocker

$outputDir = Join-Path $script:SingleUavRoot ('logs\single-uav\' + (Get-Date -Format 'yyyyMMdd-HHmmss'))
New-Item -ItemType Directory -Force -Path $outputDir | Out-Null
Write-SingleUavLog "collecting logs into $outputDir"

& $script:SingleUavDocker inspect robocup-single-uav | Out-File -LiteralPath (Join-Path $outputDir 'container-inspect.json') -Encoding utf8
& $script:SingleUavDocker logs --timestamps robocup-single-uav 2>&1 | Out-File -LiteralPath (Join-Path $outputDir 'container.log') -Encoding utf8
& $script:SingleUavDocker exec robocup-single-uav bash -lc 'source /workspace/scripts/container/single_uav_env.sh >/dev/null; rosnode list; echo ---TOPICS---; rostopic list; echo ---WORLD---; rosservice call /gazebo/get_world_properties "{}"; echo ---STATE---; timeout 10s rostopic echo -n 1 /mavros/state' | Out-File -LiteralPath (Join-Path $outputDir 'ros-graph.txt') -Encoding utf8
if ($LASTEXITCODE -ne 0) { throw 'Failed to collect the ROS graph from the running container.' }
Write-SingleUavLog 'log collection completed'
