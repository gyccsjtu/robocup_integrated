# Open a display-only companion; leave the PX4/Gazebo server untouched.
. (Join-Path $PSScriptRoot 'single_uav_common.ps1')
Assert-SingleUavDocker
$docker = $script:SingleUavDocker
$viewerName = 'robocup-single-uav-gui'
$names = @(& $docker ps -a --format '{{.Names}}')
if ($LASTEXITCODE -ne 0) { throw 'Could not list Docker containers.' }
if ($names -notcontains 'robocup-single-uav') {
    & (Join-Path $PSScriptRoot 'start_single_uav.ps1')
}
$sim = @(& $docker inspect robocup-single-uav | ConvertFrom-Json)[0]
if (-not $sim.State.Running) {
    & (Join-Path $PSScriptRoot 'start_single_uav.ps1')
    $sim = @(& $docker inspect robocup-single-uav | ConvertFrom-Json)[0]
}
& (Join-Path $PSScriptRoot 'health_check_single_uav.ps1')

# This socket belongs to Docker Desktop's WSLg instance, not an Ubuntu UNC mount.
# --mount fails clearly if WSLg is unavailable; it does not create a fake directory.
$socketMount = 'type=bind,src=/mnt/host/wslg/.X11-unix,dst=/tmp/.X11-unix,readonly'
& $docker run --rm --pull never --mount $socketMount --entrypoint test $sim.Image -S /tmp/.X11-unix/X0
if ($LASTEXITCODE -ne 0) {
    throw 'Docker Desktop WSLg socket is unavailable. See docs/gazebo_visual_demo.md. No flight was started.'
}

if ($names -contains $viewerName) {
    $viewer = @(& $docker inspect $viewerName | ConvertFrom-Json)[0]
    $role = $null
    if ($null -ne $viewer.Config.Labels) { $role = $viewer.Config.Labels.PSObject.Properties['robocup.role'] }
    if ($null -eq $role -or $role.Value -ne 'gazebo-viewer') {
        throw "Container $viewerName is not our labeled viewer; refusing to replace it."
    }
    if ($viewer.State.Running -and $viewer.HostConfig.NetworkMode -eq "container:$($sim.Id)" -and $viewer.Image -eq $sim.Image) {
        & $docker exec $viewerName test -f /tmp/robocup-gui-ready
        if ($LASTEXITCODE -eq 0) {
            Write-SingleUavLog 'Gazebo viewer is already open. Select Gazebo in the Windows taskbar.'
            return
        }
    }
    # Only the disposable, labeled GUI container is replaced, never the simulation.
    if ($viewer.State.Running) {
        & $docker stop --time 5 $viewerName | Out-Host
        if ($LASTEXITCODE -ne 0) { throw 'Could not stop the old display-only viewer.' }
    }
    & $docker rm $viewerName | Out-Host
    if ($LASTEXITCODE -ne 0) { throw 'Could not remove the old display-only viewer.' }
}

Write-SingleUavLog 'opening Gazebo through WSLg; the camera will follow iris'
$arguments = @('run', '-d', '--pull', 'never', '--init', '--name', $viewerName,
    '--label', 'robocup.role=gazebo-viewer', '--network', "container:$($sim.Id)",
    '--volumes-from', 'robocup-single-uav', '--mount', $socketMount,
    '-e', 'DISPLAY=:0', '-e', 'QT_X11_NO_MITSHM=1', '-e', 'QT_QPA_PLATFORM=xcb',
    '-e', 'LIBGL_ALWAYS_SOFTWARE=1', '--entrypoint', 'bash', $sim.Image,
    '/workspace/scripts/container/show_single_uav.sh')
& $docker @arguments | Out-Host
if ($LASTEXITCODE -ne 0) { throw 'Could not start the Gazebo viewer. No flight was started.' }
$timer = [Diagnostics.Stopwatch]::StartNew()
while ($timer.Elapsed.TotalSeconds -lt 65) {
    $viewer = @(& $docker inspect $viewerName | ConvertFrom-Json)[0]
    if (-not $viewer.State.Running) { break }
    & $docker exec $viewerName test -f /tmp/robocup-gui-ready
    if ($LASTEXITCODE -eq 0) {
        Write-SingleUavLog 'Gazebo viewer ready. It only displays the simulation; opening it does not arm the drone.'
        return
    }
    Start-Sleep -Seconds 2
}
& $docker logs --tail 50 $viewerName
throw 'Gazebo viewer did not become ready within 65 seconds. No flight was started.'
