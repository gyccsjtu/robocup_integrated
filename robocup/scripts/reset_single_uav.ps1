. (Join-Path $PSScriptRoot 'single_uav_common.ps1')

try { & (Join-Path $PSScriptRoot 'stop_single_uav.ps1') } catch {
    Write-SingleUavLog "stop was not required or Docker was unavailable: $($_.Exception.Message)"
}

$targets = @(
    (Join-Path $script:SingleUavRoot 'build\xtdrone_catkin_ws'),
    (Join-Path $script:SingleUavRoot 'third_party\PX4-Autopilot\build\px4_sitl_default')
)
foreach ($target in $targets) {
    $absolute = [System.IO.Path]::GetFullPath($target)
    if (-not $absolute.StartsWith($script:SingleUavRoot, [System.StringComparison]::OrdinalIgnoreCase)) {
        throw "Unsafe reset target: $absolute"
    }
    if (Test-Path -LiteralPath $absolute) {
        Write-SingleUavLog "removing generated build directory: $absolute"
        Remove-Item -LiteralPath $absolute -Recurse -Force
    }
}
Write-SingleUavLog 'reset completed; images, pinned sources, configuration and pre.data were preserved'
