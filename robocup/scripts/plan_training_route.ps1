param(
    [string]$Metadata = '',
    [string]$GoalId = '',
    [string]$Output = '',
    [string]$AsciiOutput = ''
)

. (Join-Path $PSScriptRoot 'single_uav_common.ps1')
Assert-SingleUavDocker

$running = & $script:SingleUavDocker inspect --format '{{.State.Running}}' robocup-single-uav 2>$null
if ($LASTEXITCODE -ne 0 -or $running -ne 'true') {
    throw 'robocup-single-uav is not running. Start a generated training city first.'
}
if (-not $Metadata) {
    $Metadata = [string](& $script:SingleUavDocker exec robocup-single-uav `
        cat /workspace/build/training_city/current_metadata.txt)
    if ($LASTEXITCODE -ne 0 -or -not $Metadata) {
        throw 'No current training metadata is recorded.'
    }
    $Metadata = $Metadata.Trim()
}
if (-not $Metadata.StartsWith('/workspace/')) {
    throw 'Metadata must be a container path under /workspace/.'
}

$arguments = @('exec', '-e', 'PYTHONPATH=/workspace/src/robocup_navigation/src',
    'robocup-single-uav', 'python3',
    '/workspace/src/robocup_navigation/scripts/plan_training_route.py',
    '--metadata', $Metadata,
    '--config', '/workspace/src/robocup_navigation/config/astar_planner.yaml')
if ($GoalId) { $arguments += @('--goal-id', $GoalId) }
if ($Output) { $arguments += @('--output', $Output) }
if ($AsciiOutput) { $arguments += @('--ascii-output', $AsciiOutput) }

& $script:SingleUavDocker @arguments
if ($LASTEXITCODE -ne 0) { throw "A* planning failed with exit code $LASTEXITCODE." }
