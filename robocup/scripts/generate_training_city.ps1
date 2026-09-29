param(
    [ValidateSet('unit', 'small', 'full')]
    [string]$Preset = 'unit',
    [int]$Seed = 42,
    [ValidateSet('', 'empty', 'single_wall', 'wall_with_gap', 'u_shape',
                 'narrow_corridor', 'blocked', 'no_path', 'goal_in_obstacle')]
    [string]$Category = '',
    [string]$Output = ''
)

. (Join-Path $PSScriptRoot 'single_uav_common.ps1')
Assert-SingleUavDocker

$command = "bash /workspace/scripts/container/generate_training_city.sh --preset $Preset --seed $Seed"
if ($Category) { $command += " --category $Category" }
if ($Output) { $command += " --output $Output" }

Set-Location -LiteralPath $script:SingleUavRoot
$running = & $script:SingleUavDocker inspect --format '{{.State.Running}}' robocup-single-uav 2>$null
if ($LASTEXITCODE -eq 0 -and $running -eq 'true') {
    Write-SingleUavLog "generating the $Preset training world inside the running container"
    & $script:SingleUavDocker compose --project-directory $script:SingleUavRoot -f $script:SingleUavCompose exec -T sim bash -lc $command
} else {
    Write-SingleUavLog "generating the $Preset training world (one-shot container)"
    & $script:SingleUavDocker compose --project-directory $script:SingleUavRoot -f $script:SingleUavCompose run --rm --no-deps --entrypoint bash sim -lc $command
}
if ($LASTEXITCODE -ne 0) {
    throw "Training world generation failed with exit code $LASTEXITCODE. No world was recorded."
}

Write-SingleUavLog "world and metadata are under src\robocup_training_worlds\worlds\generated"
