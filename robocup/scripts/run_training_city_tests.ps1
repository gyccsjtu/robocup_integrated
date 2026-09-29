# Offline tests for the training-city generator. No Gazebo, no GUI, no network.
. (Join-Path $PSScriptRoot 'single_uav_common.ps1')
Assert-SingleUavDocker

Set-Location -LiteralPath $script:SingleUavRoot
$command = 'bash /workspace/scripts/container/run_training_city_tests.sh'
$running = & $script:SingleUavDocker inspect --format '{{.State.Running}}' robocup-single-uav 2>$null
if ($LASTEXITCODE -eq 0 -and $running -eq 'true') {
    & $script:SingleUavDocker compose --project-directory $script:SingleUavRoot -f $script:SingleUavCompose exec -T sim bash -lc $command
} else {
    & $script:SingleUavDocker compose --project-directory $script:SingleUavRoot -f $script:SingleUavCompose run --rm --no-deps --entrypoint bash sim -lc $command
}
if ($LASTEXITCODE -ne 0) { throw "Training-city generator tests failed with exit code $LASTEXITCODE." }
Write-SingleUavLog 'training-city generator tests passed'
