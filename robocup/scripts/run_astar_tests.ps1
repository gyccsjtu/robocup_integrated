. (Join-Path $PSScriptRoot 'single_uav_common.ps1')
Assert-SingleUavDocker

Set-Location -LiteralPath $script:SingleUavRoot
$running = & $script:SingleUavDocker inspect --format '{{.State.Running}}' robocup-single-uav 2>$null
if ($LASTEXITCODE -eq 0 -and $running -eq 'true') {
    & $script:SingleUavDocker exec robocup-single-uav bash /workspace/scripts/container/run_astar_tests.sh
} else {
    & $script:SingleUavDocker compose --project-directory $script:SingleUavRoot `
        -f $script:SingleUavCompose run --rm --no-deps --entrypoint bash sim `
        /workspace/scripts/container/run_astar_tests.sh
}
if ($LASTEXITCODE -ne 0) { throw "A* tests failed with exit code $LASTEXITCODE." }
Write-SingleUavLog 'offline A* tests passed'
