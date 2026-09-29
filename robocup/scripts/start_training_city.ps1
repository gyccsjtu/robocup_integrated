param(
    [ValidateSet('unit', 'small', 'full')]
    [string]$Preset = 'unit',
    [int]$Seed = 42,
    [ValidateSet('', 'empty', 'single_wall', 'wall_with_gap', 'u_shape',
                 'narrow_corridor', 'blocked', 'no_path', 'goal_in_obstacle')]
    [string]$Category = '',
    [switch]$NoGenerate,
    [switch]$SkipHealthCheck
)

. (Join-Path $PSScriptRoot 'single_uav_common.ps1')
Assert-SingleUavSources
Assert-SingleUavDocker

Set-Location -LiteralPath $script:SingleUavRoot

if (-not $NoGenerate) {
    $generate = Join-Path $PSScriptRoot 'generate_training_city.ps1'
    & $generate -Preset $Preset -Seed $Seed -Category $Category
    if ($LASTEXITCODE -ne 0) { throw 'World generation failed; the simulation was not started.' }
}

$overlay = Join-Path $script:SingleUavRoot 'docker-compose.training-city.yml'
Write-SingleUavLog "starting PX4/Gazebo/MAVROS with the generated $Preset world"
& $script:SingleUavDocker compose --project-directory $script:SingleUavRoot `
    -f $script:SingleUavCompose -f $overlay up --detach --no-build sim
if ($LASTEXITCODE -ne 0) { throw 'docker compose failed while starting the training-city simulation.' }

$ready = $false
$timer = [Diagnostics.Stopwatch]::StartNew()
while ($timer.Elapsed.TotalSeconds -lt 90) {
    $state = & $script:SingleUavDocker inspect --format '{{.State.Running}}' robocup-single-uav 2>$null
    if ($LASTEXITCODE -eq 0 -and $state -ne 'true') {
        & $script:SingleUavDocker logs --tail 120 robocup-single-uav
        throw 'The simulation container exited. Most likely the world failed validation; see the log above.'
    }
    & $script:SingleUavDocker exec robocup-single-uav bash /workspace/scripts/container/check_single_uav_connected.sh *> $null
    if ($LASTEXITCODE -eq 0) { $ready = $true; break }
    Write-SingleUavLog "waiting for MAVROS connection ($([int]$timer.Elapsed.TotalSeconds)s elapsed)"
    Start-Sleep -Seconds 5
}

if (-not $ready) {
    & $script:SingleUavDocker logs --tail 120 robocup-single-uav
    throw 'Simulation connection wait exceeded its 90-second budget. See the log above.'
}

if (-not $SkipHealthCheck) {
    & (Join-Path $PSScriptRoot 'health_check_single_uav.ps1')
    if ($LASTEXITCODE -ne 0) { throw 'Single-UAV health check failed.' }
}
