param(
    [switch]$Headless,
    [switch]$PrepareOnly,
    [switch]$SkipBuild,
    [ValidateSet('normal', 'cancel', 'replan')]
    [string]$Case = 'normal',
    [ValidateRange(1, 3)]
    [int]$MaxAttempts = 2
)

. (Join-Path $PSScriptRoot 'single_uav_common.ps1')
Assert-SingleUavSources
Assert-SingleUavDocker
Set-Location -LiteralPath $script:SingleUavRoot
$stateDir = Join-Path $script:SingleUavRoot 'build\training_city'
[IO.Directory]::CreateDirectory($stateDir) | Out-Null
$lease = [IO.File]::Open((Join-Path $stateDir 'single_wall_host.lock'), 'OpenOrCreate', 'ReadWrite', 'None')
try {
    $attemptLimit = if ($Case -eq 'normal' -and -not $PrepareOnly) { $MaxAttempts } else { 1 }
    for ($attempt = 1; $attempt -le $attemptLimit; $attempt++) {
        if ($attemptLimit -gt 1) { Write-SingleUavLog "mission attempt $attempt of $attemptLimit" }
        $names = @(& $script:SingleUavDocker ps -a --format '{{.Names}}')
        if ($LASTEXITCODE -ne 0) { throw 'Cannot inspect containers.' }
        if ($names -contains 'robocup-single-uav') {
            $sim = @(& $script:SingleUavDocker inspect robocup-single-uav | ConvertFrom-Json)[0]
            if ($sim.State.Running) {
                Write-SingleUavLog 'checking that the old scene has no airborne UAV or active controller'
                & $script:SingleUavDocker exec robocup-single-uav bash /workspace/scripts/container/single_wall_tools.sh guard
                if ($LASTEXITCODE -ne 0) { throw 'Scene switch refused: ground/controller check failed.' }
            }
        }
        & (Join-Path $PSScriptRoot 'generate_training_city.ps1') -Preset unit -Seed 42 -Category single_wall
        if ($LASTEXITCODE -ne 0) { throw 'Map generation failed.' }
        $overlay = Join-Path $script:SingleUavRoot 'docker-compose.training-city.yml'
        Write-SingleUavLog 'loading a fresh single_wall scene; resetting the simulation to its spawn'
        & $script:SingleUavDocker compose --project-directory $script:SingleUavRoot `
            -f $script:SingleUavCompose -f $overlay up --detach --no-build --force-recreate sim
        if ($LASTEXITCODE -ne 0) { throw 'Fresh scene launch failed.' }
        & (Join-Path $PSScriptRoot 'start_training_city.ps1') -Preset unit -NoGenerate
        if ($LASTEXITCODE -ne 0) { throw 'Fresh scene did not become healthy.' }
        $sim = @(& $script:SingleUavDocker inspect robocup-single-uav | ConvertFrom-Json)[0]
        $launchId = [guid]::NewGuid().ToString()
        & $script:SingleUavDocker exec robocup-single-uav bash /workspace/scripts/container/single_wall_tools.sh receipt `
            --launch-id $launchId --container-id $sim.Id --started-at $sim.State.StartedAt
        if ($LASTEXITCODE -ne 0) { throw 'Fresh scene verification failed; no flight was started.' }
        if (-not $Headless) { & (Join-Path $PSScriptRoot 'show_single_uav.ps1') }
        if ($PrepareOnly) {
            Write-SingleUavLog 'single_wall ready; PrepareOnly leaves the UAV on the ground'
            return
        }
        Write-SingleUavLog 'starting the validated single-wall route with the existing sole controller'
        $demoArgs = @('/workspace/scripts/container/run_single_wall_demo.sh')
        if ($SkipBuild -or $attempt -gt 1) { $demoArgs += '--skip-build' }
        $demoArgs += @('--case', $Case)
        & $script:SingleUavDocker exec robocup-single-uav bash @demoArgs
        $missionExit = $LASTEXITCODE
        if ($missionExit -eq 0) { return }

        $latest = Get-ChildItem -LiteralPath (Join-Path $script:SingleUavRoot 'logs\single_wall') -Directory |
            Sort-Object LastWriteTimeUtc -Descending | Select-Object -First 1
        $retryable = $false
        $retryReason = ''
        if ($latest) {
            $validationPath = Join-Path $latest.FullName 'validation.json'
            $eventsPath = Join-Path $latest.FullName 'events.jsonl'
            if ((Test-Path -LiteralPath $validationPath) -and (Test-Path -LiteralPath $eventsPath)) {
                $validation = Get-Content -LiteralPath $validationPath -Raw | ConvertFrom-Json
                $failure = Get-Content -LiteralPath $eventsPath | ForEach-Object { $_ | ConvertFrom-Json } |
                    Where-Object event -eq 'FAILURE' | Select-Object -Last 1
                if ($failure) {
                    $retryReason = [string]$failure.detail
                    $retryable = (-not $validation.checks.took_off -and
                                  $retryReason -match 'TAKEOFF_NO_PHYSICAL_CLIMB|PREARM_STABILITY_TIMEOUT')
                }
            }
        }
        if ($retryable -and $attempt -lt $attemptLimit) {
            & $script:SingleUavDocker exec robocup-single-uav bash /workspace/scripts/container/single_wall_tools.sh guard
            if ($LASTEXITCODE -eq 0) {
                Write-SingleUavLog "bounded startup retry after $retryReason; rebuilding the confirmed-ground scene"
                continue
            }
        }
        throw "Single-wall mission ended with code $missionExit. Inspect its flight log."
    }
} finally {
    $lease.Dispose()
}
