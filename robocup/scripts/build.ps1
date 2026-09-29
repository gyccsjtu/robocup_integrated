. (Join-Path $PSScriptRoot 'common.ps1')
$config = Assert-OfficialConfig
Write-Log 'validating compose configuration and local official image'
Invoke-Compose config --quiet
& $script:DockerExe image inspect $config['ROBOCUP_OFFICIAL_IMAGE'] | Out-Null
if ($LASTEXITCODE -ne 0) { throw 'Organizer image is not present locally. Import its supplied tarball or obtain it from the organizer before an offline run.' }
Write-Log 'running a clean catkin build in the organizer image'
Invoke-Compose run --rm --no-deps --entrypoint bash sim /workspace/scripts/container/init_workspace.sh
