. (Join-Path $PSScriptRoot 'common.ps1')
$exitCode = 0
try { [void](Assert-OfficialConfig) } catch { Write-Error $_; exit 1 }
Write-Log 'running container health checks'
& $script:DockerExe compose --env-file $script:EnvFile --project-directory $script:WorkspaceRoot -f $script:ComposeFile exec --no-TTY sim bash /workspace/scripts/container/health_check.sh
$exitCode = $LASTEXITCODE
if ($exitCode -ne 0) { throw "health check failed with exit code $exitCode" }
