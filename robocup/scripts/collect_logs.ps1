. (Join-Path $PSScriptRoot 'common.ps1')
Assert-DockerReady
$outputDir = Join-Path $script:WorkspaceRoot ('logs\host\' + (Get-Date -Format 'yyyyMMdd-HHmmss'))
New-Item -ItemType Directory -Force -Path $outputDir | Out-Null
Write-Log "collecting Docker logs into $outputDir"
& $script:DockerExe compose --env-file $script:EnvFile --project-directory $script:WorkspaceRoot -f $script:ComposeFile logs --no-color --timestamps | Out-File -LiteralPath (Join-Path $outputDir 'docker-compose.log') -Encoding utf8
& $script:DockerExe compose --env-file $script:EnvFile --project-directory $script:WorkspaceRoot -f $script:ComposeFile exec --no-TTY sim bash -lc 'source "$ROBOCUP_ROS_SETUP" && { rosnode list; rostopic list; }' | Out-File -LiteralPath (Join-Path $outputDir 'ros-graph.txt') -Encoding utf8
Write-Log 'log collection complete'
