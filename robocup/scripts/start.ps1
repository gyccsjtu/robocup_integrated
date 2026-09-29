. (Join-Path $PSScriptRoot 'common.ps1')
[void](Assert-OfficialConfig)
Write-Log 'starting the idle organizer container'
Invoke-Compose up --detach sim
Write-Log 'starting organizer-provided simulation command'
Invoke-Compose exec --detach sim bash /workspace/scripts/container/start_sim.sh
Start-Sleep -Seconds 2
& (Join-Path $PSScriptRoot 'health_check.ps1')
