param([string]$ConfigFile = '/workspace/src/robocup_navigation/config/single_uav_waypoints.yaml')

$ErrorActionPreference = 'Stop'
& (Join-Path $PSScriptRoot 'show_single_uav.ps1')
Write-Host 'Gazebo is ready. Starting the existing A/B flight demo in 3 seconds. Keep Docker running until landing completes.'
Start-Sleep -Seconds 3
& (Join-Path $PSScriptRoot 'run_single_uav_demo.ps1') -ConfigFile $ConfigFile
