. (Join-Path $PSScriptRoot 'single_uav_common.ps1')
Set-Location -LiteralPath $script:SingleUavRoot
Write-SingleUavLog 'stopping the single-UAV baseline (images and build products are preserved)'
Invoke-SingleUavCompose down --remove-orphans
Write-SingleUavLog 'stopped'
