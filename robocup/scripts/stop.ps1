. (Join-Path $PSScriptRoot 'common.ps1')
if (Test-Path -LiteralPath $script:EnvFile) {
    Write-Log 'stopping containers without deleting images or official assets'
    Invoke-Compose down --remove-orphans
} else {
    Write-Log 'No .env exists; nothing to stop.'
}
