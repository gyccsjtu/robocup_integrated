Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$script:SingleUavRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$script:SingleUavCompose = Join-Path $script:SingleUavRoot 'docker-compose.single-uav.yml'
$script:SingleUavDocker = $null

function Write-SingleUavLog([string]$Message) {
    Write-Host "[single-uav $(Get-Date -Format 'HH:mm:ss')] $Message"
}

function Assert-SingleUavDocker {
    $dockerCommand = Get-Command docker -ErrorAction SilentlyContinue
    $dockerFallback = Join-Path $env:ProgramFiles 'Docker\Docker\resources\bin\docker.exe'
    if ($dockerCommand) {
        $script:SingleUavDocker = $dockerCommand.Source
    } elseif (Test-Path -LiteralPath $dockerFallback -PathType Leaf) {
        $script:SingleUavDocker = $dockerFallback
    } else {
        throw 'Docker CLI is unavailable. Start the installed Docker Desktop first.'
    }

    & $script:SingleUavDocker info *> $null
    if ($LASTEXITCODE -ne 0) { throw 'Docker Engine is not ready. Start Docker Desktop and retry.' }
}

function Invoke-SingleUavCompose {
    param([Parameter(ValueFromRemainingArguments = $true)][string[]]$Arguments)
    Assert-SingleUavDocker
    & $script:SingleUavDocker compose --project-directory $script:SingleUavRoot -f $script:SingleUavCompose @Arguments
    if ($LASTEXITCODE -ne 0) { throw "docker compose failed with exit code $LASTEXITCODE" }
}

function Assert-SingleUavSources {
    $xtd = Join-Path $script:SingleUavRoot 'third_party\XTDrone\.git'
    $px4 = Join-Path $script:SingleUavRoot 'third_party\PX4-Autopilot\.git'
    if (-not (Test-Path -LiteralPath $xtd)) { throw 'Pinned XTDrone source is missing under third_party\XTDrone.' }
    if (-not (Test-Path -LiteralPath $px4)) { throw 'Pinned PX4 source is missing under third_party\PX4-Autopilot.' }
}
