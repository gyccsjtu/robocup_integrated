Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$script:WorkspaceRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$script:EnvFile = Join-Path $script:WorkspaceRoot '.env'
$script:ComposeFile = Join-Path $script:WorkspaceRoot 'docker-compose.yml'
$script:DockerExe = $null

function Write-Log([string]$Message) {
    Write-Host "[robocup] $Message"
}

function Assert-DockerReady {
    $dockerCommand = Get-Command docker -ErrorAction SilentlyContinue
    $dockerFallback = 'C:\Program Files\Docker\Docker\resources\bin\docker.exe'
    if ($dockerCommand) {
        $script:DockerExe = $dockerCommand.Source
    } elseif (Test-Path -LiteralPath $dockerFallback -PathType Leaf) {
        $script:DockerExe = $dockerFallback
    } else {
        throw 'Docker CLI is unavailable. Install/configure Docker Desktop with WSL2 integration first; this script will not install it.'
    }
    if (-not (Test-Path -LiteralPath $script:EnvFile)) {
        throw "Missing $script:EnvFile. Copy .env.example to .env and fill only organizer-confirmed values."
    }
}

function Assert-OfficialConfig {
    Assert-DockerReady
    $required = 'ROBOCUP_OFFICIAL_IMAGE', 'ROBOCUP_OFFICIAL_ASSETS_DIR', 'ROBOCUP_ROS_SETUP', 'ROBOCUP_OFFICIAL_START_COMMAND'
    $pairs = @{}
    foreach ($line in Get-Content -LiteralPath $script:EnvFile) {
        if ($line -match '^\s*([A-Za-z_][A-Za-z0-9_]*)=(.*)$') { $pairs[$Matches[1]] = $Matches[2].Trim() }
    }
    foreach ($key in $required) {
        if (-not $pairs.ContainsKey($key) -or [string]::IsNullOrWhiteSpace($pairs[$key]) -or $pairs[$key] -match '^PENDING_') {
            throw "Official configuration is incomplete: $key in .env. Do not guess this value."
        }
    }
    $assetPath = $pairs['ROBOCUP_OFFICIAL_ASSETS_DIR']
    if (-not [System.IO.Path]::IsPathRooted($assetPath)) { $assetPath = Join-Path $script:WorkspaceRoot $assetPath }
    if (-not (Test-Path -LiteralPath $assetPath -PathType Container)) {
        throw "Official assets directory is missing: $assetPath"
    }
    $pairs
}

function Invoke-Compose {
    param([Parameter(ValueFromRemainingArguments = $true)][string[]]$Arguments)
    Assert-DockerReady
    & $script:DockerExe compose --env-file $script:EnvFile --project-directory $script:WorkspaceRoot -f $script:ComposeFile @Arguments
    if ($LASTEXITCODE -ne 0) { throw "docker compose failed with exit code $LASTEXITCODE" }
}
