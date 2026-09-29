param(
    [ValidateRange(1, 20)]
    [int]$Runs = 5,
    [switch]$IncludeCancel,
    [switch]$SkipBuild
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$demo = Join-Path $PSScriptRoot 'run_single_wall_demo.ps1'
for ($index = 1; $index -le $Runs; $index++) {
    Write-Host "[single-wall-validation] normal run $index of $Runs"
    & $demo -Headless -SkipBuild:$SkipBuild -Case normal
}
if ($IncludeCancel) {
    Write-Host '[single-wall-validation] controlled-cancel run'
    & $demo -Headless -SkipBuild:$SkipBuild -Case cancel
}
