. (Join-Path $PSScriptRoot 'common.ps1')
& (Join-Path $PSScriptRoot 'stop.ps1')
$targets = @('build', 'devel', 'install', 'logs') | ForEach-Object { Join-Path $script:WorkspaceRoot $_ }
foreach ($target in $targets) {
    $resolvedParent = [System.IO.Path]::GetFullPath($target)
    if (-not $resolvedParent.StartsWith($script:WorkspaceRoot, [System.StringComparison]::OrdinalIgnoreCase)) { throw "Unsafe reset target: $target" }
    if (Test-Path -LiteralPath $target) {
        Write-Log "removing generated runtime directory: $target"
        Remove-Item -LiteralPath $target -Recurse -Force
    }
}
$catkinMarker = Join-Path $script:WorkspaceRoot '.catkin_workspace'
if (Test-Path -LiteralPath $catkinMarker -PathType Leaf) {
    Write-Log "removing generated catkin marker: $catkinMarker"
    Remove-Item -LiteralPath $catkinMarker -Force
}
$catkinTopLevel = Join-Path $script:WorkspaceRoot 'src\CMakeLists.txt'
if (Test-Path -LiteralPath $catkinTopLevel) {
    $item = Get-Item -LiteralPath $catkinTopLevel -Force
    if (($item.Attributes -band [IO.FileAttributes]::ReparsePoint) -eq [IO.FileAttributes]::ReparsePoint) {
        Write-Log "removing generated catkin symlink: $catkinTopLevel"
        Remove-Item -LiteralPath $catkinTopLevel -Force
    }
}
New-Item -ItemType Directory -Force -Path (Join-Path $script:WorkspaceRoot 'logs') | Out-Null
Write-Log 'reset complete; source, configuration, and read-only official assets were preserved.'
