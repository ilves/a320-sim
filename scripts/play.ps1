<#
.SYNOPSIS
  Starts the simulator as a standalone game window (no packaging needed).
#>
param(
    [string]$EngineDir = '',
    [int]$ResX = 1600,
    [int]$ResY = 900,
    [switch]$Fullscreen
)

. (Join-Path $PSScriptRoot 'common.ps1')

$engine = Get-EngineOrFail $EngineDir
$dll = Join-Path $RepoRoot 'unreal\A320Sim\Binaries\Win64\A320Core.dll'
if (-not (Test-Path $dll)) {
    Fail 'The project is not built yet. Run Setup.bat first.'
}
$editor = Join-Path $engine.Dir 'Engine\Binaries\Win64\UnrealEditor.exe'
$launchArgs = @("`"$ProjectFile`"", '-game', '-log=A320Sim.log')
if ($Fullscreen) { $launchArgs += '-fullscreen' } else { $launchArgs += @('-windowed', "-ResX=$ResX", "-ResY=$ResY") }
Write-Host "Starting A320 Sim ($editor)..."
Start-Process -FilePath $editor -ArgumentList $launchArgs
