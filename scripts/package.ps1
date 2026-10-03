<#
.SYNOPSIS
  Builds a standalone Windows game into dist\Windows and zips it as dist\A320Sim-Win64.zip.
  The zip runs on any Windows PC with a DX12 GPU; no Unreal Engine needed there.
#>
param(
    [string]$EngineDir = '',
    [ValidateSet('Development', 'Shipping')][string]$Configuration = 'Development'
)

. (Join-Path $PSScriptRoot 'common.ps1')

$engine = Get-EngineOrFail $EngineDir
if (-not (Test-Path (Join-Path $RepoRoot 'unreal\A320Sim\Source\ThirdParty\A320Core\bin\A320Core.dll'))) {
    Fail 'The flight model is not built yet. Run Setup.bat first.'
}
$dist = Join-Path $RepoRoot 'dist'
$runUat = Join-Path $engine.Dir 'Engine\Build\BatchFiles\RunUAT.bat'

Write-Step "Packaging ($Configuration) - this takes a while the first time (shader compilation)"
Invoke-Checked $runUat @('BuildCookRun', "-project=$ProjectFile", '-noP4', '-platform=Win64',
                         "-clientconfig=$Configuration", '-build', '-cook', '-stage', '-pak', '-prereqs',
                         '-archive', "-archivedirectory=$dist", '-utf8output')

$gameDir = Join-Path $dist 'Windows'
$zip = Join-Path $dist 'A320Sim-Win64.zip'
Write-Step "Zipping $gameDir"
if (Test-Path $zip) { Remove-Item $zip }
Compress-Archive -Path (Join-Path $gameDir '*') -DestinationPath $zip
Write-Host "Done: $zip  (unzip anywhere and run A320Sim.exe)" -ForegroundColor Green
