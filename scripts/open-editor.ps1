<#
.SYNOPSIS
  Opens the project in the Unreal Editor (press Play there to fly inside the editor).
#>
param([string]$EngineDir = '')

. (Join-Path $PSScriptRoot 'common.ps1')

$engine = Get-EngineOrFail $EngineDir
Start-Process -FilePath (Join-Path $engine.Dir 'Engine\Binaries\Win64\UnrealEditor.exe') -ArgumentList "`"$ProjectFile`""
