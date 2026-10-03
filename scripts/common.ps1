# Shared helpers for the Windows scripts (dot-sourced).
$ErrorActionPreference = 'Stop'

$RepoRoot = Split-Path -Parent $PSScriptRoot
$ProjectFile = Join-Path $RepoRoot 'unreal\A320Sim\A320Sim.uproject'
$BuildDir = Join-Path $RepoRoot 'build'
$EnginePathFile = Join-Path $BuildDir 'engine-path.txt'

function Write-Step([string]$Message) {
    Write-Host ''
    Write-Host "==> $Message" -ForegroundColor Cyan
}

function Fail([string]$Message) {
    Write-Host ''
    Write-Host "ERROR: $Message" -ForegroundColor Red
    exit 1
}

function Invoke-Checked([string]$Exe, [string[]]$Arguments) {
    & $Exe @Arguments
    if ($LASTEXITCODE -ne 0) {
        Fail "'$Exe $($Arguments -join ' ')' failed with exit code $LASTEXITCODE."
    }
}

# Returns @{ Dir = <engine root>; Version = '5.x' or '' } for the newest UE5 found.
function Find-UnrealEngine([string]$Override) {
    $candidates = @()
    if ($Override) {
        $candidates += [pscustomobject]@{ Dir = $Override; Version = '' }
    }
    if (Test-Path $EnginePathFile) {
        $saved = (Get-Content $EnginePathFile -Raw).Trim()
        if ($saved) { $candidates += [pscustomobject]@{ Dir = $saved; Version = '' } }
    }
    # Epic Games Launcher installs.
    $launcherDat = Join-Path $env:ProgramData 'Epic\UnrealEngineLauncher\LauncherInstalled.dat'
    if (Test-Path $launcherDat) {
        $list = (Get-Content $launcherDat -Raw | ConvertFrom-Json).InstallationList
        foreach ($item in $list) {
            if ($item.AppName -match '^UE_(5\.\d+)$') {
                $candidates += [pscustomobject]@{ Dir = $item.InstallLocation; Version = $Matches[1] }
            }
        }
    }
    foreach ($key in Get-ChildItem 'HKLM:\SOFTWARE\EpicGames\Unreal Engine' -ErrorAction SilentlyContinue) {
        if ($key.PSChildName -match '^5\.\d+$') {
            $dir = (Get-ItemProperty $key.PSPath -ErrorAction SilentlyContinue).InstalledDirectory
            if ($dir) { $candidates += [pscustomobject]@{ Dir = $dir; Version = $key.PSChildName } }
        }
    }
    foreach ($dir in Get-ChildItem 'C:\Program Files\Epic Games' -Directory -Filter 'UE_5.*' -ErrorAction SilentlyContinue) {
        $candidates += [pscustomobject]@{ Dir = $dir.FullName; Version = $dir.Name.Substring(3) }
    }

    $valid = $candidates | Where-Object { Test-Path (Join-Path $_.Dir 'Engine\Binaries\Win64\UnrealEditor.exe') }
    if ($Override -or (Test-Path $EnginePathFile)) {
        $first = $valid | Select-Object -First 1
        if ($first) { return $first }
    }
    return $valid | Sort-Object { if ($_.Version) { [version]$_.Version } else { [version]'0.0' } } -Descending | Select-Object -First 1
}

function Get-EngineOrFail([string]$Override) {
    $engine = Find-UnrealEngine $Override
    if (-not $engine) {
        Fail ("Unreal Engine 5 was not found. Install it from the Epic Games Launcher " +
              "(Unreal Engine > Library > +), or pass -EngineDir 'C:\Path\To\UE_5.x'.")
    }
    return $engine
}
