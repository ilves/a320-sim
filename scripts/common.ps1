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
    # Source builds registered by the engine's Setup/GenerateProjectFiles.
    $builds = Get-ItemProperty 'HKCU:\SOFTWARE\Epic Games\Unreal Engine\Builds' -ErrorAction SilentlyContinue
    if ($builds) {
        foreach ($prop in $builds.PSObject.Properties | Where-Object { $_.Name -notlike 'PS*' }) {
            $candidates += [pscustomobject]@{ Dir = [string]$prop.Value; Version = '' }
        }
    }
    # Default and custom launcher locations on every drive: X:\Program Files\Epic Games\UE_5.x,
    # X:\Epic Games\UE_5.x, X:\UE_5.x.
    foreach ($drive in Get-PSDrive -PSProvider FileSystem -ErrorAction SilentlyContinue) {
        foreach ($parent in @((Join-Path $drive.Root 'Program Files\Epic Games'), (Join-Path $drive.Root 'Epic Games'), $drive.Root)) {
            foreach ($dir in Get-ChildItem $parent -Directory -Filter 'UE_5.*' -ErrorAction SilentlyContinue) {
                $candidates += [pscustomobject]@{ Dir = $dir.FullName; Version = $dir.Name.Substring(3) }
            }
        }
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
        Fail ("Unreal Engine 5 was not found (looked in the Epic Games Launcher's install list, the registry and " +
              "<drive>:\Program Files\Epic Games\UE_5.x, <drive>:\Epic Games\UE_5.x, <drive>:\UE_5.x).`n" +
              "Install it: Epic Games Launcher > Unreal Engine > Library > Engine Versions > + > Install (5.5 or newer).`n" +
              "If it is installed somewhere else, run: Setup.bat -EngineDir `"D:\Path\To\UE_5.x`" " +
              "(the folder that contains Engine\Binaries).")
    }
    return $engine
}

# Every Visual Studio install with the MSVC toolsets that actually have a compiler, newest first.
function Get-VisualStudioInstances {
    $vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
    if (-not (Test-Path $vswhere)) { return @() }
    $json = (& $vswhere -all -products * -prerelease -format json) | Out-String
    $result = @()
    foreach ($vs in @($json | ConvertFrom-Json)) {
        $msvcRoot = Join-Path $vs.installationPath 'VC\Tools\MSVC'
        $toolsets = @(Get-ChildItem $msvcRoot -Directory -ErrorAction SilentlyContinue |
            Where-Object { Test-Path (Join-Path $_.FullName 'bin\Hostx64\x64\cl.exe') } |
            ForEach-Object { $_.Name })
        $result += [pscustomobject]@{
            Path = $vs.installationPath
            Name = $vs.displayName
            Major = [int]($vs.installationVersion.Split('.')[0])
            Msvc = $toolsets
        }
    }
    return $result | Sort-Object Major -Descending
}

# MSVC compiler folder version (e.g. 14.44.35207) -> MSBuild platform toolset (v143).
function Get-PlatformToolset([string]$MsvcVersion) {
    $minor = [int]($MsvcVersion.Split('.')[1])
    if ($minor -ge 50) { return 'v145' }
    if ($minor -ge 30) { return 'v143' }
    if ($minor -ge 20) { return 'v142' }
    return ''
}

# Installed Windows 10/11 SDK versions that have the headers a C++ build needs, newest first.
function Get-WindowsSdkVersions {
    $roots = @()
    $key = Get-ItemProperty 'HKLM:\SOFTWARE\Microsoft\Windows Kits\Installed Roots' -ErrorAction SilentlyContinue
    if ($key -and $key.KitsRoot10) { $roots += $key.KitsRoot10 }
    $roots += Join-Path ${env:ProgramFiles(x86)} 'Windows Kits\10'
    $versions = @()
    foreach ($root in ($roots | Select-Object -Unique)) {
        foreach ($dir in Get-ChildItem (Join-Path $root 'Include') -Directory -Filter '10.*' -ErrorAction SilentlyContinue) {
            if ((Test-Path (Join-Path $dir.FullName 'um\Windows.h')) -and (Test-Path (Join-Path $dir.FullName 'ucrt\stdio.h'))) {
                $versions += $dir.Name
            }
        }
    }
    return @($versions | Select-Object -Unique | Sort-Object { [version]$_ } -Descending)
}

# Picks the Visual Studio install and toolset to build with. Unreal Engine 5 builds with
# the v143 (VS 2022) toolset, so an install that has it is preferred.
function Select-VisualStudio($Instances) {
    $usable = @($Instances | Where-Object { $_.Msvc.Count -gt 0 })
    if ($usable.Count -eq 0) { return $null }
    $withV143 = @($usable | Where-Object { @($_.Msvc | Where-Object { (Get-PlatformToolset $_) -eq 'v143' }).Count -gt 0 })
    $vs = if ($withV143.Count -gt 0) { $withV143[0] } else { $usable[0] }
    $generator = switch ($vs.Major) { 17 { 'Visual Studio 17 2022' } 18 { 'Visual Studio 18 2026' } default { "Visual Studio $($vs.Major)" } }
    $defaultToolset = switch ($vs.Major) { 17 { 'v143' } 18 { 'v145' } default { '' } }
    $installed = @($vs.Msvc | ForEach-Object { Get-PlatformToolset $_ } | Select-Object -Unique)
    # Only force a toolset when the generator's default one is not installed.
    $toolset = if ($installed -contains $defaultToolset) { '' } elseif ($installed -contains 'v143') { 'v143' } else { $installed[0] }
    return [pscustomobject]@{ Instance = $vs; Generator = $generator; Toolset = $toolset; Toolsets = $installed }
}
