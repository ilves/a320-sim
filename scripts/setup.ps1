<#
.SYNOPSIS
  One-time setup on Windows: builds the flight model (JSBSim + A320 core) as A320Core.dll,
  runs its tests, installs it into the Unreal project and compiles the Unreal project.

.PARAMETER EngineDir   Unreal Engine root (the folder containing Engine\). Auto-detected if omitted.
.PARAMETER CoreOnly    Build and test the flight model only; skip Unreal.
.PARAMETER SkipTests   Do not run the flight-model tests.
#>
param(
    [string]$EngineDir = '',
    [switch]$CoreOnly,
    [switch]$SkipTests
)

. (Join-Path $PSScriptRoot 'common.ps1')

$JsbsimVersion = '1.3.1'
$CoreBuildDir = Join-Path $BuildDir 'core'
$ThirdPartyDir = Join-Path $RepoRoot 'unreal\A320Sim\Source\ThirdParty\A320Core'
$BinariesDir = Join-Path $RepoRoot 'unreal\A320Sim\Binaries\Win64'
New-Item -ItemType Directory -Force -Path $BuildDir | Out-Null

Write-Step 'Checking Visual Studio (C++ toolset)'
$vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
if (-not (Test-Path $vswhere)) {
    Fail ("Visual Studio 2022 was not found. Install it with the 'Game development with C++' workload:`n" +
          "  winget install Microsoft.VisualStudio.2022.Community --override `"--add Microsoft.VisualStudio.Workload.NativeGame --includeRecommended --passive`"")
}
$vsPath = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
$vsMajor = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationVersion
if (-not $vsPath) {
    Fail "Visual Studio is installed without the C++ tools. Add the 'Game development with C++' workload in the Visual Studio Installer."
}
$vsMajor = [int]($vsMajor.Split('.')[0])
$generator = switch ($vsMajor) { 17 { 'Visual Studio 17 2022' } 18 { 'Visual Studio 18 2026' } default { 'Visual Studio 17 2022' } }
Write-Host "Visual Studio: $vsPath ($generator)"

Write-Step 'Checking CMake'
$cmake = (Get-Command cmake -ErrorAction SilentlyContinue).Source
if (-not $cmake) {
    $bundled = Join-Path $vsPath 'Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe'
    if (Test-Path $bundled) { $cmake = $bundled }
}
if (-not $cmake) {
    Fail "CMake was not found. Install it ('winget install Kitware.CMake') or add the 'C++ CMake tools for Windows' component in the Visual Studio Installer."
}
$ctest = Join-Path (Split-Path $cmake) 'ctest.exe'
Write-Host "CMake: $cmake ($((& $cmake --version | Select-Object -First 1)))"

Write-Step "Fetching JSBSim $JsbsimVersion source"
$jsbsimDir = Join-Path $BuildDir "jsbsim-$JsbsimVersion"
if (-not (Test-Path (Join-Path $jsbsimDir 'CMakeLists.txt'))) {
    $zip = Join-Path $BuildDir "jsbsim-$JsbsimVersion.zip"
    [Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12
    $ProgressPreference = 'SilentlyContinue'
    Invoke-WebRequest -Uri "https://github.com/JSBSim-Team/jsbsim/archive/refs/tags/v$JsbsimVersion.zip" -OutFile $zip
    Expand-Archive -Path $zip -DestinationPath $BuildDir -Force
    Remove-Item $zip
}
Write-Host "JSBSim: $jsbsimDir"

Write-Step 'Building the flight model (A320Core.dll)'
Invoke-Checked $cmake @('-S', (Join-Path $RepoRoot 'core'), '-B', $CoreBuildDir, '-G', $generator, '-A', 'x64',
                        "-DA320_JSBSIM_SOURCE_DIR=$jsbsimDir")
Invoke-Checked $cmake @('--build', $CoreBuildDir, '--config', 'Release', '--parallel')

if (-not $SkipTests) {
    Write-Step 'Testing the flight model (takeoff, ILS approaches and landings in JSBSim)'
    Invoke-Checked $ctest @('--test-dir', $CoreBuildDir, '-C', 'Release', '--output-on-failure')
}

Write-Step 'Installing A320Core into the Unreal project'
Invoke-Checked $cmake @('--install', $CoreBuildDir, '--config', 'Release', '--prefix', $ThirdPartyDir)
New-Item -ItemType Directory -Force -Path $BinariesDir | Out-Null
Copy-Item (Join-Path $ThirdPartyDir 'bin\A320Core.dll') $BinariesDir -Force

if ($CoreOnly) {
    Write-Step 'Done (flight model only).'
    exit 0
}

Write-Step 'Locating Unreal Engine 5'
$engine = Get-EngineOrFail $EngineDir
Write-Host "Unreal Engine: $($engine.Dir) $($engine.Version)"
Set-Content -Path $EnginePathFile -Value $engine.Dir

if ($engine.Version) {
    # Associates the .uproject with this engine, so double-clicking it opens without asking.
    $project = Get-Content $ProjectFile -Raw | ConvertFrom-Json
    if ($project.EngineAssociation -ne $engine.Version) {
        $project.EngineAssociation = $engine.Version
        $project | ConvertTo-Json -Depth 10 | Set-Content -Path $ProjectFile -Encoding UTF8
    }
}

Write-Step 'Compiling the Unreal project (first time takes a few minutes)'
$buildBat = Join-Path $engine.Dir 'Engine\Build\BatchFiles\Build.bat'
Invoke-Checked $buildBat @('A320SimEditor', 'Win64', 'Development', "-Project=$ProjectFile", '-WaitMutex')

Write-Step 'Setup complete'
Write-Host 'Run Play.bat to fly, OpenEditor.bat to open the project, Package.bat to build a standalone game.' -ForegroundColor Green
