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

Write-Step 'Checking Visual Studio, MSVC and the Windows SDK'
$vsInstallHelp = ("In the Visual Studio Installer choose Modify, then tick the 'Desktop development with C++' and " +
                  "'Game development with C++' workloads, and under Individual components " +
                  "'MSVC v143 - VS 2022 C++ x64/x86 build tools (Latest)' and a 'Windows 11 SDK'. Then run Setup.bat again.")
$instances = @(Get-VisualStudioInstances)
if ($instances.Count -eq 0) {
    Fail ("Visual Studio was not found. Install Visual Studio 2022 with the 'Game development with C++' workload:`n" +
          "  winget install Microsoft.VisualStudio.2022.Community --override `"--add Microsoft.VisualStudio.Workload.NativeGame --add Microsoft.VisualStudio.Workload.NativeDesktop --includeRecommended --passive`"")
}
foreach ($vs in $instances) {
    $sets = if ($vs.Msvc.Count) { ($vs.Msvc | ForEach-Object { "$_ ($(Get-PlatformToolset $_))" }) -join ', ' } else { 'no C++ compiler' }
    Write-Host "Found: $($vs.Name) at $($vs.Path): $sets"
}
$selected = Select-VisualStudio $instances
if (-not $selected) {
    Fail "None of the Visual Studio installs has the C++ compiler. $vsInstallHelp"
}
$vsPath = $selected.Instance.Path
$generator = $selected.Generator
$sdks = @(Get-WindowsSdkVersions)
if ($sdks.Count -eq 0) {
    Fail "No Windows SDK was found (needed for any C++ build). $vsInstallHelp"
}
$toolsetText = if ($selected.Toolset) { $selected.Toolset } else { 'default' }
Write-Host "Using: $($selected.Instance.Name), generator '$generator', toolset $toolsetText, Windows SDK $($sdks[0])"
if (-not $CoreOnly -and ($selected.Toolsets -notcontains 'v143')) {
    Write-Host ("WARNING: Unreal Engine 5 builds with the MSVC v143 toolset, which is not installed. " +
                "Add 'MSVC v143 - VS 2022 C++ x64/x86 build tools (Latest)' in the Visual Studio Installer if the Unreal build fails.") -ForegroundColor Yellow
}

Write-Step 'Checking CMake'
# Prefer the CMake that ships with the selected Visual Studio: it always knows its generator.
$cmakeCandidates = @(
    (Join-Path $vsPath 'Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe'),
    (Get-Command cmake -ErrorAction SilentlyContinue).Source
) | Where-Object { $_ -and (Test-Path $_) }
$cmake = $null
foreach ($candidate in $cmakeCandidates) {
    if ((& $candidate --help | Out-String) -match [regex]::Escape($generator)) { $cmake = $candidate; break }
}
if (-not $cmake) {
    Fail ("No CMake that supports '$generator' was found. Install the latest CMake ('winget install Kitware.CMake') " +
          "or add 'C++ CMake tools for Windows' in the Visual Studio Installer.")
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
$configureArgs = @('-S', (Join-Path $RepoRoot 'core'), '-B', $CoreBuildDir, '-G', $generator, '-A', 'x64',
                   "-DCMAKE_GENERATOR_INSTANCE=$vsPath", "-DCMAKE_SYSTEM_VERSION=$($sdks[0])",
                   "-DA320_JSBSIM_SOURCE_DIR=$jsbsimDir")
if ($selected.Toolset) { $configureArgs += @('-T', $selected.Toolset) }
# A cache from another generator, toolset or failed attempt cannot be reused.
$stampFile = Join-Path $BuildDir 'core-configure.txt'
$stamp = $configureArgs -join '|'
if ((Test-Path $CoreBuildDir) -and (-not (Test-Path $stampFile) -or (Get-Content $stampFile -Raw).Trim() -ne $stamp)) {
    Remove-Item -Recurse -Force $CoreBuildDir
}
& $cmake @configureArgs
if ($LASTEXITCODE -ne 0) {
    $log = Join-Path $CoreBuildDir 'CMakeFiles\CMakeConfigureLog.yaml'
    if (Test-Path $log) {
        Write-Host ''
        Write-Host 'Compiler check output (from CMakeConfigureLog.yaml):' -ForegroundColor Yellow
        Select-String -Path $log -Pattern 'error|cannot|not found' | Select-Object -Last 15 | ForEach-Object { Write-Host "  $($_.Line.Trim())" }
    }
    Fail "CMake could not set up the C++ compiler. $vsInstallHelp"
}
Set-Content -Path $stampFile -Value $stamp
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
