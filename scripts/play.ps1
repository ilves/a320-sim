<#
.SYNOPSIS
  Starts the simulator as a standalone game window and shows its start-up progress here
  (engine start, flight model, shader compilation on the first start) until it is ready.
#>
param(
    [string]$EngineDir = '',
    [int]$ResX = 1600,
    [int]$ResY = 900,
    [switch]$Fullscreen,
    [switch]$NoWait     # start the game and close this window immediately
)

. (Join-Path $PSScriptRoot 'common.ps1')

$engine = Get-EngineOrFail $EngineDir
$dll = Join-Path $RepoRoot 'unreal\A320Sim\Binaries\Win64\A320Core.dll'
if (-not (Test-Path $dll)) {
    Fail 'The project is not built yet. Run Setup.bat first.'
}
$editor = Join-Path $engine.Dir 'Engine\Binaries\Win64\UnrealEditor.exe'
$logFile = Join-Path $RepoRoot 'unreal\A320Sim\Saved\Logs\A320Sim.log'
Remove-Item $logFile -ErrorAction SilentlyContinue

$launchArgs = @("`"$ProjectFile`"", '-game', '-log=A320Sim.log')
if ($Fullscreen) { $launchArgs += '-fullscreen' } else { $launchArgs += @('-windowed', "-ResX=$ResX", "-ResY=$ResY") }
Write-Host "Starting A320 Sim ($editor)..."
$game = Start-Process -FilePath $editor -ArgumentList $launchArgs -PassThru
if ($NoWait) { exit 0 }

Write-Host ''
Write-Host 'This window shows the start-up progress and closes itself when the simulator is ready.' -ForegroundColor Cyan
Write-Host 'The first start compiles shaders and can take 5-15 minutes; later starts take seconds.' -ForegroundColor Cyan
Write-Host ''

# Follows the game's log. Our own lines (LogA320) mark the stages; shader progress comes from
# "Preparing graphics: N shaders ..." which the game logs every 2 s while compiling.
$started = Get-Date
$stage = 'Starting Unreal Engine'
$percent = 2
$maxPending = 0
$lastStage = ''
$reader = $null
$errors = New-Object System.Collections.Generic.List[string]

function Show-Status([string]$text, [int]$pct) {
    $elapsed = (Get-Date) - $started
    Write-Progress -Activity 'A320 Sim is starting' -Status ("{0}   ({1:mm\:ss})" -f $text, $elapsed) -PercentComplete ([Math]::Min([Math]::Max($pct, 0), 100))
}

while (-not $game.HasExited) {
    if (-not $reader -and (Test-Path $logFile)) {
        $stream = [System.IO.File]::Open($logFile, 'Open', 'Read', 'ReadWrite')
        $reader = New-Object System.IO.StreamReader($stream)
    }
    if ($reader) {
        while ($null -ne ($line = $reader.ReadLine())) {
            if ($line -match 'LogA320: .*Loaded .*A320Core\.dll') { $stage = 'Engine started, loading the simulator'; $percent = [Math]::Max($percent, 10) }
            elseif ($line -match 'LogA320: .*Game mode') { $stage = 'Starting the game'; $percent = [Math]::Max($percent, 15) }
            elseif ($line -match 'LogA320: .*Flight model ready') { $stage = 'Flight model loaded, preparing graphics'; $percent = [Math]::Max($percent, 20) }
            elseif ($line -match 'LogA320: .*Preparing graphics: (\d+) shaders, (\d+) assets remaining') {
                $pending = [int]$Matches[1] + [int]$Matches[2]
                $maxPending = [Math]::Max($maxPending, $pending)
                $stage = "Compiling shaders: $($Matches[1]) remaining (first start only)"
                if ($maxPending -gt 0) { $percent = [Math]::Max($percent, 20 + [int](78 * (1 - $pending / $maxPending))) }
            }
            elseif ($line -match 'LogShaderCompilers: .*?(\d[\d,]*) (?:jobs|shaders) (?:left|remaining)') {
                if ($percent -lt 20) { $stage = "Compiling engine shaders: $($Matches[1]) remaining (first start only)" }
            }
            elseif ($line -match 'LogA320: .*READY') {
                Show-Status 'Ready' 100
                Write-Progress -Activity 'A320 Sim is starting' -Completed
                Write-Host ("Ready after {0:mm\:ss}. Switch to the A320 Sim window - have a good flight!" -f ((Get-Date) - $started)) -ForegroundColor Green
                Start-Sleep -Seconds 4
                exit 0
            }
            if ($line -match '(Error|Fatal|Failed).*' -and $line -match 'LogA320|Fatal error|LogWindows: Error') {
                $errors.Add($line)
                Write-Host $line -ForegroundColor Red
            }
        }
    }
    if ($stage -ne $lastStage) {
        Write-Host ("[{0:mm\:ss}] {1}" -f ((Get-Date) - $started), $stage)
        $lastStage = $stage
    }
    Show-Status $stage $percent
    Start-Sleep -Milliseconds 500
}

Write-Progress -Activity 'A320 Sim is starting' -Completed
if ($reader) { $reader.Close() }
Write-Host ''
Write-Host "The simulator closed before it was ready (exit code $($game.ExitCode))." -ForegroundColor Red
if (Test-Path $logFile) {
    Write-Host 'Last log lines:' -ForegroundColor Yellow
    Get-Content $logFile -Tail 25 | ForEach-Object { Write-Host "  $_" }
    Write-Host "Full log: $logFile"
}
exit 1
