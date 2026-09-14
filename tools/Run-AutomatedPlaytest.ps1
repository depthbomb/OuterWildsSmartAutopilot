param(
    [string[]]$Targets = @('VolcanicMoon_Body', 'Comet_Body'),
    [ValidateRange(1, 10)][int]$Repeat = 1,
    [ValidateRange(0, 900)][double]$LaunchAtSeconds = 35,
    [ValidateRange(5, 600)][double]$LegTimeoutSeconds = 180,
    [ValidateRange(30, 1200)][double]$SetupTimeoutSeconds = 180,
    [ValidateRange(-1, 300)][double]$CancelAfterSeconds = -1,
    [ValidateRange(0.5, 15)][double]$ReengageDelaySeconds = 2,
    [string]$StartBody,
    [float[]]$StartOffset,
    [float[]]$StartVelocity = @(0, 0, 0),
    [ValidateSet('', 'fuel-exhaustion', 'autopilot-damage', 'insufficient-thrust', 'blocked-route')][string]$Fault = '',
    [string]$OwmlPath = (Join-Path $env:APPDATA 'OuterWildsModManager\OWML'),
    [switch]$Build
)

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest
if ($Build) {
    if (Get-Process -Name OuterWilds, OWML.Launcher -ErrorAction SilentlyContinue) {
        throw 'Close the game and launcher before building the test mod.'
    }

    $project = Join-Path (Split-Path $PSScriptRoot -Parent) 'SmartAutopilot\SmartAutopilot.csproj'
    $output = Join-Path $OwmlPath 'Mods\Depthbomb.SmartAutopilot'
    dotnet build $project -c Release "-p:OutputPath=$output"
    if ($LASTEXITCODE -ne 0) {
        throw 'Mod build failed.'
    }
}

$root = Join-Path ([Environment]::GetFolderPath('MyDocuments')) 'Codex\ProjectNotes\outer-wilds-smart-autopilot\automation'
$saveRoot = Join-Path $env:USERPROFILE 'AppData\LocalLow\Mobius Digital\Outer Wilds'
$launcherPath = Join-Path $OwmlPath 'OWML.Launcher.exe'
$modPath = Join-Path $OwmlPath 'Mods\Depthbomb.SmartAutopilot'
$gameConfig = Get-Content -LiteralPath (Join-Path $OwmlPath 'OWML.Config.json') -Raw | ConvertFrom-Json
$gameExe = [IO.Path]::GetFullPath((Join-Path $gameConfig.gamePath 'OuterWilds.exe'))
if (!(Test-Path -LiteralPath $launcherPath -PathType Leaf) -or !(Test-Path -LiteralPath $gameExe -PathType Leaf)) {
    throw 'OWML launcher or Outer Wilds executable is missing.'
}

if (!(Get-Content -LiteralPath (Join-Path $modPath 'config.json') -Raw | ConvertFrom-Json).enabled) {
    throw 'Smart Autopilot must be enabled before automated testing.'
}

if ($Targets.Count -lt 1 -or $Targets.Count -gt 12 -or ($StartBody -and ($StartOffset.Count -ne 3 -or $StartVelocity.Count -ne 3))) {
    throw 'Provide 1-12 targets and three-component vectors for an anchored start.'
}

New-Item -ItemType Directory -Path $root -Force | Out-Null
$requestPath = Join-Path $root 'request.json'
$lockPath = Join-Path $root 'controller.lock'
$controllerLock = [IO.File]::Open($lockPath, [IO.FileMode]::OpenOrCreate, [IO.FileAccess]::ReadWrite, [IO.FileShare]::None)

function Write-AtomicJson {
    param([string]$Path, [object]$Value)
    $Value | ConvertTo-Json -Depth 12 | Set-Content -LiteralPath ($Path + '.tmp') -Encoding utf8
    Move-Item -LiteralPath ($Path + '.tmp') -Destination $Path -Force
}

function Get-OwnedGame {
    param([datetime]$Since)
    foreach ($candidate in @(Get-Process -Name OuterWilds -ErrorAction SilentlyContinue)) {
        if ($candidate.StartTime.ToUniversalTime() -ge $Since -and [IO.Path]::GetFullPath($candidate.Path) -eq $gameExe) {
            return $candidate
        }
    }
}

function Read-SharedJson {
    param([string]$Path)
    $stream = [IO.File]::Open($Path, [IO.FileMode]::Open, [IO.FileAccess]::Read, ([IO.FileShare]::ReadWrite -bor [IO.FileShare]::Delete))
    $reader = [IO.StreamReader]::new($stream)
    try {
        return $reader.ReadToEnd() | ConvertFrom-Json
    }
    finally {
        $reader.Dispose()
    }
}

function Assert-SavePath {
    param([string]$Path)
    $resolved = [IO.Path]::GetFullPath($Path)
    $allowed = @([IO.Path]::GetFullPath((Join-Path $saveRoot 'SteamSaves')), [IO.Path]::GetFullPath((Join-Path $saveRoot 'Backup')))
    if ($resolved -notin $allowed -or ((Get-Item -LiteralPath $resolved).Attributes -band [IO.FileAttributes]::ReparsePoint)) {
        throw "Save path is outside the expected ordinary directories: $resolved"
    }
}

function Get-SaveFingerprint {
    param([string]$Directory)
    $prefix = [IO.Path]::GetFullPath($Directory).TrimEnd('\') + '\'
    $entries = foreach ($file in Get-ChildItem -LiteralPath $Directory -File -Recurse) {
        [pscustomobject]@{
            path = $file.FullName.Substring($prefix.Length)
            hash = (Get-FileHash -LiteralPath $file.FullName).Hash
        }
    }

    return @($entries | Sort-Object path)
}

try {
    for ($iteration = 1; $iteration -le $Repeat; $iteration++) {
        if (Get-Process -Name OuterWilds, OWML.Launcher -ErrorAction SilentlyContinue) {
            throw 'An existing game or OWML launcher is running. Automated testing requires its own session.'
        }

        if (Test-Path -LiteralPath $requestPath) {
            throw "An unclaimed automation request already exists: $requestPath"
        }

        $runId = [guid]::NewGuid().ToString('N')
        $runPath = Join-Path $root $runId
        New-Item -ItemType Directory -Path $runPath | Out-Null
        Copy-Item -LiteralPath (Join-Path $modPath 'config.json') -Destination (Join-Path $runPath 'mod-config.json')
        Copy-Item -LiteralPath (Join-Path $modPath 'manifest.json') -Destination (Join-Path $runPath 'mod-manifest.json')
        $saveBackup = Join-Path $runPath 'save-before'
        New-Item -ItemType Directory -Path $saveBackup | Out-Null
        $savedFolders = @()
        foreach ($name in @('SteamSaves', 'Backup')) {
            $source = Join-Path $saveRoot $name
            if (Test-Path -LiteralPath $source) {
                Assert-SavePath $source
                Copy-Item -LiteralPath $source -Destination $saveBackup -Recurse
                $savedFolders += $name
            }
        }

        if ('SteamSaves' -notin $savedFolders) {
            throw 'No local Steam save directory was found to preserve.'
        }

        $saveFingerprint = @(Get-SaveFingerprint $saveBackup)
        Write-AtomicJson (Join-Path $runPath 'save-manifest.json') $saveFingerprint

        $hostResult = [ordered]@{
            id = $runId
            status = 'running'
            saveRestored = $false
            forcedShutdown = $false
            gameProcessId = $null
            result = $null
            error = $null
        }
        $launched = [datetime]::UtcNow
        $request = [ordered]@{
            protocol = 1
            id = $runId
            expiresUtc = $launched.AddMinutes(100).ToString('O')
            dllHash = (Get-FileHash -LiteralPath (Join-Path $modPath 'SmartAutopilot.dll')).Hash
            targets = $Targets
            launchAtSeconds = $LaunchAtSeconds
            legTimeoutSeconds = $LegTimeoutSeconds
            setupTimeoutSeconds = $SetupTimeoutSeconds
            cancelAfterSeconds = $CancelAfterSeconds
            reengageDelaySeconds = $ReengageDelaySeconds
            startBody = $StartBody
            startOffset = $StartOffset
            startVelocity = $StartVelocity
            fault = $Fault
        }
        $leasePath = Join-Path $runPath 'lease'
        Set-Content -LiteralPath $leasePath -Value $PID
        Write-AtomicJson (Join-Path $runPath 'request.json') $request
        $gameProcess = $null
        $launcher = $null
        $lastStage = ''
        $failure = $null
        try {
            Write-AtomicJson $requestPath $request
            Write-Output "Automated run $iteration/$Repeat`: $runPath"
            $launcher = Start-Process -FilePath $launcherPath -WorkingDirectory $OwmlPath -WindowStyle Hidden -PassThru -RedirectStandardOutput (Join-Path $runPath 'launcher.stdout.log') -RedirectStandardError (Join-Path $runPath 'launcher.stderr.log')
            $deadline = $launched.AddSeconds($SetupTimeoutSeconds + $Targets.Count * ($LegTimeoutSeconds + 20) + 90)
            $finishedAt = $null
            while ([datetime]::UtcNow -lt $deadline) {
                Set-Content -LiteralPath $leasePath -Value $PID
                if (!$gameProcess) {
                    $gameProcess = Get-OwnedGame $launched
                    if ($gameProcess) {
                        $hostResult.gameProcessId = $gameProcess.Id
                    }
                }

                $statusPath = Join-Path $runPath 'status.json'
                if (Test-Path -LiteralPath $statusPath) {
                    try {
                        $status = Read-SharedJson $statusPath
                    }
                    catch [IO.IOException] {
                        Start-Sleep -Milliseconds 100
                        continue
                    }
                    if ($status.id -ne $runId -or $status.dllHash -ne $request.dllHash -or ($gameProcess -and $status.gameProcessId -ne $gameProcess.Id)) {
                        throw 'Automation status does not match this run, process and build.'
                    }

                    if ($status.stage -ne $lastStage) {
                        $lastStage = $status.stage
                        Write-Output "Stage: $lastStage; status: $($status.status); legs: $($status.legs.Count)"
                    }

                    if ($status.status -ne 'running' -and !$finishedAt) {
                        $finishedAt = [datetime]::UtcNow
                    }
                }

                if ($gameProcess) {
                    $gameProcess.Refresh()
                    if ($gameProcess.HasExited) {
                        break
                    }
                }
                elseif (([datetime]::UtcNow - $launched).TotalSeconds -gt 90) {
                    throw 'The game did not start within 90 seconds.'
                }

                if (!$lastStage -and ([datetime]::UtcNow - $launched).TotalSeconds -gt 90) {
                    throw 'The installed mod did not claim the automation request within 90 seconds.'
                }

                if ($finishedAt -and ([datetime]::UtcNow - $finishedAt).TotalSeconds -gt 45) {
                    throw 'The test finished but the game did not close.'
                }

                Start-Sleep -Seconds 2
            }

            if (!$gameProcess -or !$gameProcess.HasExited) {
                throw 'Automated run exceeded its wall-clock deadline.'
            }

            $resultPath = Join-Path $runPath 'result.json'
            if (!(Test-Path -LiteralPath $resultPath)) {
                throw 'The game exited without a test result.'
            }

            $result = Get-Content -LiteralPath $resultPath -Raw | ConvertFrom-Json
            if ($result.id -ne $runId -or $result.dllHash -ne $request.dllHash -or $result.gameProcessId -ne $gameProcess.Id) {
                throw 'Final result does not match the launched run.'
            }

            $hostResult.result = $result
            $hostResult.status = $result.status
            if ($result.status -ne 'passed') {
                throw "In-game test failed: $($result.reason)"
            }

            if ($result.shutdown -ne 'returned-to-title') {
                throw 'The game exited before completing the return-to-title step.'
            }
        }
        catch {
            $failure = $_
            $hostResult.status = 'failed'
            $hostResult.error = $_.ToString()
        }
        finally {
            Set-Content -LiteralPath (Join-Path $runPath 'stop') -Value $runId
            if (!$gameProcess) {
                $gameProcess = Get-OwnedGame $launched
            }

            if ($gameProcess -and !$gameProcess.HasExited) {
                $null = $gameProcess.CloseMainWindow()
                if (!$gameProcess.WaitForExit(20000)) {
                    $hostResult.forcedShutdown = $true
                    Stop-Process -InputObject $gameProcess -Force
                    $null = $gameProcess.WaitForExit(10000)
                }
            }

            if ($launcher -and !$launcher.HasExited) {
                Stop-Process -InputObject $launcher -Force
            }

            if (Get-Process -Name OuterWilds -ErrorAction SilentlyContinue) {
                throw "Save restore deferred because a game is still running. Backup: $saveBackup"
            }

            $afterPath = Join-Path $runPath 'save-after'
            New-Item -ItemType Directory -Path $afterPath | Out-Null
            foreach ($name in $savedFolders) {
                $destination = Join-Path $saveRoot $name
                if (Test-Path -LiteralPath $destination) {
                    Assert-SavePath $destination
                    $archive = [IO.Path]::GetFullPath((Join-Path $afterPath $name))
                    if (!$archive.StartsWith([IO.Path]::GetFullPath($runPath) + '\', [StringComparison]::OrdinalIgnoreCase)) {
                        throw 'Save archive escaped the current run directory.'
                    }

                    Move-Item -LiteralPath $destination -Destination $archive
                }

                Copy-Item -LiteralPath (Join-Path $saveBackup $name) -Destination $saveRoot -Recurse
            }

            $restoredFingerprint = @(foreach ($name in $savedFolders) {
                foreach ($entry in Get-SaveFingerprint (Join-Path $saveRoot $name)) {
                    [pscustomobject]@{
                        path = $name + '\' + $entry.path
                        hash = $entry.hash
                    }
                }
            })
            if (Compare-Object $saveFingerprint $restoredFingerprint -Property path, hash) {
                throw "Restored save files failed hash verification. Backup retained at $saveBackup"
            }

            $hostResult.saveRestored = $true
            foreach ($log in @('Player.log', 'Player-prev.log')) {
                $source = Join-Path $saveRoot $log
                if (Test-Path -LiteralPath $source) {
                    Copy-Item -LiteralPath $source -Destination $runPath
                }
            }

            $owmlLog = Get-ChildItem (Join-Path $OwmlPath 'Logs') -Filter 'OWML.Log.*' -File | Where-Object { $_.LastWriteTimeUtc -ge $launched } | Sort-Object LastWriteTimeUtc -Descending | Select-Object -First 1
            if ($owmlLog) {
                Copy-Item -LiteralPath $owmlLog.FullName -Destination (Join-Path $runPath 'OWML.log')
            }

            if (Test-Path -LiteralPath $requestPath) {
                $pending = Get-Content -LiteralPath $requestPath -Raw | ConvertFrom-Json
                if ($pending.id -eq $runId) {
                    Move-Item -LiteralPath $requestPath -Destination (Join-Path $runPath 'unclaimed.json')
                }
            }

            Write-AtomicJson (Join-Path $runPath 'host-result.json') $hostResult
            Write-Output "Result: $($hostResult.status); save restored: $($hostResult.saveRestored); artifacts: $runPath"
        }

        if ($failure) {
            throw $failure
        }
    }
}
finally {
    $controllerLock.Dispose()
}
