param(
    [Parameter(Mandatory = $true)]
    [string]$BuildPath,
    [Parameter(Mandatory = $true)]
    [string]$OwmlPath,
    [Parameter(Mandatory = $true)]
    [string]$NotesPath
)

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest
$running = Get-Process -Name 'OuterWilds', 'OWML.Launcher' -ErrorAction SilentlyContinue
if ($running) {
    throw 'Close Outer Wilds and the OWML launcher before installing the playtest build. The package is already prepared.'
}

$launcherPath = Join-Path $OwmlPath 'OWML.Launcher.exe'
if (!(Test-Path -LiteralPath $launcherPath -PathType Leaf)) {
    throw "OWML launcher was not found at $launcherPath"
}

$manifest = Get-Content -LiteralPath (Join-Path $BuildPath 'manifest.json') -Raw | ConvertFrom-Json
if ($manifest.uniqueName -ne 'Depthbomb.SmartAutopilot' -or $manifest.filename -ne 'SmartAutopilot.dll') {
    throw 'The build does not identify the expected mod.'
}

$modsRoot = [IO.Path]::GetFullPath((Join-Path $OwmlPath 'Mods'))
New-Item -ItemType Directory -Path $modsRoot -Force | Out-Null
$installations = @(
    foreach ($folder in Get-ChildItem -LiteralPath $modsRoot -Directory) {
        $manifestPath = Join-Path $folder.FullName 'manifest.json'
        if (Test-Path -LiteralPath $manifestPath -PathType Leaf) {
            $installedManifest = Get-Content -LiteralPath $manifestPath -Raw | ConvertFrom-Json
            if ($installedManifest.uniqueName -eq $manifest.uniqueName) {
                $folder.FullName
            }
        }
    }
)
if ($installations.Count -gt 1) {
    throw 'More than one installation of this mod exists in OWML. Resolve the duplicate before playtesting.'
}

$installPath = if ($installations.Count -eq 1) { $installations[0] } else { Join-Path $modsRoot $manifest.uniqueName }
$installPath = [IO.Path]::GetFullPath($installPath)
if (!$installPath.StartsWith($modsRoot.TrimEnd('\') + '\', [StringComparison]::OrdinalIgnoreCase)) {
    throw 'The installation path is outside the configured OWML Mods directory.'
}

New-Item -ItemType Directory -Path $installPath -Force | Out-Null
$installDirectory = Get-Item -LiteralPath $installPath
if (($installDirectory.Attributes -band [IO.FileAttributes]::ReparsePoint) -ne 0) {
    throw 'The installation directory is a link; use an ordinary OWML mod directory.'
}

$defaults   = Get-Content -LiteralPath (Join-Path $BuildPath 'default-config.json') -Raw | ConvertFrom-Json
$configPath = Join-Path $installPath 'config.json'
$config = if (Test-Path -LiteralPath $configPath) { Get-Content -LiteralPath $configPath -Raw | ConvertFrom-Json } else { $defaults }
if (!$config.PSObject.Properties['settings']) {
    $config | Add-Member -NotePropertyName 'settings' -NotePropertyValue ([pscustomobject]@{})
}

foreach ($setting in $defaults.settings.PSObject.Properties) {
    if (!$config.settings.PSObject.Properties[$setting.Name]) {
        $config.settings | Add-Member -NotePropertyName $setting.Name -NotePropertyValue $setting.Value
    }
}

$config | Add-Member -NotePropertyName 'enabled' -NotePropertyValue $true -Force
$packageFiles = @('SmartAutopilot.dll', 'SmartAutopilot.pdb', 'manifest.json', 'default-config.json', 'README.md', 'LICENSE')
$installFiles = @($packageFiles) + 'config.json'
$backupPath = Join-Path $NotesPath ('playtest-install-backups\' + [DateTime]::UtcNow.ToString('yyyyMMdd-HHmmss-fffffff'))
New-Item -ItemType Directory -Path $backupPath -Force | Out-Null
foreach ($name in $installFiles) {
    $existingPath = Join-Path $installPath $name
    if (Test-Path -LiteralPath $existingPath -PathType Leaf) {
        Copy-Item -LiteralPath $existingPath -Destination (Join-Path $backupPath $name)
    }
}

$running = Get-Process -Name 'OuterWilds', 'OWML.Launcher' -ErrorAction SilentlyContinue
if ($running) {
    throw 'The game or OWML launcher started during preparation. Close it before installing the prepared package.'
}

try {
    foreach ($name in $packageFiles) {
        $sourcePath = Join-Path $BuildPath $name
        $targetPath = Join-Path $installPath $name
        Copy-Item -LiteralPath $sourcePath -Destination $targetPath -Force
        if ((Get-FileHash -LiteralPath $sourcePath -Algorithm SHA256).Hash -ne (Get-FileHash -LiteralPath $targetPath -Algorithm SHA256).Hash) {
            throw "Installed file verification failed: $name"
        }
    }

    $config | ConvertTo-Json -Depth 20 | Set-Content -LiteralPath $configPath -Encoding utf8
    $installedConfig = Get-Content -LiteralPath $configPath -Raw | ConvertFrom-Json
    if ($installedConfig.enabled -ne $true) {
        throw 'The installed mod is not enabled.'
    }
}
catch {
    foreach ($name in $installFiles) {
        $backupFile = Join-Path $backupPath $name
        $targetPath = Join-Path $installPath $name
        if (Test-Path -LiteralPath $backupFile -PathType Leaf) {
            Copy-Item -LiteralPath $backupFile -Destination $targetPath -Force
        }
        elseif (Test-Path -LiteralPath $targetPath -PathType Leaf) {
            Remove-Item -LiteralPath $targetPath
        }
    }

    throw
}

Write-Output "Installed and enabled Smart Autopilot $($manifest.version): $installPath"
Write-Output 'Installed files match the verified package inputs. Existing settings were preserved and new settings were added.'
Write-Output 'Ready to playtest: launch Outer Wilds through the mod manager.'
