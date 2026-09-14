param(
    [string]$GamePath = 'C:\Program Files (x86)\Steam\steamapps\common\Outer Wilds',
    [string]$OwmlPath = (Join-Path $env:APPDATA 'OuterWildsModManager\OWML'),
    [string]$NotesPath = 'C:\Users\reese\Documents\Codex\ProjectNotes\outer-wilds-smart-autopilot',
    [switch]$ChecksOnly,
    [switch]$Release,
    [string]$PlaytestRecord
)

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest
. (Join-Path $PSScriptRoot 'ReleaseChecks.ps1')
if ($ChecksOnly -and $Release) {
    throw 'Release validation requires installed-game checks.'
}

$projectRoot    = $PSScriptRoot
$modProject     = Join-Path $projectRoot 'SmartAutopilot\SmartAutopilot.csproj'
$testProject    = Join-Path $projectRoot 'Navigation.Tests\Navigation.Tests.csproj'
$bindingProject = Join-Path $projectRoot 'Binding.Tests\Binding.Tests.csproj'
$perfProject    = Join-Path $projectRoot 'Performance.Tests\Performance.Tests.csproj'
$outputPath     = Join-Path $projectRoot 'SmartAutopilot\bin\Release\net48'
$artifactPath   = Join-Path $projectRoot 'artifacts'
$gameAssembly   = Join-Path $GamePath 'OuterWilds_Data\Managed\Assembly-CSharp.dll'
$modAssembly    = Join-Path $outputPath 'SmartAutopilot.dll'
$manifest       = Get-Content -LiteralPath (Join-Path $projectRoot 'SmartAutopilot\manifest.json') -Raw | ConvertFrom-Json
$config         = Get-Content -LiteralPath (Join-Path $projectRoot 'SmartAutopilot\default-config.json') -Raw | ConvertFrom-Json
$projectXml     = [xml](Get-Content -LiteralPath $modProject -Raw)
$owmlReference  = $projectXml.SelectSingleNode("/Project/ItemGroup/PackageReference[@Include='OWML']")
$validIdentity  = $manifest.version -match '^\d+\.\d+\.\d+$' -and $manifest.version -eq $projectXml.Project.PropertyGroup.Version `
    -and $manifest.uniqueName -eq 'Depthbomb.SmartAutopilot' -and $manifest.filename -eq 'SmartAutopilot.dll' `
    -and $manifest.owmlVersion -eq $owmlReference.Version
if (!$validIdentity) {
    throw 'Manifest identity, version, filename, or OWML dependency disagrees with the project.'
}

if ($config.settings.'Log detailed flight samples' -ne $false -or $config.settings.'Show navigation debug overlay' -ne $false) {
    throw 'Detailed logging and the debug overlay must default to off.'
}

foreach ($project in @($modProject, $testProject, $perfProject)) {
    dotnet build $project -c Release -p:RestoreLockedMode=true --nologo
    if ($LASTEXITCODE -ne 0) {
        throw "Build failed: $project"
    }
}

dotnet run --project $testProject -c Release --no-build
if ($LASTEXITCODE -ne 0) {
    throw 'Navigation checks failed.'
}

& (Join-Path $projectRoot 'Performance.Tests\bin\Release\net48\Performance.Tests.exe')
if ($LASTEXITCODE -ne 0) {
    throw 'Performance checks failed.'
}

& (Join-Path $projectRoot 'Release.Tests.ps1')

if ($ChecksOnly) {
    Write-Output 'Portable checks passed. Installed-game bindings and Unity playtests are still required; no package created.'

    return
}

dotnet run --project $testProject -c Release --no-build -- $gameAssembly $modAssembly
if ($LASTEXITCODE -ne 0) {
    throw 'Installed game member checks failed.'
}

dotnet build $bindingProject -c Release -p:RestoreLockedMode=true "-p:GamePath=$GamePath" --nologo
if ($LASTEXITCODE -ne 0) {
    throw 'Runtime binding test build failed.'
}

& (Join-Path $projectRoot 'Binding.Tests\bin\Release\net48\Binding.Tests.exe') $GamePath $modAssembly $OwmlPath
if ($LASTEXITCODE -ne 0) {
    throw 'Runtime binding checks failed.'
}

$assemblyVersion = [Reflection.AssemblyName]::GetAssemblyName($modAssembly).Version.ToString(3)
if ($assemblyVersion -ne $manifest.version) {
    throw 'Built DLL version disagrees with the package manifest.'
}

New-Item -ItemType Directory -Path $artifactPath -Force | Out-Null
$packagePath  = Join-Path $artifactPath ($manifest.uniqueName + '-' + $manifest.version + '.zip')
$packageFiles = @('SmartAutopilot.dll', 'SmartAutopilot.pdb', 'manifest.json', 'default-config.json', 'README.md', 'LICENSE')
Add-Type -AssemblyName System.IO.Compression
$archive = [IO.Compression.ZipArchive]::new([IO.File]::Open($packagePath, [IO.FileMode]::Create), [IO.Compression.ZipArchiveMode]::Create)
try {
    foreach ($name in $packageFiles) {
        $entry = $archive.CreateEntry($name, [IO.Compression.CompressionLevel]::Optimal)
        $entry.LastWriteTime = [DateTimeOffset]::new(2000, 1, 1, 0, 0, 0, [TimeSpan]::Zero)
        $inputStream  = [IO.File]::OpenRead((Join-Path $outputPath $name))
        $outputStream = $entry.Open()
        try {
            $inputStream.CopyTo($outputStream)
        }
        finally {
            $inputStream.Dispose()
            $outputStream.Dispose()
        }
    }
}
finally {
    $archive.Dispose()
}

$archive = [IO.Compression.ZipArchive]::new([IO.File]::OpenRead($packagePath), [IO.Compression.ZipArchiveMode]::Read)
$hasher  = [Security.Cryptography.SHA256]::Create()
try {
    if ($archive.Entries.Count -ne $packageFiles.Count) {
        throw 'Unexpected files in the package.'
    }

    foreach ($name in $packageFiles) {
        $entry = $archive.GetEntry($name)
        if ($null -eq $entry) {
            throw "Missing file in the package: $name"
        }

        $stream = $entry.Open()
        try {
            $entryHash = [BitConverter]::ToString($hasher.ComputeHash($stream)).Replace('-', '')
        }
        finally {
            $stream.Dispose()
        }

        $sourceHash = (Get-FileHash -LiteralPath (Join-Path $outputPath $name) -Algorithm SHA256).Hash
        if ($entryHash -ne $sourceHash) {
            throw "Packaged file differs from the verified build: $name"
        }
    }
}
finally {
    $hasher.Dispose()
    $archive.Dispose()
}

$packageHash = (Get-FileHash -LiteralPath $packagePath -Algorithm SHA256).Hash
$requiredChecks = Get-PlaytestCheckNames
New-Item -ItemType Directory -Path $NotesPath -Force | Out-Null
$templatePath = Join-Path $NotesPath ('playtest-' + $manifest.version + '-' + $packageHash.Substring(0, 12) + '.json')
if (!(Test-Path -LiteralPath $templatePath)) {
    $checks = [ordered]@{}
    foreach ($check in $requiredChecks) {
        $checks[$check] = $false
    }

    [ordered]@{
        version     = $manifest.version
        packageHash = $packageHash
        gameVersion = $manifest.minGameVersion
        testedBy    = ''
        testedAt    = ''
        checks      = $checks
        notes       = 'Pending in-game playtest. Record actual observations before marking any check passed.'
    } | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath $templatePath -Encoding utf8
}

Write-Output "Package: $packagePath"
Write-Output "SHA256: $packageHash"
Write-Output "Playtest record: $templatePath"
if (!$Release) {
    & (Join-Path $projectRoot 'InstallPlaytest.ps1') -BuildPath $outputPath -OwmlPath $OwmlPath -NotesPath $NotesPath
    Write-Output 'Prepared for playtesting. Public release validation remains pending.'

    return
}

Assert-PlaytestRecord -Path $PlaytestRecord -PackageHash $packageHash -Version $manifest.version -GameVersion $manifest.minGameVersion

& (Join-Path $projectRoot 'InstallPlaytest.ps1') -BuildPath $outputPath -OwmlPath $OwmlPath -NotesPath $NotesPath

Write-Output 'Release validation passed. This command does not publish the package.'
