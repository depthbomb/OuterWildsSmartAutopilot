param(
    [string]$Configuration = 'Release'
)

$ErrorActionPreference = 'Stop'
$artifacts = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot 'artifacts'))
$staging = Join-Path $artifacts ('.package-' + [guid]::NewGuid().ToString('N'))
$manifest = Get-Content -LiteralPath (Join-Path $PSScriptRoot 'SmartAutopilot\manifest.json') -Raw | ConvertFrom-Json
$name = "$($manifest.uniqueName)-$($manifest.version).zip"
$package = Join-Path $artifacts $name
New-Item -ItemType Directory -Path $staging -Force | Out-Null

try {
    dotnet build (Join-Path $PSScriptRoot 'SmartAutopilot\SmartAutopilot.csproj') -c $Configuration "-p:OutputPath=$staging" -p:RestoreLockedMode=true --nologo
    if ($LASTEXITCODE -ne 0) {
        throw 'Mod build failed.'
    }

    $files = @('SmartAutopilot.dll', 'SmartAutopilot.pdb', 'manifest.json', 'default-config.json', 'README.md', 'LICENSE')
    Compress-Archive -LiteralPath @($files | ForEach-Object { Join-Path $staging $_ }) -DestinationPath $package -Force
    $hash = (Get-FileHash -LiteralPath $package -Algorithm SHA256).Hash.ToLowerInvariant()
    [IO.File]::WriteAllText($package + '.sha256', "$hash  $name`n", [Text.UTF8Encoding]::new($false))
    Write-Output "Package: $package"
}
finally {
    $resolved = [IO.Path]::GetFullPath($staging)
    if ((Split-Path $resolved -Parent) -ne $artifacts) {
        throw 'Unexpected package staging path.'
    }

    Remove-Item -LiteralPath $resolved -Recurse -Force
}
