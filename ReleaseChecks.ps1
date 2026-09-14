function Get-PlaytestCheckNames {
    return @('launch-surfaces', 'cancel-reengage', 'navigation-arrival', 'physical-obstacles', 'blocked-holds',
        'damage-fuel', 'target-changes', 'keyboard-controller-camera', 'loop-scene-cleanup', 'debug-overlay', 'gameplay-performance')
}

function Assert-PlaytestRecord {
    param(
        [string]$Path,
        [string]$PackageHash,
        [string]$Version,
        [string]$GameVersion
    )

    if (!$Path -or !(Test-Path -LiteralPath $Path -PathType Leaf)) {
        throw 'Release validation requires a completed in-game playtest record for this package.'
    }

    $record = Get-Content -LiteralPath $Path -Raw | ConvertFrom-Json
    $matches = $record.packageHash -eq $PackageHash -and $record.version -eq $Version `
        -and $record.gameVersion -eq $GameVersion -and ![string]::IsNullOrWhiteSpace($record.testedBy) `
        -and ![string]::IsNullOrWhiteSpace($record.testedAt)
    if (!$matches) {
        throw 'Playtest record does not identify this exact package and supported game build, or lacks tester details.'
    }

    foreach ($check in Get-PlaytestCheckNames) {
        $property = $record.checks.PSObject.Properties[$check]
        if ($null -eq $property -or $property.Value -isnot [bool] -or $property.Value -ne $true) {
            throw "Required in-game check is incomplete: $check"
        }
    }
}
