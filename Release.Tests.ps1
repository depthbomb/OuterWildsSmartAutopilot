$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest
. (Join-Path $PSScriptRoot 'ReleaseChecks.ps1')

$testFile = New-TemporaryFile
try {
    $checks = [ordered]@{}
    foreach ($check in Get-PlaytestCheckNames) {
        $checks[$check] = $true
    }

    $record = [ordered]@{
        version     = '0.1.0'
        packageHash = 'synthetic-test-hash'
        gameVersion = '1.1.16.1372'
        testedBy    = 'Synthetic unit test; not a real playtest'
        testedAt    = '2000-01-01'
        checks      = $checks
    }
    $record | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath $testFile.FullName
    Assert-PlaytestRecord -Path $testFile.FullName -PackageHash 'synthetic-test-hash' -Version '0.1.0' -GameVersion '1.1.16.1372'
    foreach ($failure in @('missing-file', 'wrong-hash', 'unchecked', 'string-true', 'missing-check', 'missing-tester')) {
        $candidate = $record | ConvertTo-Json -Depth 5 | ConvertFrom-Json
        $testPath = $testFile.FullName
        switch ($failure) {
            'missing-file' {
                $testPath = $testFile.FullName + '.missing'
            }
            'wrong-hash' {
                $candidate.packageHash = 'different-package'
            }
            'unchecked' {
                $candidate.checks.'debug-overlay' = $false
            }
            'string-true' {
                $candidate.checks.'debug-overlay' = 'true'
            }
            'missing-check' {
                $candidate.checks.PSObject.Properties.Remove('debug-overlay')
            }
            'missing-tester' {
                $candidate.testedBy = ''
            }
        }

        $candidate | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath $testFile.FullName
        $rejected = $false
        try {
            Assert-PlaytestRecord -Path $testPath -PackageHash 'synthetic-test-hash' -Version '0.1.0' -GameVersion '1.1.16.1372'
        }
        catch {
            $rejected = $true
        }

        if (!$rejected) {
            throw "Release validation accepted invalid test case: $failure"
        }
    }
}
finally {
    Remove-Item -LiteralPath $testFile.FullName
}

Write-Output 'PASS: release validation accepts a complete synthetic record and rejects six invalid record cases. No real playtest was recorded.'
