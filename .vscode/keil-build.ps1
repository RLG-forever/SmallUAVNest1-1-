param(
    [ValidateSet('Build', 'Rebuild', 'Clean')]
    [string]$Action = 'Build'
)

$ErrorActionPreference = 'Stop'
[Console]::OutputEncoding = [System.Text.Encoding]::UTF8

$workspaceRoot = Split-Path -Parent $PSScriptRoot
$projectPath = Join-Path $workspaceRoot 'USER\EC17001-FCU71-24CAN.uvprojx'
$logPath = Join-Path $workspaceRoot 'USER\vscode_keil_build.log'

if ($env:KEIL_UV4_PATH) {
    $uv4Path = $env:KEIL_UV4_PATH
} else {
    $uv4Path = 'C:\Keil_v5\UV4\UV4.exe'
}

if (-not (Test-Path -LiteralPath $uv4Path -PathType Leaf)) {
    Write-Error "Keil UV4.exe not found: $uv4Path. Install Keil or set KEIL_UV4_PATH."
    exit 2
}

if (-not (Test-Path -LiteralPath $projectPath -PathType Leaf)) {
    Write-Error "Keil project not found: $projectPath"
    exit 2
}

$actionFlag = switch ($Action) {
    'Build'   { '-b' }
    'Rebuild' { '-r' }
    'Clean'   { '-c' }
}

Write-Host "Keil $Action`: $projectPath"
if (Test-Path -LiteralPath $logPath -PathType Leaf) {
    Remove-Item -LiteralPath $logPath -Force
}
& $uv4Path $actionFlag $projectPath '-j0' '-o' $logPath

$logDeadline = [DateTime]::UtcNow.AddSeconds(30)
$logText = $null
do {
    if (Test-Path -LiteralPath $logPath -PathType Leaf) {
        $logText = Get-Content -LiteralPath $logPath -Encoding Default -Raw
        if ($logText -match '\d+\s+Error\(s\)') {
            break
        }
    }
    Start-Sleep -Milliseconds 100
} while ([DateTime]::UtcNow -lt $logDeadline)

if ([string]::IsNullOrWhiteSpace($logText)) {
    Write-Error "Keil did not create the build log: $logPath"
    exit 2
}

Write-Output $logText

$errorCount = 0
$summaryMatches = [regex]::Matches($logText, '(\d+)\s+Error\(s\)')
if ($summaryMatches.Count -gt 0) {
    $errorCount = [int]$summaryMatches[$summaryMatches.Count - 1].Groups[1].Value
}

# Some UV4 versions return 1 even after a successful build, so use the
# final Error(s) count in the Keil log as the authoritative result.
if ($summaryMatches.Count -eq 0) {
    exit 1
}
if ($errorCount -ne 0) {
    exit 1
}

exit 0
