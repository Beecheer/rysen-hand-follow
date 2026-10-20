[CmdletBinding()]
param(
    [ValidateRange(0, 10)][int]$Camera = 0,
    [ValidateSet('auto','dshow','msmf')][string]$Backend = 'auto',
    [switch]$NoMirror,
    [switch]$SwapHandedness
)
$ErrorActionPreference = 'Stop'
if (-not (Get-Command uv -ErrorAction SilentlyContinue)) { throw 'uv is not on PATH.' }
Push-Location $PSScriptRoot
try {
    & uv sync --frozen
    if ($LASTEXITCODE -ne 0) { throw 'uv sync failed. Check the error above.' }
    if (-not (Test-Path -LiteralPath (Join-Path $PSScriptRoot 'models\hand_landmarker.task'))) {
        & uv run --frozen python -m hand_preview --download-model
        if ($LASTEXITCODE -ne 0) { throw 'Model download failed. Check network/proxy access to storage.googleapis.com.' }
    }
    $previewArgs = @('--camera', "$Camera", '--backend', $Backend)
    if ($NoMirror) { $previewArgs += '--no-mirror' }
    if ($SwapHandedness) { $previewArgs += '--swap-handedness' }
    & uv run --frozen python -m hand_preview @previewArgs
    if ($LASTEXITCODE -ne 0) { throw "Preview exited with code $LASTEXITCODE" }
} finally { Pop-Location }
