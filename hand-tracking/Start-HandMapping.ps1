[CmdletBinding()]
param(
    [ValidateRange(0,10)][int]$Camera = 0,
    [ValidateSet('auto','dshow','msmf')][string]$Backend = 'auto',
    [ValidateRange(1024,65535)][int]$Port = 8090,
    [switch]$NoMirror,
    [switch]$SwapHandedness,
    [switch]$NoBrowser,
    [switch]$SdkMonitor,
    [switch]$IndexFollow,
    [switch]$FullHandFollow,
    [ValidateSet('mediapipe','wilor','d435','wilor-d435')][string]$Tracker='mediapipe',
    [string]$DepthSerial
)
$ErrorActionPreference='Stop'
# Small vision/matrix tasks run beside MediaPipe and CUDA; large CPU pools
# compete for cores and make end-to-end latency worse on hybrid-core laptops.
$env:OPENBLAS_NUM_THREADS='1'
$env:MKL_NUM_THREADS='1'
$env:NUMEXPR_NUM_THREADS='1'
$env:OMP_NUM_THREADS='2'
if (-not (Get-Command uv -ErrorAction SilentlyContinue)) { throw 'uv is not on PATH.' }
Push-Location $PSScriptRoot
try {
    & uv sync --frozen
    if ($LASTEXITCODE -ne 0) { throw 'uv sync failed.' }
    if (-not (Test-Path -LiteralPath (Join-Path $PSScriptRoot 'models\hand_landmarker.task'))) {
        & uv run --frozen python -m hand_preview --download-model
        if ($LASTEXITCODE -ne 0) { throw 'Model download failed.' }
    }
    $mappingArgs=@('--camera',"$Camera",'--backend',$Backend,'--port',"$Port")
    $mappingArgs+=@('--tracker',$Tracker)
    if($DepthSerial){$mappingArgs+=@('--depth-serial',$DepthSerial)}
    if ($NoMirror) { $mappingArgs+='--no-mirror' }
    if ($SwapHandedness) { $mappingArgs+='--swap-handedness' }
    if ($NoBrowser) { $mappingArgs+='--no-browser' }
    if ($SdkMonitor) { $mappingArgs+='--sdk-monitor' }
    if ($IndexFollow) { $mappingArgs+='--index-follow' }
    if ($FullHandFollow) { $mappingArgs+='--full-hand-follow' }
    & uv run --frozen python -m hand_preview.mapping_app @mappingArgs
    if ($LASTEXITCODE -ne 0) { throw "Mapping preview exited with code $LASTEXITCODE" }
} finally { Pop-Location }
