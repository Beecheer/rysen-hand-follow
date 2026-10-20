$ErrorActionPreference='Stop'
$env:UV_CACHE_DIR=Join-Path (Split-Path $PSScriptRoot) '.uv-cache'
$env:UV_PYTHON_INSTALL_DIR=Join-Path $PSScriptRoot '.python'
$env:HF_HOME=Join-Path $PSScriptRoot '.hf-cache'
$env:YOLO_CONFIG_DIR=Join-Path $PSScriptRoot 'models\yolo-config'
$env:OMP_NUM_THREADS='2'
$env:OPENBLAS_NUM_THREADS='1'
$env:MKL_NUM_THREADS='1'
$env:NUMEXPR_NUM_THREADS='1'
Push-Location $PSScriptRoot
try {
    & uv run --frozen python server.py
    if($LASTEXITCODE -ne 0){throw 'WiLoR backend exited with an error; see logs.'}
} finally {Pop-Location}
