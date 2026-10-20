[CmdletBinding()]
param([switch]$NoBrowser,[string]$Video,[switch]$SwapWiLoR)
$ErrorActionPreference='Stop'
if($Video){$Video=(Resolve-Path -LiteralPath $Video).Path}
$origin='http://127.0.0.1:8090'
$existing=$null
$reuseCamera=$false
try {$existing=Invoke-RestMethod "$origin/api/status" -TimeoutSec 2} catch {}
if($existing){
    if($existing.comparison.enabled){
        if($Video -or $SwapWiLoR){throw 'Stop the existing comparison before changing replay or handedness options'}
        $reuseCamera=$true
    }
    if($existing.robot_connected -ne $false -or $existing.hardware -or $existing.index_follow){
        throw 'An SDK session is running. Stop it before starting camera-only comparison.'
    }
    if(-not $reuseCamera){& (Join-Path $PSScriptRoot 'hand-tracking\Stop-HandMapping.ps1')}
}
$backend=Join-Path $PSScriptRoot 'wilor-backend'
$env:UV_CACHE_DIR=Join-Path $PSScriptRoot '.uv-cache'
$env:UV_PYTHON_INSTALL_DIR=Join-Path $backend '.python'
if(-not(Test-Path -LiteralPath (Join-Path $backend 'wheels\torch-2.5.0+cu121-cp310-cp310-win_amd64.whl'))){
    & uv run --project (Join-Path $PSScriptRoot 'hand-tracking') --frozen python (Join-Path $backend 'download_models.py') --torch
    if($LASTEXITCODE -ne 0){throw 'CUDA wheel download failed; configure a process HTTP proxy and retry'}
}
Push-Location $backend
try {& uv sync --frozen;if($LASTEXITCODE -ne 0){throw 'WiLoR uv sync failed'}} finally {Pop-Location}
$health=$null
try {$health=Invoke-RestMethod 'http://127.0.0.1:8093/health' -TimeoutSec 2} catch {}
if($health -and $health.service -ne 'rysen-wilor-inference'){throw 'Unexpected service on port 8093'}
if(-not $health){
    $logs=Join-Path $backend 'logs';$null=New-Item -ItemType Directory -Path $logs -Force
    Start-Process pwsh -ArgumentList @('-NoProfile','-File',(Join-Path $backend 'Start-Backend.ps1')) -WindowStyle Hidden -RedirectStandardOutput (Join-Path $logs 'backend.stdout.log') -RedirectStandardError (Join-Path $logs 'backend.stderr.log')
    $deadline=[DateTime]::UtcNow.AddSeconds(30)
    do{
        try{$health=Invoke-RestMethod 'http://127.0.0.1:8093/health' -TimeoutSec 2}catch{}
        if(-not $health){Start-Sleep -Milliseconds 300}
    }while(-not $health -and [DateTime]::UtcNow -lt $deadline)
    if(-not $health){throw 'WiLoR backend did not start; see wilor-backend/logs'}
}
if($health.service -ne 'rysen-wilor-inference'){throw 'Unexpected backend identity'}
Write-Host 'Camera-only comparison. Initial model download/loading is shown in the page.'
if($reuseCamera){
    Write-Host "Comparison camera session reused: $origin"
    if(-not $NoBrowser){Start-Process $origin}
    return
}
Push-Location (Join-Path $PSScriptRoot 'hand-tracking')
try {
    & uv sync --frozen
    if($LASTEXITCODE -ne 0){throw 'Camera environment uv sync failed'}
    $arguments=@('run','--frozen','python','-m','hand_preview.mapping_app','--compare-wilor')
    if($SwapWiLoR){$arguments+='--wilor-swap-handedness'}
    if($Video){$arguments+=@('--compare-video',(Resolve-Path -LiteralPath $Video).Path)}
    if($NoBrowser){$arguments+='--no-browser'}
    & uv @arguments
    if($LASTEXITCODE -ne 0){throw 'Comparison preview exited with error'}
}finally {Pop-Location}
