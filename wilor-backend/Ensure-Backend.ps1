$ErrorActionPreference='Stop'
$env:UV_CACHE_DIR=Join-Path (Split-Path $PSScriptRoot) '.uv-cache'
$env:UV_PYTHON_INSTALL_DIR=Join-Path $PSScriptRoot '.python'
Push-Location $PSScriptRoot
try {& uv sync --frozen;if($LASTEXITCODE -ne 0){throw 'WiLoR environment sync failed'}}finally{Pop-Location}
$health=$null
try{$health=Invoke-RestMethod 'http://127.0.0.1:8093/health' -TimeoutSec 2}catch{}
if($health -and $health.service -ne 'rysen-wilor-inference'){throw 'Unexpected service on port 8093'}
if(-not $health){
    $logs=Join-Path $PSScriptRoot 'logs';$null=New-Item -ItemType Directory -Path $logs -Force
    Start-Process pwsh -ArgumentList @('-NoProfile','-File',(Join-Path $PSScriptRoot 'Start-Backend.ps1')) -WindowStyle Hidden -RedirectStandardOutput (Join-Path $logs 'backend.stdout.log') -RedirectStandardError (Join-Path $logs 'backend.stderr.log')
}
$deadline=[DateTime]::UtcNow.AddSeconds(120)
do{
    try{$health=Invoke-RestMethod 'http://127.0.0.1:8093/health' -TimeoutSec 2}catch{$health=$null}
    if($health.error){throw "WiLoR load failed: $($health.error)"}
    if($health.ready -and $health.service -eq 'rysen-wilor-inference'){Write-Host 'WiLoR GPU backend ready.';return}
    Start-Sleep -Milliseconds 400
}while([DateTime]::UtcNow -lt $deadline)
throw 'WiLoR GPU backend did not become ready; device was not enabled.'
