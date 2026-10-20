param([string]$Serial,[switch]$Hands)
$ErrorActionPreference='Stop'
$env:UV_CACHE_DIR=Join-Path (Split-Path $PSScriptRoot) '.uv-cache'
Push-Location $PSScriptRoot
try{
 $arguments=@('run','--frozen','python','depth_camera.py')
 if($Serial){$arguments+=@('--serial',$Serial)}
 if($Hands){$arguments+='--hands'}
 & uv @arguments
 if($LASTEXITCODE -ne 0){throw 'Depth camera process failed.'}
}finally{Pop-Location}
