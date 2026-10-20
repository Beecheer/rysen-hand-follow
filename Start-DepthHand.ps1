[CmdletBinding()]
param([switch]$NoBrowser,[switch]$Restart,[string]$Serial)
$ErrorActionPreference='Stop'
& (Join-Path $PSScriptRoot 'Start-DepthCamera.ps1') -Hands -NoBrowser:$NoBrowser -Restart:$Restart -Serial $Serial
