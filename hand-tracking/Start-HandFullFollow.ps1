[CmdletBinding()]
param([switch]$NoBrowser,[ValidateSet('mediapipe','wilor','d435','wilor-d435')][string]$Tracker='mediapipe',[string]$DepthSerial)
& (Join-Path $PSScriptRoot 'Start-HandIndexFollow.ps1') -FullHand -NoBrowser:$NoBrowser -Tracker $Tracker -DepthSerial $DepthSerial
