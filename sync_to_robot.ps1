# Send changed robot files (root *.sh and src/) to the Raspberry Pi over ssh.
# src/ServoCalibration.py is never sent: the robot rewrites it in calibration mode.
#   .\sync_to_robot.ps1            sync
#   .\sync_to_robot.ps1 -DryRun    list changed files only
param(
    [string]$RobotHost = "pi@192.168.0.190",
    [string]$RemoteDir = "PrintPupper",
    [switch]$DryRun
)
$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot

$excluded = @("src/ServoCalibration.py")

$files = @(Get-ChildItem -File -Filter *.sh) + @(Get-ChildItem src -File -Recurse) |
    Where-Object { $_.FullName -notmatch '\\__pycache__\\' } |
    ForEach-Object { $_.FullName.Substring($PSScriptRoot.Length + 1).Replace('\', '/') } |
    Where-Object { $_ -notin $excluded }

$localHash = @{}
foreach ($f in $files) {
    $localHash[$f] = (Get-FileHash $f -Algorithm MD5).Hash.ToLower()
}

$remoteHash = @{}
# Windows PowerShell 5.1 may prepend a BOM and uses CRLF when piping to ssh
$files | ssh $RobotHost "cd $RemoteDir && sed '1s/^\xEF\xBB\xBF//; s/\r$//' | xargs -d '\n' md5sum 2>/dev/null; true" |
    ForEach-Object {
        $hash, $path = $_ -split '\s+', 2
        $remoteHash[$path] = $hash
    }
if ($LASTEXITCODE -ne 0) { throw "ssh to $RobotHost failed" }

$changed = @($files | Where-Object { $localHash[$_] -ne $remoteHash[$_] })
if ($changed.Count -eq 0) {
    Write-Host "Already up to date."
    exit 0
}
$changed | ForEach-Object { Write-Host "  $_" }
if ($DryRun) {
    Write-Host "$($changed.Count) file(s) would be sent (dry run)."
    exit 0
}

$listFile = New-TemporaryFile
try {
    [IO.File]::WriteAllText($listFile, ($changed -join "`n") + "`n", [Text.UTF8Encoding]::new($false))
    # cmd's pipe keeps the tar stream binary-safe
    cmd /c "tar --format=ustar -cf - -T `"$listFile`" | ssh $RobotHost `"tar -xmf - -C $RemoteDir && chmod +x $RemoteDir/*.sh $RemoteDir/src/test/*.sh`""
    if ($LASTEXITCODE -ne 0) { throw "transfer to $RobotHost failed" }
} finally {
    Remove-Item $listFile
}
Write-Host "$($changed.Count) file(s) sent."
