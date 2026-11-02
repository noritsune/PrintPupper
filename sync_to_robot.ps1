# Send changed robot files (root *.sh and src/) to the Raspberry Pi over ssh.
# src/ServoCalibration.py is never sent: the robot rewrites it in calibration mode.
#   .\sync_to_robot.ps1            sync
#   .\sync_to_robot.ps1 -DryRun    list changed files only
#   .\sync_to_robot.ps1 -PullPolicy  copy the RL policy from $PolicySource into src/rl first
#   .\sync_to_robot.ps1 -NoRestart   do not restart robot.service after sending
#   .\sync_to_robot.ps1 -RobotHost pi@192.168.0.59   use this host without asking
# After files are sent, robot.service is restarted so the new code is loaded.
# The robot host is asked on the first run and saved to .robot_host (not committed).
# Later runs show the saved host and ask whether to keep it (Enter) or type a new one.
param(
    [string]$RobotHost = "",
    [string]$RemoteDir = "PrintPupper",
    [switch]$DryRun,
    [switch]$PullPolicy,
    [switch]$NoRestart,
    [string]$PolicySource = "\\wsl.localhost\Ubuntu\root\pupper_rl\deploy\policy_u_stand.npz"
)
$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot

$hostFile = Join-Path $PSScriptRoot ".robot_host"
if (-not $RobotHost) {
    $saved = if (Test-Path -LiteralPath $hostFile) { (Get-Content -LiteralPath $hostFile -Raw).Trim() } else { "" }
    if ($saved) {
        $answer = (Read-Host "Robot host [$saved] (Enter to keep, or type a new one)").Trim()
        $RobotHost = if ($answer) { $answer } else { $saved }
    } else {
        $RobotHost = (Read-Host "Robot host (e.g. pi@192.168.0.59)").Trim()
    }
}
if ($RobotHost -notmatch '^[\w.-]+@[\w.-]+$') { throw "robot host must look like user@host: '$RobotHost'" }
[IO.File]::WriteAllText($hostFile, "$RobotHost`n", [Text.UTF8Encoding]::new($false))
Write-Host "Robot host: $RobotHost"

if ($PullPolicy) {
    $policyInRepo = "src/rl/policy_u_stand.npz"
    if (-not (Test-Path -LiteralPath $PolicySource -PathType Leaf)) { throw "policy not found: $PolicySource" }
    if ($DryRun) {
        Write-Host "Would copy policy: $PolicySource -> $policyInRepo"
    } else {
        Copy-Item -LiteralPath $PolicySource $policyInRepo
        Write-Host "Policy copied: $PolicySource -> $policyInRepo"
    }
}

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
    # cmd's pipe keeps the tar stream binary-safe.
    # sync flushes to the SD card: cutting robot power within ~30 s otherwise leaves 0-byte files.
    cmd /c "tar --format=ustar -cf - -T `"$listFile`" | ssh $RobotHost `"tar -xmf - -C $RemoteDir && chmod +x $RemoteDir/*.sh $RemoteDir/src/test/*.sh && sync`""
    if ($LASTEXITCODE -ne 0) { throw "transfer to $RobotHost failed" }
} finally {
    Remove-Item $listFile
}
Write-Host "$($changed.Count) file(s) sent."

if (-not $NoRestart) {
    ssh $RobotHost "sudo systemctl restart robot && systemctl is-active robot"
    if ($LASTEXITCODE -ne 0) { throw "restart of robot.service on $RobotHost failed" }
    Write-Host "robot.service restarted."
}
