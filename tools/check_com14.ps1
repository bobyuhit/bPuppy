# 检查 COM14 (CH343) 设备状态与错误码
$ErrorActionPreference = 'SilentlyContinue'
$target = 'USB\VID_1A86&PID_55D3\5C4C203934'

Write-Output "=== COM14 ($target) ==="
$all = Get-CimInstance Win32_PnPEntity
$dev = $all | Where-Object { $_.DeviceID -eq $target }
if ($dev) {
    $dev | Select-Object Name, Status, ConfigManagerErrorCode, Service | Format-List
} else {
    Write-Output "未找到 (已消失/未枚举)"
}

Write-Output "=== 全部 CH343 (VID_1A86 PID_55D3) ==="
$all | Where-Object { $_.Name -like '*CH343*' } |
    Select-Object Name, Status, ConfigManagerErrorCode, DeviceID | Format-Table -AutoSize
