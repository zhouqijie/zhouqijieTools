# 分发器 - 按配置文件将源目录内容拷贝到目标目录
# 用法: .\distributor.ps1 [-Config <配置文件路径>]
#       默认使用脚本同级目录下的 distributor.conf

param(
    [string]$Config = $null
)

$ErrorActionPreference = "Stop"

# 确定配置文件路径（默认与脚本同级）
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
if (-not $Config) {
    $Config = Join-Path $scriptDir "distributor.conf"
}

if (-not (Test-Path $Config)) {
    Write-Host "[ERROR] 配置文件不存在: $Config" -ForegroundColor Red
    exit 1
}

Write-Host "========================================" -ForegroundColor Cyan
Write-Host "  分发器" -ForegroundColor Cyan
Write-Host "  配置文件: $Config" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan

# 解析 INI 风格配置文件
function Parse-Config {
    param([string]$Path)

    $content = Get-Content $Path -Encoding UTF8
    $result = @{}
    $currentSection = $null

    foreach ($line in $content) {
        $trimmed = $line.Trim()
        # 跳过空行和注释
        if ($trimmed -eq "" -or $trimmed.StartsWith("#") -or $trimmed.StartsWith(";")) {
            continue
        }
        # 匹配节头 [Section]
        if ($trimmed -match '^\[(.+)\]$') {
            $currentSection = $Matches[1]
            if (-not $result.ContainsKey($currentSection)) {
                $result[$currentSection] = @()
            }
        }
        elseif ($currentSection -ne $null) {
            $result[$currentSection] += $trimmed
        }
    }

    return $result
}

# 解析路径中的特殊前缀
# /path/to/dir  →  C:\Users\<当前用户>\path\to\dir
# ./path/to/dir →  脚本同级目录下的 path\to\dir
# 其他          →  保持原样（绝对路径等）
function Resolve-PathEx {
    param([string]$Path, [string]$BaseDir)

    $p = $Path.TrimEnd('/', '\')

    if ($p.StartsWith("/")) {
        # / 前缀 → 用户目录
        $relative = $p.Substring(1)
        $relative = $relative -replace '/', '\'
        return Join-Path $env:USERPROFILE $relative
    }
    elseif ($p.StartsWith("./")) {
        # ./ 前缀 → 脚本同级目录
        $relative = $p.Substring(2)
        $relative = $relative -replace '/', '\'
        return Join-Path $BaseDir $relative
    }
    else {
        return $p
    }
}

# 解析配置
$configData = Parse-Config -Path $Config

# 验证必填节
if (-not $configData.ContainsKey("Src")) {
    Write-Host "[ERROR] 配置文件中缺少 [Src] 节" -ForegroundColor Red
    exit 1
}
if (-not $configData.ContainsKey("DST")) {
    Write-Host "[ERROR] 配置文件中缺少 [DST] 节" -ForegroundColor Red
    exit 1
}

# 解析所有 Src 和 DST 路径
$srcPaths = $configData["Src"] | ForEach-Object { Resolve-PathEx -Path $_ -BaseDir $scriptDir }
$dstPaths = $configData["DST"] | ForEach-Object { Resolve-PathEx -Path $_ -BaseDir $scriptDir }

# 显示即将执行的任务
Write-Host ""
Write-Host "源目录 ($($srcPaths.Count)):" -ForegroundColor Yellow
$srcPaths | ForEach-Object { Write-Host "  $_" }
Write-Host ""
Write-Host "目标目录 ($($dstPaths.Count)):" -ForegroundColor Yellow
$dstPaths | ForEach-Object { Write-Host "  $_" }

# 验证所有源目录
$hasSrcError = $false
foreach ($sp in $srcPaths) {
    if (-not (Test-Path $sp)) {
        Write-Host "[ERROR] 源目录不存在: $sp" -ForegroundColor Red
        $hasSrcError = $true
    }
}
if ($hasSrcError) { exit 1 }

# 拷贝任务执行函数
function Invoke-CopyTask {
    param([string]$Src, [string]$Dst)

    if (-not (Test-Path $Dst)) {
        New-Item -ItemType Directory -Path $Dst -Force | Out-Null
    }

    $args = @(
        $Src, $Dst,
        "/E", "/COPY:DAT", "/DCOPY:DAT",
        "/R:3", "/W:3", "/NP", "/NFL", "/NDL"
    )
    $code = robocopy @args

    switch ($code) {
        0 { Write-Host "  [OK] 已是最新" -ForegroundColor Green }
        1 { Write-Host "  [OK] 有新文件" -ForegroundColor Green }
        { $_ -ge 2 -and $_ -le 7 } { Write-Host "  [OK] 完成 (exit=$code)" -ForegroundColor Yellow }
        default { Write-Host "  [ERROR] robocopy exit=$code" -ForegroundColor Red; return $false }
    }
    return $true
}

# 每个 Src → 每个 DST
Write-Host ""
$total = $srcPaths.Count * $dstPaths.Count
$done = 0
$errors = 0

foreach ($sp in $srcPaths) {
    foreach ($dp in $dstPaths) {
        $done++
        Write-Host "[$done/$total] $sp  →  $dp" -ForegroundColor Cyan
        if (-not (Invoke-CopyTask -Src $sp -Dst $dp)) {
            $errors++
        }
    }
}

Write-Host ""
Write-Host "========================================" -ForegroundColor Cyan
if ($errors -gt 0) {
    Write-Host "  分发完成（$($total - $errors)/$total 成功，$errors 失败）" -ForegroundColor Yellow
} else {
    Write-Host "  分发完成! ($total/$total)" -ForegroundColor Green
}
Write-Host "========================================" -ForegroundColor Cyan
if ($errors -gt 0) { exit 1 }
