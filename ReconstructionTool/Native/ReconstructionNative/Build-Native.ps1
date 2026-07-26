param(
    [ValidateSet('Release', 'Debug')]
    [string]$Configuration = 'Release',
    [string]$OutputDirectory = '..\..\Assets\Plugins\x86_64',
    [string]$DependencyRoot = ''
)

$ErrorActionPreference = 'Stop'
$scriptRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$buildRoot = Join-Path $scriptRoot '.native-build'
$resolvedOutput = [System.IO.Path]::GetFullPath((Join-Path $scriptRoot $OutputDirectory))
$pluginBaseName = 'ReconstructionNative_1_8'

$vswhere = 'C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe'
if (-not (Test-Path -LiteralPath $vswhere)) {
    throw 'Visual Studio Installer (vswhere.exe) was not found.'
}

$visualStudio = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if (-not $visualStudio) {
    throw 'A Visual Studio installation with the x64 C++ workload is required.'
}
$visualStudioVersion = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationVersion
$visualStudioMajor = [int]($visualStudioVersion.Split('.')[0])
$generator = switch ($visualStudioMajor) {
    18 { 'Visual Studio 18 2026' }
    17 { 'Visual Studio 17 2022' }
    default { throw "Unsupported Visual Studio major version: $visualStudioMajor" }
}

$cmake = Join-Path $visualStudio 'Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe'
if (-not (Test-Path -LiteralPath $cmake)) {
    throw "CMake was not found at $cmake"
}

$arguments = @(
    '-S', $scriptRoot,
    '-B', $buildRoot,
    '-G', $generator,
    '-A', 'x64',
    '-DRECONSTRUCTION_BUILD_TESTS=ON'
)

if ($DependencyRoot) {
    $resolvedDependencies = [System.IO.Path]::GetFullPath($DependencyRoot)
    $arguments += "-DFETCHCONTENT_SOURCE_DIR_EIGEN=$(Join-Path $resolvedDependencies 'eigen')"
    $arguments += "-DFETCHCONTENT_SOURCE_DIR_CERES=$(Join-Path $resolvedDependencies 'ceres-solver')"
    $arguments += "-DFETCHCONTENT_SOURCE_DIR_POSELIB=$(Join-Path $resolvedDependencies 'PoseLib')"
}

& $cmake @arguments
if ($LASTEXITCODE -ne 0) {
    throw 'CMake configuration failed.'
}

& $cmake --build $buildRoot --config $Configuration --target ReconstructionNative ReconstructionNativeTests -- /m
if ($LASTEXITCODE -ne 0) {
    throw 'Native build failed.'
}

$testExecutable = Join-Path $buildRoot "$Configuration\ReconstructionNativeTests.exe"
if (-not (Test-Path -LiteralPath $testExecutable)) {
    throw "Native test executable was not found at $testExecutable"
}

& $testExecutable
if ($LASTEXITCODE -ne 0) {
    throw 'Native tests failed.'
}

New-Item -ItemType Directory -Force -Path $resolvedOutput | Out-Null
Copy-Item -Force -LiteralPath (Join-Path $buildRoot "$Configuration\$pluginBaseName.dll") -Destination $resolvedOutput

$pdb = Join-Path $buildRoot "$Configuration\$pluginBaseName.pdb"
if (Test-Path -LiteralPath $pdb) {
    Copy-Item -Force -LiteralPath $pdb -Destination $resolvedOutput
}

Write-Host "Native plugin copied to $resolvedOutput"
