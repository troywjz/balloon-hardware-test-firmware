param(
    [string]$CubeMxHome = $env:STM32CUBEMX_HOME
)

$ErrorActionPreference = "Stop"

if ([string]::IsNullOrWhiteSpace($CubeMxHome)) {
    $CubeMxHome = "D:\STM32CubeMX"
}

$java = Join-Path $CubeMxHome "jre\bin\java.exe"
$cubeMx = Join-Path $CubeMxHome "STM32CubeMX.exe"
$projectRoot = Split-Path -Parent $PSScriptRoot
$commandFile = Join-Path $projectRoot "cubemx-generate.txt"

if (-not (Test-Path -LiteralPath $java) -or -not (Test-Path -LiteralPath $cubeMx)) {
    throw "STM32CubeMX not found. Set STM32CUBEMX_HOME to its installation directory."
}

Push-Location $projectRoot
try {
    & $java -jar $cubeMx -q $commandFile
    if ($LASTEXITCODE -ne 0) {
        throw "STM32CubeMX generation failed with exit code $LASTEXITCODE."
    }
}
finally {
    Pop-Location
}
