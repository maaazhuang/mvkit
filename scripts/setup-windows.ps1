[CmdletBinding()]
param(
  [ValidateSet('Debug','Release','RelWithDebInfo','MinSizeRel')][string]$Configuration='Release',
  [string]$Generator='',
  [ValidateSet('x64','Win32','ARM64')][string]$Architecture='x64',
  [switch]$Offline,
  [string]$PrefixPath='',
  [switch]$ConfigureOnly,
  [ValidateRange(0,256)][int]$Jobs=0
)
$ErrorActionPreference='Stop'
$projectRoot=Split-Path -Parent $PSScriptRoot
$buildPath=Join-Path $projectRoot 'build'
# PATH 中的旧 CMake 可能不支持 VS2026；按已安装 VS 和生成器能力选择工具，优先使用最新 VS。
$candidates=@()
$pathCmake=Get-Command cmake -CommandType Application -ErrorAction SilentlyContinue
if($pathCmake){$candidates+=$pathCmake.Source}
$vswhere=Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio/Installer/vswhere.exe'
if(Test-Path -LiteralPath $vswhere){
  $installations=& $vswhere -all -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
  foreach($installation in $installations){
    $candidates+=Join-Path $installation 'Common7/IDE/CommonExtensions/Microsoft/CMake/CMake/bin/cmake.exe'
  }
}
$cmakePath=$null
$generators=if($Generator){@($Generator)}else{@('Visual Studio 18 2026','Visual Studio 17 2022','Visual Studio 16 2019')}
$available=@()
foreach($candidate in ($candidates | Select-Object -Unique)){
  if(-not(Test-Path -LiteralPath $candidate)){continue}
  $capabilities=& $candidate -E capabilities | ConvertFrom-Json
  if($LASTEXITCODE -eq 0){$available+=@{Path=$candidate; Generators=$capabilities.generators.name}}
}
foreach($preferred in $generators){
  # 自动模式还要确认对应 VS 的 C++ 工具已安装，不能只依据 CMake 支持列表。
  if(-not $Generator){
    $major=($preferred -split ' ')[2]
    $instances=if(Test-Path -LiteralPath $vswhere){& $vswhere -products '*' -version "[$major.0,$([int]$major+1).0)" -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath}
    if(-not $instances){continue}
  }
  foreach($tool in $available){
    if($preferred -in $tool.Generators){$cmakePath=$tool.Path;$Generator=$preferred;break}
  }
  if($cmakePath){break}
}
if(-not $cmakePath){throw 'No supported compiler/CMake pair found. Install VS2019/2022/2026 with Desktop development with C++ and CMake tools; VS2026 requires CMake 4.2+.'}
Write-Host "CMake: $cmakePath; generator: $Generator"
$configure=@('-S',$projectRoot,'-B',$buildPath)
# 默认配置与直接 CMake 入口共用预设；自选生成器不继承 VS 的平台参数。
if($Generator -like 'Visual Studio *'){$configure+=@('--preset','windows','-G',$Generator)}
else{$configure+=@('-G',$Generator)}
if($Generator -like 'Visual Studio *'){$configure+=@('-A',$Architecture)}
else{$configure+="-DCMAKE_BUILD_TYPE=$Configuration"}
$configure+="-DMVKIT_OFFLINE=$($Offline.IsPresent)"
if($PrefixPath){$configure+="-DCMAKE_PREFIX_PATH=$PrefixPath"}
# 工具链或生成器变化时请先删除 build，不能混用旧 CMake 缓存。
& $cmakePath @configure
if($LASTEXITCODE -ne 0){throw 'MVKit configuration failed.'}
if($ConfigureOnly){return}
$buildArguments=@('--build',$buildPath,'--config',$Configuration,'--parallel')
if($Jobs){$buildArguments+="$Jobs"}
& $cmakePath @buildArguments
if($LASTEXITCODE -ne 0){throw 'MVKit build failed.'}
Write-Host "MVKit built in $buildPath"
