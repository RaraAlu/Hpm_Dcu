param([string]$SscToolDir = 'C:/Program Files (x86)/Beckhoff/EtherCAT Slave Stack Code Tool')
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path (Split-Path $PSScriptRoot -Parent) -Parent
[void][Reflection.Assembly]::LoadFrom((Join-Path $SscToolDir 'ET9300Utilities.dll'))
$esi = Join-Path $projectRoot 'protocol/ethercat/HPM_ENCOS_4CAN_28AXIS_IMU.xml'
$compactEsi = Join-Path $env:TEMP 'HPM_ENCOS_4CAN_28AXIS_IMU.sii.xml'
[xml]$xml = [IO.File]::ReadAllText($esi)
# Names remain in the published ESI and online CoE dictionary. They are not
# needed in the board EEPROM, where IgH imposes an 8 KiB category boundary.
@($xml.SelectNodes('//Name')) |
    ForEach-Object { [void]$_.ParentNode.RemoveChild($_) }
$xml.SelectSingleNode('//Device/Type').InnerText = 'HPM_28AXIS'
$settings = [Xml.XmlWriterSettings]::new()
$settings.Encoding = [Text.UTF8Encoding]::new($false)
$settings.Indent = $false
$writer = [Xml.XmlWriter]::Create($compactEsi, $settings)
try { $xml.Save($writer) } finally { $writer.Dispose() }
try {
    $devices = [ET9300Utilities.EsiSiiHandler]::GetDeviceDescriptions($compactEsi)
    if ($devices.Count -ne 1) { throw 'Expected one ENCOS device.' }
    $content = [ET9300Utilities.EsiSiiHandler]::GetEEPROMContent($devices[0])
} finally {
    Remove-Item -LiteralPath $compactEsi -ErrorAction SilentlyContinue
}
if ($content.Length -ne 16384) { throw 'Expected 16 KiB EEPROM image.' }
if ([BitConverter]::ToUInt32($content, 20) -ne 2) { throw 'ProductCode mismatch.' }
$word = 64
while ($word -lt ($content.Length / 2)) {
    $category = [BitConverter]::ToUInt16($content, $word * 2)
    if ($category -eq 0xFFFF) { break }
    $word += 2 + [BitConverter]::ToUInt16($content, $word * 2 + 2)
}
if ($category -ne 0xFFFF -or $word -ge 4096) { throw "SII terminator word $word must be below 4096 for IgH." }
[IO.File]::WriteAllBytes((Join-Path $projectRoot 'protocol/ethercat/HPM_ENCOS_4CAN_28AXIS_IMU.bin'), $content)
[ET9300Utilities.EsiSiiHandler]::CreateSscEepromHeader($content, (Join-Path $projectRoot 'firmware/generated/ethercat/eeprom.h'), 16384, '/* Generated from protocol/ethercat/HPM_ENCOS_4CAN_28AXIS_IMU.xml using Beckhoff SSC. */')
Write-Output "Generated 16 KiB SII with full PDO categories; terminator is at word $word."
