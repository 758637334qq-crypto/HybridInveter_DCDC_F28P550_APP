param(
    [Parameter(Mandatory = $true)][string]$InputFile,
    [Parameter(Mandatory = $true)][string]$Output
)

$ErrorActionPreference = 'Stop'
$lines = New-Object System.Collections.Generic.List[string]
$inputUpperAddress = 0
$outputUpperAddress = -1

function AddIntelHexRecord([UInt16]$Address, [byte]$RecordType, [byte[]]$Data)
{
    $sum = $Data.Length + $RecordType + (($Address -shr 8) -band 0xFF) + ($Address -band 0xFF)
    $text = ':' + ('{0:X2}{1:X4}{2:X2}' -f $Data.Length, $Address, $RecordType)
    foreach ($value in $Data) {
        $sum += $value
        $text += ('{0:X2}' -f $value)
    }
    $script:lines.Add($text + ('{0:X2}' -f ((-$sum) -band 0xFF)))
}

function AddWordAddressedData([UInt32]$WordAddress, [byte[]]$Data)
{
    $offset = 0
    while ($offset -lt $Data.Length) {
        $currentWordAddress = $WordAddress + [UInt32]($offset / 2)
        $upperAddress = [UInt16](($currentWordAddress -shr 16) -band 0xFFFF)
        if ($script:outputUpperAddress -ne $upperAddress) {
            $upperData = New-Object byte[] 2
            $upperData[0] = [byte]($upperAddress -shr 8)
            $upperData[1] = [byte]($upperAddress -band 0xFF)
            AddIntelHexRecord 0 4 $upperData
            $script:outputUpperAddress = $upperAddress
        }

        $wordsUntilBoundary = 0x10000 - ($currentWordAddress -band 0xFFFF)
        $byteCount = [Math]::Min($Data.Length - $offset, $wordsUntilBoundary * 2)
        if (($byteCount % 2) -ne 0) {
            throw 'C28x word-addressed HEX requires an even byte count.'
        }

        $recordData = New-Object byte[] $byteCount
        [Array]::Copy($Data, $offset, $recordData, 0, $byteCount)
        AddIntelHexRecord ([UInt16]($currentWordAddress -band 0xFFFF)) 0 $recordData
        $offset += $byteCount
    }
}

foreach ($line in [System.IO.File]::ReadAllLines($InputFile)) {
    if (($line.Length -lt 11) -or ($line[0] -ne ':')) {
        continue
    }

    $length = [Convert]::ToInt32($line.Substring(1, 2), 16)
    $address = [Convert]::ToUInt32($line.Substring(3, 4), 16)
    $recordType = [Convert]::ToUInt32($line.Substring(7, 2), 16)

    if ($recordType -eq 4) {
        $inputUpperAddress = [Convert]::ToUInt32($line.Substring(9, 4), 16)
        continue
    }
    if ($recordType -ne 0) {
        continue
    }

    $byteAddress = ($inputUpperAddress -shl 16) + $address
    if ((($byteAddress % 2) -ne 0) -or (($length % 2) -ne 0)) {
        throw 'Input HEX contains an unaligned C28x word address.'
    }

    $data = New-Object byte[] $length
    for ($index = 0; $index -lt $length; $index++) {
        $data[$index] = [Convert]::ToByte($line.Substring(9 + ($index * 2), 2), 16)
    }
    AddWordAddressedData ([UInt32]($byteAddress / 2)) $data
}

$lines.Add(':00000001FF')
[System.IO.File]::WriteAllText($Output, (($lines -join "`r`n") + "`r`n"), [System.Text.Encoding]::ASCII)
