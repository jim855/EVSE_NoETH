Get-Content ..\src\screen.cpp,..\src\main.cpp -Raw -Encoding UTF8 > temp.cpp
$src = Get-Content temp.cpp -Encoding UTF8 -Raw
$uniEnc = [System.Text.Encoding]::Unicode
$uniChars = [System.Text.RegularExpressions.Regex]::Matches($src, '[^\x0a-\x7f]') | 
    ForEach-Object { $_.Value } | Sort-Object | Get-Unique
Write-Host "發現中文字元: "
$uniChars -join ', ' 
(@("32-128") + ($uniChars | ForEach-Object { '$' + ([Uint32]([char]$_)).ToString('X4') })) -join ",`n" | Out-File ".\build\chinese1.map" -Encoding utf8

& .\bdfconv\bdfconv.exe -v ./bdf/WenQuanYiZenHeiMono-16.bdf -b 0 -f 1 -M ./build/chinese1.map -d ./bdf/WenQuanYiZenHeiMono-16.bdf -n cht_font_16 -o cht_font_16.h
& .\bdfconv\bdfconv.exe -v ./bdf/WenQuanYiZenHeiMono-24.bdf -b 0 -f 1 -M ./build/chinese1.map -d ./bdf/WenQuanYiZenHeiMono-24.bdf -n cht_font_24 -o cht_font_24.h
& .\bdfconv\bdfconv.exe -v ./bdf/WenQuanYiZenHeiMono-32.bdf -b 0 -f 1 -M ./build/chinese1.map -d ./bdf/WenQuanYiZenHeiMono-32.bdf -n cht_font_32 -o cht_font_32.h


