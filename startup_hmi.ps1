# ===== PATHS =====
$projectPath = "C:\Users\JACKS\Downloads\CODE HOPEFULLY WORKS\CODE HOPEFULLY WORKS\routefix_fixed"
$chromePath = "C:\Program Files\Google\Chrome\Application\chrome.exe"
$loadingPage = "file:///C:/Users/JACKS/Documents/HMI/loading.html"

Write-Host "Closing existing Chrome instances..."
taskkill /f /im chrome.exe *> $null

Start-Sleep -Seconds 2

# ===== LAUNCH CHROME FIRST (IMPORTANT) =====
Write-Host "Launching Chrome in kiosk..."
Start-Process -FilePath $chromePath `
    -ArgumentList "--user-data-dir=`"$env:LOCALAPPDATA\ChromeKioskProfile`" --kiosk `"$loadingPage`" --new-window --no-first-run --disable-infobars"

Start-Sleep -Seconds 2

# ===== START RELAY (HIDDEN) =====
Write-Host "Starting relay..."
Start-Process powershell.exe -WindowStyle Hidden `
    -ArgumentList "-NoProfile -Command `"Set-Location '$projectPath'; python relay.py`""

Start-Sleep -Seconds 3

# ===== START HTTP SERVER (HIDDEN) =====
Write-Host "Starting http server..."
Start-Process powershell.exe -WindowStyle Hidden `
    -ArgumentList "-NoProfile -Command `"Set-Location '$projectPath'; npx http-server -p 8086`""