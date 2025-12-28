@echo off
echo ==========================================
echo      STARTING DEEP CLEAN & REBUILD
echo ==========================================

REM 1. Stop any running Node processes to unlock files
echo [1/5] Stopping background Node processes...
taskkill /F /IM node.exe >nul 2>&1

REM 2. Force delete corrupted folders (This might take a minute)
echo [2/5] Deleting corrupted folders (node_modules, .docusaurus)...
rd /s /q node_modules
rd /s /q .docusaurus
rd /s /q build
del package-lock.json

REM 3. Clean the internal NPM cache
echo [3/5] Cleaning NPM Cache...
npm cache clean --force

REM 4. Re-install fresh dependencies
echo [4/5] Installing fresh dependencies...
call npm install

REM 5. Run the build
echo [5/5] Building the project...
call npm run build

echo ==========================================
echo      PROCESS COMPLETE
echo ==========================================
pause