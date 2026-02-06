# Script per copiare i file di navigazione sulla Raspberry Pi
# PowerShell version for Windows

$RPI_USER = "pi"
$RPI_HOST = "pi.local"
$RPI_PASSWORD = "raspberrypi"  # Change this to your Raspberry Pi password
$RPI_BT_DIR = "/home/pi/robot_code/bt"
$RPI_MAIN_DIR = "/home/pi/robot_code"
$LOCAL_DIR = $PSScriptRoot

# Check if using SSH keys or password
$USE_SSH_KEYS = $true  # Set to $true if you have SSH keys configured

Write-Host "==========================================" -ForegroundColor Cyan
Write-Host "DEPLOYMENT NAVIGAZIONE SU RASPBERRY PI" -ForegroundColor Cyan
Write-Host "==========================================" -ForegroundColor Cyan
Write-Host ""
Write-Host "Target: $RPI_USER@$RPI_HOST`:$RPI_BT_DIR"
Write-Host ""

# Determine which tools to use
if ($USE_SSH_KEYS) {
    $SSH_CMD = "ssh"
    $SCP_CMD = "scp"
    Write-Host "Using SSH keys for authentication" -ForegroundColor Cyan
} else {
    # Check if plink and pscp are available (from PuTTY)
    $plinkPath = Get-Command plink -ErrorAction SilentlyContinue
    $pscpPath = Get-Command pscp -ErrorAction SilentlyContinue
    
    if ($plinkPath -and $pscpPath) {
        $SSH_CMD = "plink"
        $SCP_CMD = "pscp"
        $SSH_OPTS = "-pw `"$RPI_PASSWORD`""
        $SCP_OPTS = "-pw `"$RPI_PASSWORD`""
        Write-Host "Using password authentication with PuTTY tools" -ForegroundColor Cyan
    } else {
        Write-Host "ERROR: Password authentication requires PuTTY tools (plink, pscp)" -ForegroundColor Red
        Write-Host ""
        Write-Host "Options:" -ForegroundColor Yellow
        Write-Host "1. Install PuTTY from: https://www.putty.org/" -ForegroundColor Yellow
        Write-Host "   Then add PuTTY directory to your PATH" -ForegroundColor Yellow
        Write-Host ""
        Write-Host "2. Set up SSH keys (recommended):" -ForegroundColor Yellow
        Write-Host "   ssh-keygen -t rsa" -ForegroundColor Yellow
        Write-Host "   ssh-copy-id $RPI_USER@$RPI_HOST" -ForegroundColor Yellow
        Write-Host "   Then set `$USE_SSH_KEYS = `$true in this script" -ForegroundColor Yellow
        Write-Host ""
        exit 1
    }
}
Write-Host ""

# Verifica connessione
Write-Host "Verifica connessione..." -ForegroundColor Yellow
$pingResult = Test-Connection -ComputerName $RPI_HOST -Count 1 -Quiet -ErrorAction SilentlyContinue
if (-not $pingResult) {
    Write-Host "Impossibile raggiungere $RPI_HOST" -ForegroundColor Red
    exit 1
}
Write-Host "Raspberry Pi raggiungibile" -ForegroundColor Green
Write-Host ""

# Crea struttura directory se non esiste
Write-Host "Verifica struttura directory..." -ForegroundColor Yellow
if ($USE_SSH_KEYS) {
    & $SSH_CMD "$RPI_USER@$RPI_HOST" "mkdir -p $RPI_MAIN_DIR $RPI_BT_DIR"
} else {
    & $SSH_CMD $SSH_OPTS.Split() "$RPI_USER@$RPI_HOST" "mkdir -p $RPI_MAIN_DIR $RPI_BT_DIR"
}
Write-Host "Struttura directory verificata" -ForegroundColor Green
Write-Host ""

# Backup file esistenti
Write-Host "Backup file esistenti..." -ForegroundColor Yellow
if ($USE_SSH_KEYS) {
    & $SSH_CMD "$RPI_USER@$RPI_HOST" "cd $RPI_BT_DIR && for f in *.py; do [ -f `"`$f`" ] && cp `"`$f`" `"`$f.backup`" 2>/dev/null || true; done"
    & $SSH_CMD "$RPI_USER@$RPI_HOST" "cd $RPI_MAIN_DIR && for f in *.py; do [ -f `"`$f`" ] && cp `"`$f`" `"`$f.backup`" 2>/dev/null || true; done"
} else {
    & $SSH_CMD $SSH_OPTS.Split() "$RPI_USER@$RPI_HOST" "cd $RPI_BT_DIR && for f in *.py; do [ -f `"`$f`" ] && cp `"`$f`" `"`$f.backup`" 2>/dev/null || true; done"
    & $SSH_CMD $SSH_OPTS.Split() "$RPI_USER@$RPI_HOST" "cd $RPI_MAIN_DIR && for f in *.py; do [ -f `"`$f`" ] && cp `"`$f`" `"`$f.backup`" 2>/dev/null || true; done"
}
Write-Host "Backup completato" -ForegroundColor Green
Write-Host ""

# Copia file del modulo bt/
Write-Host "Copia file del modulo bt/..." -ForegroundColor Yellow

# Get all .py files from bt/ subdirectory
$btFiles = Get-ChildItem -Path "$LOCAL_DIR\bt" -Filter "*.py" -File
$allSuccess = $true

foreach ($file in $btFiles) {
    $fileName = $file.Name
    Write-Host "  Copiando bt/$fileName..." -ForegroundColor Gray
    if ($USE_SSH_KEYS) {
        & $SCP_CMD "$LOCAL_DIR\bt\$fileName" "$RPI_USER@$RPI_HOST`:$RPI_BT_DIR/"
    } else {
        & $SCP_CMD $SCP_OPTS.Split() "$LOCAL_DIR\bt\$fileName" "$RPI_USER@$RPI_HOST`:$RPI_BT_DIR/"
    }
    if ($LASTEXITCODE -ne 0) {
        $allSuccess = $false
    }
}

Write-Host ""
Write-Host "Copia file principali..." -ForegroundColor Yellow

# Get all .py files from root directory (excluding subdirectories)
$mainFiles = Get-ChildItem -Path "$LOCAL_DIR" -Filter "*.py" -File

foreach ($file in $mainFiles) {
    $fileName = $file.Name
    Write-Host "  Copiando $fileName..." -ForegroundColor Gray
    if ($USE_SSH_KEYS) {
        & $SCP_CMD "$LOCAL_DIR\$fileName" "$RPI_USER@$RPI_HOST`:$RPI_MAIN_DIR/"
    } else {
        & $SCP_CMD $SCP_OPTS.Split() "$LOCAL_DIR\$fileName" "$RPI_USER@$RPI_HOST`:$RPI_MAIN_DIR/"
    }
    if ($LASTEXITCODE -ne 0) {
        $allSuccess = $false
    }
}

if ($allSuccess) {
    Write-Host ""
    Write-Host "File copiati con successo!" -ForegroundColor Green
} else {
    Write-Host ""
    Write-Host "Errore durante la copia" -ForegroundColor Red
    exit 1
}

# Verifica installazione
Write-Host ""
Write-Host "Verifica file..." -ForegroundColor Yellow
if ($USE_SSH_KEYS) {
    & $SSH_CMD "$RPI_USER@$RPI_HOST" "echo '=== File in bt/ ===' && ls -lh $RPI_BT_DIR/*.py && echo '' && echo '=== File nella directory principale ===' && ls -lh $RPI_MAIN_DIR/*.py"
} else {
    & $SSH_CMD $SSH_OPTS.Split() "$RPI_USER@$RPI_HOST" "echo '=== File in bt/ ===' && ls -lh $RPI_BT_DIR/*.py && echo '' && echo '=== File nella directory principale ===' && ls -lh $RPI_MAIN_DIR/*.py"
}

Write-Host ""
Write-Host "==========================================" -ForegroundColor Cyan
Write-Host "DEPLOYMENT COMPLETATO!" -ForegroundColor Cyan
Write-Host "==========================================" -ForegroundColor Cyan
Write-Host ""
Write-Host "Per testare il sistema:" -ForegroundColor Yellow
if ($USE_SSH_KEYS) {
    Write-Host "   ssh $RPI_USER@$RPI_HOST"
} else {
    Write-Host "   plink -pw `"$RPI_PASSWORD`" $RPI_USER@$RPI_HOST"
}
Write-Host "   cd $RPI_MAIN_DIR"
Write-Host "   source ~/venv/bin/activate  # Attiva virtual environment"
Write-Host "   python3 main_mission.py"
Write-Host ""
Write-Host "File installati:" -ForegroundColor Yellow
Write-Host "   - Tutti i file .py da bt/ -> $RPI_BT_DIR/"
Write-Host "   - Tutti i file .py dalla root -> $RPI_MAIN_DIR/"
Write-Host ""
Write-Host "NOTA: " -ForegroundColor Yellow -NoNewline
if ($USE_SSH_KEYS) {
    Write-Host "Usando autenticazione SSH key (nessuna password richiesta)"
} else {
    Write-Host "Password memorizzata nello script. Per maggiore sicurezza, configura SSH keys."
}
Write-Host ""
