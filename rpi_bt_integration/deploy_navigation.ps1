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

# Backup file esistenti
Write-Host "Backup file esistenti..." -ForegroundColor Yellow
if ($USE_SSH_KEYS) {
    & $SSH_CMD "$RPI_USER@$RPI_HOST" "cd $RPI_BT_DIR && cp sensors.py sensors.py.backup 2>/dev/null || true"
} else {
    & $SSH_CMD $SSH_OPTS.Split() "$RPI_USER@$RPI_HOST" "cd $RPI_BT_DIR && cp sensors.py sensors.py.backup 2>/dev/null || true"
}
Write-Host "Backup completato" -ForegroundColor Green
Write-Host ""

# Copia nuovi file
Write-Host "Copia file di navigazione..." -ForegroundColor Yellow

$files = @(
    @{Source = "$LOCAL_DIR\sensors.py"; Dest = "$RPI_USER@$RPI_HOST`:$RPI_BT_DIR/"},
    @{Source = "$LOCAL_DIR\navigation_actions.py"; Dest = "$RPI_USER@$RPI_HOST`:$RPI_BT_DIR/"},
    @{Source = "$LOCAL_DIR\navigation_behaviours.py"; Dest = "$RPI_USER@$RPI_HOST`:$RPI_BT_DIR/"},
    @{Source = "$LOCAL_DIR\main_mission.py"; Dest = "$RPI_USER@$RPI_HOST`:$RPI_MAIN_DIR/"}
)

$allSuccess = $true
foreach ($file in $files) {
    if ($USE_SSH_KEYS) {
        & $SCP_CMD $file.Source $file.Dest
    } else {
        & $SCP_CMD $SCP_OPTS.Split() $file.Source $file.Dest
    }
    if ($LASTEXITCODE -ne 0) {
        $allSuccess = $false
    }
}

Write-Host ""
Write-Host "Copia file di supporto..." -ForegroundColor Yellow

$supportFiles = @(
    @{Source = "$LOCAL_DIR\rover_API.py"; Dest = "$RPI_USER@$RPI_HOST`:$RPI_MAIN_DIR/"},
    @{Source = "$LOCAL_DIR\main.py"; Dest = "$RPI_USER@$RPI_HOST`:$RPI_MAIN_DIR/"},
    @{Source = "$LOCAL_DIR\wifi_bridge.py"; Dest = "$RPI_USER@$RPI_HOST`:$RPI_MAIN_DIR/"}
)

foreach ($file in $supportFiles) {
    if ($USE_SSH_KEYS) {
        & $SCP_CMD $file.Source $file.Dest
    } else {
        & $SCP_CMD $SCP_OPTS.Split() $file.Source $file.Dest
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
    & $SSH_CMD "$RPI_USER@$RPI_HOST" "ls -lh $RPI_BT_DIR/*.py $RPI_MAIN_DIR/main_mission.py $RPI_MAIN_DIR/rover_API.py $RPI_MAIN_DIR/main.py $RPI_MAIN_DIR/wifi_bridge.py"
} else {
    & $SSH_CMD $SSH_OPTS.Split() "$RPI_USER@$RPI_HOST" "ls -lh $RPI_BT_DIR/*.py $RPI_MAIN_DIR/main_mission.py $RPI_MAIN_DIR/rover_API.py $RPI_MAIN_DIR/main.py $RPI_MAIN_DIR/wifi_bridge.py"
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
Write-Host "   - $RPI_BT_DIR/sensors.py (aggiornato)"
Write-Host "   - $RPI_BT_DIR/navigation_actions.py (nuovo)"
Write-Host "   - $RPI_BT_DIR/navigation_behaviours.py (nuovo)"
Write-Host "   - $RPI_MAIN_DIR/main_mission.py (nuovo)"
Write-Host "   - $RPI_MAIN_DIR/rover_API.py (aggiornato)"
Write-Host "   - $RPI_MAIN_DIR/main.py (aggiornato)"
Write-Host "   - $RPI_MAIN_DIR/wifi_bridge.py (aggiornato)"
Write-Host ""
Write-Host "NOTA: " -ForegroundColor Yellow -NoNewline
if ($USE_SSH_KEYS) {
    Write-Host "Usando autenticazione SSH key (nessuna password richiesta)"
} else {
    Write-Host "Password memorizzata nello script. Per maggiore sicurezza, configura SSH keys."
}
Write-Host ""
