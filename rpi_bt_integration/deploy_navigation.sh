#!/bin/bash
# Script per copiare i file di navigazione sulla Raspberry Pi

RPI_USER="pi"
RPI_HOST="pi.local"
RPI_BT_DIR="/home/pi/robot_code/bt"
RPI_MAIN_DIR="/home/pi/robot_code"
LOCAL_DIR="$(dirname "$0")"

echo "=========================================="
echo "📦 DEPLOYMENT NAVIGAZIONE SU RASPBERRY PI"
echo "=========================================="
echo ""
echo "Target: $RPI_USER@$RPI_HOST:$RPI_BT_DIR"
echo ""

# Verifica connessione
echo "🔍 Verifica connessione..."
if ! ping -c 1 -W 2 "$RPI_HOST" &> /dev/null; then
    echo "❌ Impossibile raggiungere $RPI_HOST"
    exit 1
fi
echo "✅ Raspberry Pi raggiungibile"
echo ""

# Backup file esistenti
echo "💾 Backup file esistenti..."
ssh "$RPI_USER@$RPI_HOST" "cd $RPI_BT_DIR && cp sensors.py sensors.py.backup 2>/dev/null || true"
echo "✅ Backup completato"
echo ""

# Copia nuovi file
echo "📤 Copia file di navigazione..."

scp "$LOCAL_DIR/sensors.py" "$RPI_USER@$RPI_HOST:$RPI_BT_DIR/"
scp "$LOCAL_DIR/navigation_actions.py" "$RPI_USER@$RPI_HOST:$RPI_BT_DIR/"
scp "$LOCAL_DIR/navigation_behaviours.py" "$RPI_USER@$RPI_HOST:$RPI_BT_DIR/"
scp "$LOCAL_DIR/main_mission.py" "$RPI_USER@$RPI_HOST:$RPI_MAIN_DIR/"

if [ $? -eq 0 ]; then
    echo ""
    echo "✅ File copiati con successo!"
else
    echo ""
    echo "❌ Errore durante la copia"
    exit 1
fi

# Verifica installazione
echo ""
echo "🔍 Verifica file..."
ssh "$RPI_USER@$RPI_HOST" "ls -lh $RPI_BT_DIR/*.py $RPI_MAIN_DIR/main_mission.py"

echo ""
echo "=========================================="
echo "✅ DEPLOYMENT COMPLETATO!"
echo "=========================================="
echo ""
echo "🚀 Per testare il sistema:"
echo "   ssh $RPI_USER@$RPI_HOST"
echo "   cd $RPI_MAIN_DIR"
echo "   source ~/venv/bin/activate  # Attiva virtual environment"
echo "   python3 main_mission.py"
echo ""
echo "📝 File installati:"
echo "   - $RPI_BT_DIR/sensors.py (aggiornato)"
echo "   - $RPI_BT_DIR/navigation_actions.py (nuovo)"
echo "   - $RPI_BT_DIR/navigation_behaviours.py (nuovo)"
echo "   - $RPI_MAIN_DIR/main_mission.py (nuovo)"
echo ""
raspberr