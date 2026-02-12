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

# Copia file del modulo bt/
echo "📤 Copia file del modulo bt/..."
for file in "$LOCAL_DIR/bt"/*.py; do
    if [ -f "$file" ]; then
        filename=$(basename "$file")
        echo "  Copiando bt/$filename..."
        scp "$file" "$RPI_USER@$RPI_HOST:$RPI_BT_DIR/"
        if [ $? -ne 0 ]; then
            echo "❌ Errore copiando $filename"
            exit 1
        fi
    fi
done

echo ""
echo "📤 Copia file principali..."
for file in "$LOCAL_DIR"/*.py; do
    if [ -f "$file" ]; then
        filename=$(basename "$file")
        echo "  Copiando $filename..."
        scp "$file" "$RPI_USER@$RPI_HOST:$RPI_MAIN_DIR/"
        if [ $? -ne 0 ]; then
            echo "❌ Errore copiando $filename"
            exit 1
        fi
    fi
done

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
echo "=== File in bt/ ==="
ssh "$RPI_USER@$RPI_HOST" "ls -lh $RPI_BT_DIR/*.py"
echo ""
echo "=== File nella directory principale ==="
ssh "$RPI_USER@$RPI_HOST" "ls -lh $RPI_MAIN_DIR/*.py"

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
echo "   - Tutti i file .py da bt/ → $RPI_BT_DIR/"
echo "   - Tutti i file .py dalla root → $RPI_MAIN_DIR/"
echo ""
raspberr