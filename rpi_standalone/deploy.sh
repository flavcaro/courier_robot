#!/bin/bash
# Script per copiare i file sulla Raspberry Pi via SSH

# Configurazione
RPI_USER="pi"
RPI_HOST="pi.local"  # o usa IP: 192.168.1.100
RPI_DIR="/home/pi/courier_robot"
LOCAL_DIR="$(dirname "$0")"

echo "=========================================="
echo "📦 DEPLOYMENT SU RASPBERRY PI"
echo "=========================================="
echo ""
echo "Target: $RPI_USER@$RPI_HOST:$RPI_DIR"
echo ""

# Verifica connessione
echo "🔍 Verifica connessione..."
if ! ping -c 1 -W 2 "$RPI_HOST" &> /dev/null; then
    echo "❌ Impossibile raggiungere $RPI_HOST"
    echo ""
    echo "💡 Suggerimenti:"
    echo "   - Verifica che Raspberry Pi sia acceso"
    echo "   - Controlla connessione di rete"
    echo "   - Prova con IP invece di hostname"
    echo "   - Modifica RPI_HOST in questo script"
    exit 1
fi
echo "✅ Raspberry Pi raggiungibile"
echo ""

# Crea directory remota
echo "📁 Creazione directory remota..."
ssh "$RPI_USER@$RPI_HOST" "mkdir -p $RPI_DIR/behaviors"
echo "✅ Directory creata"
echo ""

# Copia file
echo "📤 Copia file..."
rsync -avz --progress \
    --exclude='__pycache__' \
    --exclude='*.pyc' \
    --exclude='.git' \
    "$LOCAL_DIR/" "$RPI_USER@$RPI_HOST:$RPI_DIR/"

if [ $? -eq 0 ]; then
    echo ""
    echo "✅ File copiati con successo!"
else
    echo ""
    echo "❌ Errore durante la copia"
    exit 1
fi

# Installa dipendenze
echo ""
echo "📦 Installazione dipendenze..."
ssh "$RPI_USER@$RPI_HOST" "cd $RPI_DIR && pip3 install -r requirements.txt"

if [ $? -eq 0 ]; then
    echo "✅ Dipendenze installate"
else
    echo "⚠️  Errore installazione dipendenze (potrebbe essere normale se già installate)"
fi

# Rendi eseguibili gli script
echo ""
echo "🔧 Configurazione permessi..."
ssh "$RPI_USER@$RPI_HOST" "chmod +x $RPI_DIR/*.py"
echo "✅ Permessi configurati"

# Mostra istruzioni finali
echo ""
echo "=========================================="
echo "✅ DEPLOYMENT COMPLETATO!"
echo "=========================================="
echo ""
echo "🚀 Per avviare il robot:"
echo "   ssh $RPI_USER@$RPI_HOST"
echo "   cd $RPI_DIR"
echo "   python3 test_robot.py        # Test preliminari"
echo "   python3 robot_controller.py  # Avvia missione"
echo ""
echo "📝 Per modificare configurazione:"
echo "   nano robot_controller.py"
echo "   # Modifica parametri in __init__"
echo ""
