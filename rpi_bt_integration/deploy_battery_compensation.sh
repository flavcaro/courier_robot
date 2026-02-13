#!/bin/bash
# Script per deployment completo del sistema di compensazione batteria

RPI_USER="pi"
RPI_HOST="10.83.223.122"
RPI_BT_DIR="/home/pi/robot_code/bt"
RPI_MAIN_DIR="/home/pi/robot_code"
LOCAL_DIR="$(dirname "$0")"

echo "=========================================="
echo "🔋 DEPLOYMENT COMPENSAZIONE BATTERIA"
echo "=========================================="
echo ""

# Verifica connessione
echo "🔍 Verifica connessione..."
if ! ping -c 1 -W 2 "$RPI_HOST" &> /dev/null; then
    echo "❌ Impossibile raggiungere $RPI_HOST"
    exit 1
fi
echo "✅ Raspberry Pi raggiungibile"
echo ""

# Copia file Python modificati
echo "📤 Copia file Python..."
scp "$LOCAL_DIR/rover_API.py" "$RPI_USER@$RPI_HOST:$RPI_MAIN_DIR/"
scp "$LOCAL_DIR/bt/sensors.py" "$RPI_USER@$RPI_HOST:$RPI_BT_DIR/"
scp "$LOCAL_DIR/bt/navigation_actions.py" "$RPI_USER@$RPI_HOST:$RPI_BT_DIR/"
scp "$LOCAL_DIR/bt/actions.py" "$RPI_USER@$RPI_HOST:$RPI_BT_DIR/"
scp "$LOCAL_DIR/test_battery.py" "$RPI_USER@$RPI_HOST:$RPI_MAIN_DIR/"

if [ $? -eq 0 ]; then
    echo "✅ File Python copiati!"
else
    echo "❌ Errore copia file Python"
    exit 1
fi

echo ""
echo "=========================================="
echo "⚠️  IMPORTANTE: AGGIORNAMENTO ARDUINO"
echo "=========================================="
echo ""
echo "Il file RoverAPI.ino è stato modificato per supportare"
echo "la lettura della tensione della batteria."
echo ""
echo "📋 PASSI DA SEGUIRE:"
echo ""
echo "1. Collega l'Arduino al PC via USB"
echo "2. Apri Arduino IDE"
echo "3. Carica il file: $LOCAL_DIR/../RoverAPI.ino"
echo "4. Verifica le connessioni hardware:"
echo "   - Pin A0 collegato al divisore di tensione"
echo "   - Divisore 2:1 (es. 10kΩ + 10kΩ)"
echo "   - Batteria → R1 (10kΩ) → A0 → R2 (10kΩ) → GND"
echo "5. Carica il codice sull'Arduino (Ctrl+U)"
echo ""
echo "⚠️  ATTENZIONE: Se non usi un divisore di tensione,"
echo "   modifica VOLTAGE_DIVIDER_FACTOR nel file .ino"
echo ""
echo "=========================================="
echo "🧪 TEST SISTEMA"
echo "=========================================="
echo ""
echo "Dopo aver caricato il codice Arduino, testa il sistema:"
echo ""
echo "  ssh $RPI_USER@$RPI_HOST"
echo "  cd $RPI_MAIN_DIR"
echo "  python3 test_battery.py"
echo ""
echo "Questo script mostrerà:"
echo "  - Tensione batteria corrente"
echo "  - Fattore di compensazione"
echo "  - Tempo rotazione compensato"
echo "  - Stato batteria e autonomia stimata"
echo ""
echo "=========================================="
echo "✅ DEPLOYMENT PYTHON COMPLETATO"
echo "=========================================="
