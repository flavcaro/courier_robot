import socket
import time
from rover_API import RoverApi

# CONFIGURAZIONE
HOST = '0.0.0.0'  # Ascolta su tutti gli indirizzi
PORT = 65432      # Porta arbitraria (usiamo questa anche sul PC)

# Inizializza il Rover (Arduino)
# ATTENZIONE: Controlla se è ttyUSB0 o ttyACM0
try:
    rover = RoverApi('/dev/ttyUSB0')
    print("Arduino connesso con successo!")
except Exception as e:
    print(f"Errore connessione Arduino: {e}")
    exit()

def start_server():
    """Avvia server TCP per controllo remoto del robot."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen()
        print(f"Robot in ascolto su porta {PORT}...")

        conn, addr = s.accept()
        with conn:
            print(f"Connesso al PC: {addr}")
            
            while True:
                data = conn.recv(1024)
                if not data:
                    break
                
                # Processa comando ricevuto
                cmd = data.decode().strip()
                print(f"Comando ricevuto: {cmd}")
                
                # Invia comando al robot
                # TODO: Implementare parsing comandi
                
if __name__ == "__main__":
    start_server()
