#!/usr/bin/env python3
"""
Configurazione rete per Raspberry Pi
Per uso con sistema Courier Robot ROS2
"""
import subprocess
import time
import sys

# ============================================================================
# CONFIGURAZIONE - MODIFICA QUESTI VALORI
# ============================================================================
INTERFACE = "wlan0"  # o "wlp2s0" su alcuni Pi
SSID = "TuoSSID"  # ⚠️ MODIFICA CON IL TUO SSID
PASSWORD = "TuaPassword"  # ⚠️ MODIFICA CON LA TUA PASSWORD

# IP Configuration
USE_DHCP = True  # True = DHCP, False = IP statico
STATIC_IP = "192.168.1.100/24"  # Usato solo se USE_DHCP = False
GATEWAY = "192.168.1.1"  # IP del router (modifica se necessario)
DNS = "8.8.8.8"  # DNS server (Google DNS di default)

# ============================================================================
# FUNZIONI
# ============================================================================

def run(cmd, check=False):
    """Esegue un comando shell e ritorna stdout, stderr, returncode"""
    print(f"🔧 Eseguo: {cmd}")
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    
    if check and result.returncode != 0:
        print(f"❌ Errore: {result.stderr}")
        sys.exit(1)
    
    return result.stdout.strip(), result.stderr.strip(), result.returncode


def check_interface():
    """Verifica che l'interfaccia esista"""
    out, _, _ = run("ip link show")
    if INTERFACE not in out:
        print(f"❌ Interfaccia {INTERFACE} non trovata!")
        print("Interfacce disponibili:")
        run("ip link show | grep -E '^[0-9]+:'")
        sys.exit(1)
    print(f"✅ Interfaccia {INTERFACE} trovata")


def connect_wifi_nmcli():
    """
    Connetti usando NetworkManager (metodo raccomandato per Ubuntu/ROS2)
    """
    print(f"📡 Connessione a {SSID} usando NetworkManager...")
    
    # Verifica se NetworkManager è attivo
    _, _, rc = run("systemctl is-active NetworkManager")
    if rc != 0:
        print("⚠️  NetworkManager non attivo. Uso wpa_supplicant...")
        return connect_wifi_wpa()
    
    # Rimuovi connessione esistente se presente
    run(f"sudo nmcli connection delete '{SSID}' 2>/dev/null")
    
    # Crea nuova connessione
    if USE_DHCP:
        cmd = f"sudo nmcli device wifi connect '{SSID}' password '{PASSWORD}' ifname {INTERFACE}"
    else:
        cmd = (
            f"sudo nmcli connection add type wifi con-name '{SSID}' "
            f"ifname {INTERFACE} ssid '{SSID}' -- "
            f"wifi-sec.key-mgmt wpa-psk wifi-sec.psk '{PASSWORD}' "
            f"ipv4.method manual ipv4.addresses {STATIC_IP} "
            f"ipv4.gateway {GATEWAY} ipv4.dns {DNS}"
        )
    
    out, err, rc = run(cmd)
    
    if rc == 0:
        print("✅ Connesso con NetworkManager")
        return True
    else:
        print(f"❌ Errore NetworkManager: {err}")
        return False


def connect_wifi_wpa():
    """
    Fallback: usa wpa_supplicant direttamente
    """
    print("📡 Connessione usando wpa_supplicant...")
    
    # Crea file di configurazione
    wpa_conf_content = f"""
ctrl_interface=/var/run/wpa_supplicant
update_config=1

network={{
    ssid="{SSID}"
    psk="{PASSWORD}"
    key_mgmt=WPA-PSK
}}
"""
    
    conf_path = "/tmp/wpa_supplicant_courier.conf"
    try:
        with open(conf_path, "w") as f:
            f.write(wpa_conf_content)
        print(f"✅ File configurazione creato: {conf_path}")
    except Exception as e:
        print(f"❌ Errore scrittura config: {e}")
        return False
    
    # Stop wpa_supplicant esistente
    run(f"sudo killall wpa_supplicant 2>/dev/null")
    time.sleep(1)
    
    # Porta su l'interfaccia
    run(f"sudo ip link set {INTERFACE} up", check=True)
    
    # Avvia wpa_supplicant
    cmd = f"sudo wpa_supplicant -B -i {INTERFACE} -c {conf_path} -D nl80211,wext"
    _, _, rc = run(cmd)
    
    if rc != 0:
        print("❌ wpa_supplicant fallito")
        return False
    
    print("⏳ Attendo associazione WiFi...")
    time.sleep(10)
    
    # Verifica associazione
    out, _, _ = run(f"iw dev {INTERFACE} link")
    if "Connected to" not in out:
        print("❌ Associazione WiFi fallita")
        return False
    
    print("✅ Associato al WiFi")
    
    # Configura IP
    if USE_DHCP:
        print("🔄 Richiedo IP via DHCP...")
        run(f"sudo dhclient -v {INTERFACE}")
    else:
        print(f"🔧 Imposto IP statico: {STATIC_IP}")
        run(f"sudo ip addr flush dev {INTERFACE}")
        run(f"sudo ip addr add {STATIC_IP} dev {INTERFACE}")
        run(f"sudo ip route add default via {GATEWAY}")
        run(f"echo 'nameserver {DNS}' | sudo tee /etc/resolv.conf")
    
    return True


def test_connection():
    """Testa la connessione pingando il gateway e Google DNS"""
    print("\n🧪 Test connessione...")
    
    # Test 1: Ping al gateway
    print(f"1️⃣ Ping al gateway ({GATEWAY})...")
    out, _, rc = run(f"ping -c 3 -W 2 {GATEWAY}")
    if rc == 0:
        print("✅ Gateway raggiungibile")
    else:
        print("❌ Gateway NON raggiungibile")
        return False
    
    # Test 2: Ping a Internet
    print("2️⃣ Ping a Google DNS (8.8.8.8)...")
    out, _, rc = run("ping -c 3 -W 2 8.8.8.8")
    if rc == 0:
        print("✅ Internet raggiungibile")
    else:
        print("⚠️  Internet NON raggiungibile (potrebbe essere un problema DNS)")
    
    # Mostra IP assegnato
    out, _, _ = run(f"ip -4 addr show {INTERFACE}")
    print(f"\n📋 Configurazione attuale:\n{out}")
    
    return True


def show_status():
    """Mostra lo stato della connessione"""
    print("\n" + "="*60)
    print("📊 STATO RETE")
    print("="*60)
    
    # IP address
    out, _, _ = run(f"ip -4 addr show {INTERFACE} | grep inet")
    print(f"IP Address: {out}")
    
    # WiFi info
    out, _, _ = run(f"iw dev {INTERFACE} link")
    print(f"\nWiFi Status:\n{out}")
    
    # Route
    out, _, _ = run("ip route")
    print(f"\nRouting Table:\n{out}")


# ============================================================================
# MAIN
# ============================================================================

def main():
    print("="*60)
    print("🌐 CONFIGURAZIONE RETE RASPBERRY PI")
    print("="*60)
    
    # Verifica di essere root
    out, _, _ = run("whoami")
    if out != "root":
        print("⚠️  Questo script richiede privilegi root")
        print("Esegui con: sudo python3 config_rete.py")
        sys.exit(1)
    
    # Verifica interfaccia
    check_interface()
    
    # Connetti al WiFi
    success = connect_wifi_nmcli()
    
    if not success:
        print("\n❌ Connessione WiFi fallita!")
        print("\n📝 Controlla:")
        print(f"  - SSID corretto: '{SSID}'")
        print(f"  - Password corretta")
        print(f"  - Interfaccia corretta: {INTERFACE}")
        sys.exit(1)
    
    # Attendi stabilizzazione
    print("\n⏳ Attendo stabilizzazione rete...")
    time.sleep(5)
    
    # Test connessione
    if test_connection():
        show_status()
        print("\n" + "="*60)
        print("✅ RETE CONFIGURATA CORRETTAMENTE!")
        print("="*60)
        print(f"\n💡 Puoi connetterti via SSH:")
        out, _, _ = run(f"hostname -I | awk '{{print $1}}'")
        print(f"   ssh ubuntu@{out}")
        print("\n🚀 Pronto per avviare il sistema ROS2!")
    else:
        print("\n❌ Test di connessione fallito!")
        sys.exit(1)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n⚠️  Interrotto dall'utente")
        sys.exit(0)
    except Exception as e:
        print(f"\n❌ Errore: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
