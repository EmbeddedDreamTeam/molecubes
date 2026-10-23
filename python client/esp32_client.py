"""
ESP32-C3 Servo Controller — Python Client (Fully Dynamic Edition)
------------------------------------------
- Si adatta dinamicamente a QUALSIASI numero di servi.
- Supporta parametri opzionali: Velocità, Accelerazione e Jerk.
- Tronca o riempie gli angoli in automatico se la sequenza non matcha i servi fisici.
"""

import socket
import threading
import sys
import time

# ===========================================================================
# CONFIGURAZIONE PRINCIPALE E VALORI DI DEFAULT
# ===========================================================================
ESP_HOST = "192.168.4.1"
ESP_PORT = 3333

AUTO_MODE = False

# Valori di default per ogni movimento
DEFAULT_SPEED = 1.0
DEFAULT_ACC = 100.0
DEFAULT_JERK = 1500.0
# ===========================================================================

num_servos = None

ack_event = threading.Event()   
ready_event = threading.Event() 

# Puoi lasciare la sequenza a 4, a 6 o a 2: il codice si adatterà da solo
# al numero reale di servi connessi scartando gli extra o aggiungendo zeri.
TARGET_SEQUENCE = [
    [0.0, 0, 0, 0],
    [-139, -139, -139, -139],
    [-139, -60, 0, 0],
    [-139, -60, -60, -139],
    [139, -60, -60, 139],
    [139, -90, 139, 139],
    [139, -90, 139, 0],
    [-139, -90, 100, 0],
    [-139, 0, -120, 0],
    [0, 0, 120, 0],
    [0, 0, 120, 139],
    [0, 60, 60, 139],
    [0, 60, 60, -139],
    [139, 60, 139, -139],
    [139, 60, 139, 139],
    [-139, 60, -139, 139],
    [-139, 60, -139, -139],
    [139, 60, 139, -139],
    [139, 60, 139, 139],
    [139, 120, 0, 139],
    [139, 120, 0, 0],
    [-139, 120, 0, 0],
    [-139, 120, 120, 0],
    [-139, 0, 120, 120],
    [-139, 60, -60, 120],
    [-139, 60, -60, -139],
    [139, 60, -60, -139],
    [0, 0, -60, -139],
    [0, 0, 0, 0]
]


# Sequenza di test per la Cinematica: [Angolo1, Angolo2, Angolo3, Angolo4, Vel, Acc, Jerk]
TARGET_SEQUENCE_2 = [
    # 1. Reset iniziale: torna a 0 con i valori di default
    [0, 0, 0, 0],

    # 2. MOVIMENTO LENTO E FLUIDO (Slow Motion)
    # Va a 120°. Velocità molto bassa, accelera piano, curva morbida.
    [120, 120, 120, 120, 0.3, 30.0, 100.0],

    # 3. Ritorno a 0 NORMALE 
    # Usiamo valori medi per avere un termine di paragone
    [0, 0, 0, 0, 1.5, 60.0, 800.0],

    # 4. MOVIMENTO AGGRESSIVO E SCATTANTE (Limite Hardware)
    # Va a -120°. Spinge i parametri al MASSIMO consentito dal C++ (5.2, 100, 1500)
    [-120, -120, -120, -120, 5.2, 100.0, 1500.0],

    # 5. Ritorno a 0 NORMALE
    [0, 0, 0, 0, 1.5, 60.0, 800.0],

    # 6. EFFETTO "RINCORSA" (Velocità Massima, Accelerazione Minima)
    # Va a 130°. I motori partono lentissimi ma puntano alla velocità massima (5.2)
    [130, 130, 130, 130, 5.2, 15.0, 300.0],

    # 7. Ritorno a 0 FINALE ESTREMAMENTE MORBIDO (S-Curve perfetta)
    # Torna a casa con estrema calma.
    [0, 0, 0, 0, 0.8, 40.0, 80.0]
]

# ---------------------------------------------------------------------------
# Funzione intelligente e Adattiva
# ---------------------------------------------------------------------------
def build_payload(values: list[float], num_servos: int, is_sequence: bool = False) -> str | None:
    """
    Costruisce la stringa finale scalando su QUALUNQUE numero di servi.
    """
    n = len(values)
    
    # CASO 1: Dati espliciti per ogni singolo servo (es: N servi -> N * 4 valori)
    if n == num_servos * 4:
        return " ".join(str(v) for v in values)
        
    # CASO 2: Angoli (+ opzionali globali)
    # Estraiamo esattamente 'num_servos' angoli dall'input
    angles = values[:num_servos]
    
    # Se l'input ha MENO angoli del numero di servi, riempiamo con 0 (padding)
    while len(angles) < num_servos:
        angles.append(0.0)
        
    # Se stiamo leggendo dalla TARGET_SEQUENCE automatica, ignoriamo eventuali numeri 
    # extra come se fossero velocità/acc (evita bug se hai 3 servi ma la tupla è da 4)
    if is_sequence:
        speed, acc, jerk = DEFAULT_SPEED, DEFAULT_ACC, DEFAULT_JERK
    else:
        # Modalità Manuale: i numeri che avanzano sono interpretati come Vel, Acc, Jerk
        extra_params = values[num_servos:] if n > num_servos else []
        
        if len(extra_params) > 3:
            return None # Errore: Troppi numeri inseriti a caso
            
        speed = extra_params[0] if len(extra_params) > 0 else DEFAULT_SPEED
        acc   = extra_params[1] if len(extra_params) > 1 else DEFAULT_ACC
        jerk  = extra_params[2] if len(extra_params) > 2 else DEFAULT_JERK
        
    # Assembliamo i blocchi da 4
    parts = []
    for a in angles:
        parts.append(f"{a} {speed} {acc} {jerk}")
        
    return " ".join(parts)


# ---------------------------------------------------------------------------
# Receiver thread
# ---------------------------------------------------------------------------
def receiver(sock: socket.socket) -> None:
    global num_servos
    buffer = ""

    while True:
        try:
            chunk = sock.recv(1024).decode("utf-8")
            if not chunk:
                print("\n[disconnected] L'ESP32 ha chiuso la connessione.")
                ready_event.set() 
                ack_event.set()
                sys.exit(0)

            buffer += chunk
            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                line = line.strip()
                if not line:
                    continue

                line_up = line.upper()
                if "OK" in line_up or "ACK" in line_up or "DONE" in line_up:
                    ack_event.set()

                if line.startswith("SERVOS "):
                    try:
                        num_servos = int(line.split()[1])
                        print(f"\n[esp32] Rilevati {num_servos} servi connessi.")
                        ready_event.set()
                    except (IndexError, ValueError):
                        print(f"\n[esp32] {line}")
                else:
                    print(f"\n[esp32] {line}")

        except OSError:
            print("\n[disconnected] Connessione persa.")
            sys.exit(0)


# ---------------------------------------------------------------------------
# Funzione Sequenza Automatica
# ---------------------------------------------------------------------------
def run_sequence(sock: socket.socket) -> None:
    print("\n[sequence] Avvio sequenza automatica tra 5 secondi...")
    time.sleep(5.0) 
    
    for step_idx, step_values in enumerate(TARGET_SEQUENCE):
        
        # is_sequence=True garantisce che una riga da 4 non "inquini" la velocità di un set a 3 motori
        payload = build_payload(step_values, num_servos, is_sequence=True)
        
        if payload is None:
            continue
            
        message = payload + "\n"
        ack_event.clear()
        
        print(f"[sequence] Invio step {step_idx + 1}/{len(TARGET_SEQUENCE)}: {message.strip()}")
        sock.sendall(message.encode("utf-8"))
        
        ack_received = ack_event.wait(timeout=15.0)
        
        if not ack_received:
            print(f"[error] Timeout! Nessun ACK ricevuto per lo step {step_idx + 1}. Interrompo.")
            break
            
        print("[sequence] ACK ricevuto! Procedo...\n")
        time.sleep(3)

    print("[sequence] Test automatico completato!")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main() -> None:
    global num_servos
    
    print(f"Connessione all'ESP32 su {ESP_HOST}:{ESP_PORT} ...")

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((ESP_HOST, ESP_PORT))
    except Exception as e:
        print(f"[error] Impossibile connettersi: {e}")
        sys.exit(1)

    print("Connesso. In attesa dei dati di avvio dall'ESP32...\n")

    t = threading.Thread(target=receiver, args=(sock,), daemon=True)
    t.start()

    is_ready = ready_event.wait(timeout=5.0)
    
    # SE NON RICEVE IL DATO DALL'ESP32, CHIEDE ALL'UTENTE
    if not is_ready or num_servos is None:
        print("\n[warning] Non ho ricevuto il setup iniziale dall'ESP32.")
        while True:
            try:
                ans = input("Quanti servi fisici sono collegati? (Inserisci un numero intero) > ")
                num_servos = int(ans)
                break
            except ValueError:
                print("Per favore inserisci un numero valido.")

    # ---------------------------------------------------------
    # DIRAMAZIONE LOGICA BASATA SULLA FLAG
    # ---------------------------------------------------------
    if AUTO_MODE:
        run_sequence(sock)
        print("Chiudo la connessione.")
        sock.close()
        sys.exit(0)
        
    else:
        try:
            while True:
                
                print("\n=======================================================")
                print(f"Numero di moduli collegati: {num_servos}")
                print("Scegli l'operazione da fare:")
                print(" - 1: Inserire comandi manualmente")
                print(" - 2: Caricare un file di comandi da eseguire")
                print(" - q: Uscire")
                # print(f" Inserisci {num_servos} angoli separati da spazio.")
                # print(" Opzionale: aggiungi Vel, Acc e Jerk alla fine per sovrascrivere i default.")
                # print(f" Esempi per {num_servos} servi:")
                # print(f"   Solo angoli:   {' '.join(['90'] * num_servos)}")
                # print(f"   Con Vel:       {' '.join(['90'] * num_servos)} 3.5")
                # print(f"   Con Vel+Acc:   {' '.join(['90'] * num_servos)} 3.5 200")
                # print(" Scrivi 'quit' per uscire.")
                # print("=======================================================\n")
                raw = input("> ").strip()
                

                if raw.lower() in ("quit", "exit", "q"):
                    print("Chiudo la connessione.")
                    sock.close()
                    sys.exit(0)
                elif raw.lower() in ("1"):
                    print(f" Numero di moduli collegati: {num_servos}")
                    print("Lista di comandi accettati:")
                    print("- G6 [R] N[posizione_modulo] P[angolo in gradi] [Svelocità(rad/s)] [Aaccelerazione(rad/s^2)] [Jjerk(rad/s^3)]")
                    print("    La sezione [N P S A J] può essere ripetuta più volte dopo G6 per comandare più moduli contemporaneamente.")
                    print("    S, A e J sono parametri opzionali. Se omessi, verranno usati i valori di default. Quando usati devono essere" \
                    "specificati nell'ordine corretto, ma non necessariamente tutti.")
                    print("    Esempio: G6 N0 P90.0 S1.0 A2.0 J3.0 N1 P45.0 J1.5 N2 P45.0 J1.5 N3 P45.0 J1.5")
                    print("- M222 [nuova velocità di default(rad/s)]")
                    print("- M204 [nuova accelerazione di default(rad/s^2)]")
                    print("- M205 [nuovo jerk di default(rad/s^3)]")
                    print("    Nota: Questi comandi sono intesi per modificare i valori di default una volta sola, non" \
                    "per cambiare i parametri per ogni comando specifico")
                    print("- G4 [attesa in millisecondi]")
                    print("- M24")
                    print("    Start, avvia la sequenza caricata")
                    print("- M25")
                    print("    Stop, interrompe la sequenza in esecuzione. Nota una volta che la sequenza termina bisogna "\
                    "inviare nuovamente il comando M24 per eseguire una nuova sequenza")
                    print("- M505")
                    print("    Svuota il buffer dei comandi in esecuzione.")
                    print("- q: torna al menù precedente")
                    while True:
                        raw = input("Inserisci il comando da caricare > ").strip()
                        if raw.lower() == "q":
                            break
                        else:
                            message = raw + "\n"
                            sock.sendall(message.encode("utf-8"))
                elif raw.lower() in ("2"):
                    while True:
                        file_path = input("Inserisci il percorso del file di comandi da eseguire > ").strip()
                        try:
                            with open(file_path, "r") as f:
                                commands = f.readlines()
                                for cmd in commands:
                                    cmd = cmd.strip()
                                    if cmd and not cmd.startswith("#"):  # Ignora righe vuote e commenti
                                        message = cmd + "\n"
                                        ack_event.clear()  # Resetta l'evento di ack
                                        sock.sendall(message.encode("utf-8"))
                                        ack_event.wait(timeout=5)  # attende l'ack prima di inviare il nuovo comando
                                        time.sleep(0.3) # attesa per evitare di triggerare il WDT
                        except FileNotFoundError:
                            print(f"[error] File non trovato: {file_path}")
                            continue
                        except Exception as e:
                            print(f"[error] Errore durante la lettura del file: {e}")
                            continue
                        break
                    print("Vuoi eseguire la sequenza? (y/n)")
                    choice = input().strip().lower()
                    if choice == "y":
                        sock.sendall("M24\n".encode("utf-8"))
                if not raw:
                    continue


        except (KeyboardInterrupt, EOFError):
            print("\nChiudo la connessione.")
            sock.close()
            sys.exit(0)

if __name__ == "__main__":
    main()