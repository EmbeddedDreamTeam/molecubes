def main() -> None:
    while True:
        file_path = input("Inserisci il percorso del file di log > ").strip()
        try:
            with open(file_path, "r") as f:
                print("pos:")
                commands = f.readlines()
                for cmd in commands:
                    pos_index = cmd.find("Setting servo position: pos=")
                    if pos_index != -1:
                        position = cmd[pos_index:].split("pos=")[1].split()[0]
                        print(f"{position}")
        except FileNotFoundError:
            print(f"[error] File non trovato: {file_path}")
            continue
        except Exception as e:
            print(f"[error] Errore durante la lettura del file: {e}")
            continue
                    

if __name__ == "__main__":
    main()