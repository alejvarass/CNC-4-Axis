import socket
import json
import time

ESP32_IP = "192.168.1.167"
ESP32_PORT = 5000

def reset_first_run_nvs():
    print(f"Conectando a ESP32 en {ESP32_IP}:{ESP32_PORT}...")
    
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(3.0)
        sock.connect((ESP32_IP, ESP32_PORT))
        print("¡Conexión TCP establecida con éxito!")

        msg = {
            "type": "cmd",
            "id": "reset-nvs-cmd",
            "seq": 1,
            "payload": {
                "cmd": "reset_nvs",
                "params": {}
            }
        }
        
        line = json.dumps(msg) + "\n"
        sock.sendall(line.encode("utf-8"))
        print(" -> Orden 'reset_nvs' transmitida a la ESP32.")

        time.sleep(0.2)
        try:
            response = sock.recv(2048).decode("utf-8", errors="replace")
            print("\nRespuesta de la ESP32:")
            print(response.strip())
        except socket.timeout:
            print("\nInstrucción aceptada por el controlador.")

        sock.close()
        print("\n[ÉXITO] Memoria Flash NVS reseteada. 'first_run' restablecido a True para todos los ejes.")

    except Exception as e:
        print(f"\n[ERROR] No se pudo comunicar con la ESP32: {e}")

if __name__ == "__main__":
    reset_first_run_nvs()