import serial
import sys

PORT = '/dev/ttyUSB0' 
BAUD = 115200

def send_bl_command(cmd):
    try:
        with serial.Serial(PORT, BAUD, timeout=1) as ser:
            print(f"[>] Komut gönderiliyor: {hex(cmd)}")
            ser.write(bytes([cmd]))
            resp = ser.read(1)
            if resp:
                print(f"[<] Yanıt: {hex(resp[0])}")
                if resp[0] == 0xC1: print("[!] Sonuç: ACK (Başarılı)")
                elif resp[0] == 0x7F: print("[X] Sonuç: NACK (Hata)")
            else:
                print("[-] Yanıt alınamadı.")
    except Exception as e:
        print(f"[-] Port hatası: {e}")

if __name__ == "__main__":
    print("1: INIT (0xAB), 2: ERASE (0x50), 3: JUMP (0x52)")
    choice = input("Seçim: ")
    cmds = {'1': 0xAB, '2': 0x50, '3': 0x52}
    if choice in cmds: send_bl_command(cmds[choice])
