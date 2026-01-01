"""
================================================================================
SCRIPT: Secure Firmware Packager (AES-256 + SHA-256 + CRC32)
AUTHOR: Harun Karaca
DATE:   December 2025
DESCRIPTION:
    Takes a raw binary firmware file (app.bin), applies AES-256 encryption,
    calculates CRC32 integrity checksums, appends a digital signature,
    and prepends a structured 64-byte header for the STM32 Bootloader.
================================================================================
"""

import struct
import zlib
import os
from Cryptodome.Cipher import AES
from Cryptodome.Hash import SHA256
from Cryptodome.Util.Padding import pad

# --- CONFIGURATION (MUST MATCH STM32 MAIN.C DEFINITIONS) ---
INPUT_FILE  = "app.bin"
OUTPUT_FILE = "secure_app.bin"

# Header Metadata
MAGIC_BYTES = 0x53454355  # "SECU" (Little Endian for STM32)
FW_VERSION  = 106         # Firmware Version

# Security Keys (AES-256-CBC)
# NOTE: Ensure these match the keys hardcoded in the STM32 Bootloader.
AES_KEY = bytes([0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF] * 4) # 32 Bytes
AES_IV  = bytes([0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 
                 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F]) # 16 Bytes

def generate_secure_package():
    """
    Reads the input binary, encrypts it, calculates checksums, 
    and generates the final secure blob.
    """
    print(f"[INFO] Starting Secure Package Generation for Version {FW_VERSION}...")

    try:
        # 1. Read Raw Firmware
        if not os.path.exists(INPUT_FILE):
            print(f"[ERROR] Input file '{INPUT_FILE}' not found.")
            return

        with open(INPUT_FILE, 'rb') as f:
            raw_payload = f.read()
        
        print(f"[INFO] Raw Firmware Size: {len(raw_payload)} bytes")

        # 2. Encryption (AES-256-CBC with PKCS7 Padding)
        cipher = AES.new(AES_KEY, AES.MODE_CBC, AES_IV)
        encrypted_payload = cipher.encrypt(pad(raw_payload, 16))
        
        enc_size = len(encrypted_payload)
        print(f"[INFO] Encrypted Payload Size: {enc_size} bytes")

        # 3. Integrity Check (CRC32)
        # We calculate CRC on the ENCRYPTED payload because that is what stored in Flash.
        # This fixes the "PY:0" issue.
        crc32_val = zlib.crc32(encrypted_payload) & 0xFFFFFFFF
        print(f"[INFO] Calculated CRC32: {hex(crc32_val)}")

        # 4. Digital Signature (SHA-256)
        # Hashing the encrypted payload ensures authenticity.
        signature = SHA256.new(encrypted_payload).digest()

        # 5. Header Construction (64 Bytes)
        # Structure: [Magic(4)] [Ver(4)] [Size(4)] [CRC(4)] [Sig(32)] [Padding(16)]
        # Format '<IIII': Little Endian, 4x Unsigned Int
        header = struct.pack('<IIII', MAGIC_BYTES, FW_VERSION, enc_size, crc32_val)
        
        # Append Signature (32 bytes)
        header += signature
        
        # Pad Header to exactly 64 bytes
        header = header.ljust(64, b'\x00')

        # 6. Write Final Package
        with open(OUTPUT_FILE, 'wb') as f:
            f.write(header + encrypted_payload)
        
        print("-" * 60)
        print(f"[SUCCESS] Secure Firmware Generated: {OUTPUT_FILE}")
        print(f"[METADATA] Magic: {hex(MAGIC_BYTES)} | Ver: {FW_VERSION} | Size: {enc_size} | CRC: {hex(crc32_val)}")
        print("-" * 60)

    except Exception as e:
        print(f"[CRITICAL ERROR] Process Failed: {e}")

if __name__ == "__main__":
    generate_secure_package()
