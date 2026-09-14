"""
server.py — Satu skrip untuk semua:
  1. HTTP server  → serve file web-visualisasi/
  2. WebSocket bridge → teruskan data UDP ESP32 ke browser
  3. Buka browser otomatis

Cara pakai:
    python python/server.py
    (atau jalankan.bat)

Dependensi:
    pip install websockets
"""

import asyncio
import socket
import threading
import webbrowser
import sys
import os
import time
import http.server
import functools
from pathlib import Path

# ─── Auto-install websockets jika belum ada ───────────────────
try:
    import websockets
except ImportError:
    print("[!] Library 'websockets' belum terinstall. Menginstall...")
    import subprocess
    subprocess.run([sys.executable, "-m", "pip", "install", "websockets"], check=True)
    import websockets

# ─── Konfigurasi ───────────────────────────────────────────────
ROOT      = Path(__file__).parent.parent          # folder TugasAkhir/
WEB_DIR   = ROOT / 'web-visualisasi'
UDP_IP    = "0.0.0.0"
UDP_PORT  = 5005
WS_PORT   = 8765
HTTP_PORT = 8080
BROWSER_URL = f"http://localhost:{HTTP_PORT}/web-visualisasi/"

# ─── Daftar client WebSocket yang terhubung ────────────────────
clients: set = set()


# ══════════════════════════════════════════════════════════════
#  HTTP SERVER  (serve seluruh folder TugasAkhir/)
# ══════════════════════════════════════════════════════════════

class SilentHandler(http.server.SimpleHTTPRequestHandler):
    """HTTP handler tanpa log di terminal."""
    def log_message(self, format, *args):
        pass  # Diam-diam saja


def run_http_server():
    handler = functools.partial(SilentHandler, directory=str(ROOT))
    server  = http.server.HTTPServer(("", HTTP_PORT), handler)
    server.serve_forever()


# ══════════════════════════════════════════════════════════════
#  WEBSOCKET HANDLER  (browser terhubung ke sini)
# ══════════════════════════════════════════════════════════════

async def ws_handler(websocket):
    clients.add(websocket)
    print(f"  [+] Browser terhubung  (total: {len(clients)})")
    try:
        await websocket.wait_closed()
    finally:
        clients.discard(websocket)
        print(f"  [-] Browser terputus   (total: {len(clients)})")


# ══════════════════════════════════════════════════════════════
#  UDP → WEBSOCKET BRIDGE & DATA LOGGER
# ══════════════════════════════════════════════════════════════

async def udp_bridge():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((UDP_IP, UDP_PORT))
    sock.setblocking(False)
    print(f"  [*] Mendengarkan ESP32 di UDP port {UDP_PORT}...")

    loop    = asyncio.get_event_loop()
    pkt_cnt = 0
    
    import csv
    import datetime
    csv_file = None
    csv_writer = None

    try:
        while True:
            try:
                data = await loop.sock_recv(sock, 1024)
                msg  = data.decode("utf-8").strip()

                pkt_cnt += 1
                if pkt_cnt % 50 == 0:
                    print(f"  [~] {pkt_cnt} paket diterima dari ESP32")
                    
                # Inisialisasi file CSV saat data pertama masuk
                if csv_file is None:
                    base = ROOT / "Data"
                    base.mkdir(exist_ok=True)
                    idx = 1
                    while (base / f"percobaan_{idx}").exists():
                        idx += 1
                    d = base / f"percobaan_{idx}"
                    d.mkdir()
                    csv_path = d / "koordinat.csv"
                    csv_file = open(csv_path, 'w', newline='', encoding='utf-8')
                    csv_writer = csv.writer(csv_file)
                    csv_writer.writerow(["Waktu", "Yaw_IMU"] + [f"Sensor_{i+1}" for i in range(8)])
                    print(f"  [+] MEREKAM DATA KE: Data/percobaan_{idx}/koordinat.csv")

                # Simpan ke CSV
                parts = msg.split(',')
                if len(parts) == 9:
                    time_str = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
                    csv_writer.writerow([time_str] + parts)
                    if pkt_cnt % 10 == 0:
                        csv_file.flush()

                if clients:
                    dead = set()
                    for c in clients.copy():
                        try:
                            await c.send(msg)
                        except Exception:
                            dead.add(c)
                    clients.difference_update(dead)

            except asyncio.CancelledError:
                break
            except Exception as e:
                if "10054" not in str(e):   # abaikan "connection reset" di Windows
                    print(f"  [!] UDP error: {e}")
    finally:
        if csv_file:
            csv_file.close()
            print("  [*] Menyinkronkan data ke Web (generate_web_data)...")
            import subprocess
            subprocess.run([sys.executable, str(ROOT / "python" / "generate_web_data.py")], cwd=str(ROOT))
            print("  [+] Selesai! Data sudah bisa dilihat di dropdown Web.")


# ══════════════════════════════════════════════════════════════
#  MAIN
# ══════════════════════════════════════════════════════════════

async def main():
    # 1. HTTP server di thread terpisah
    t = threading.Thread(target=run_http_server, daemon=True)
    t.start()

    # 2. Tunggu sebentar lalu buka browser
    await asyncio.sleep(0.6)
    webbrowser.open(BROWSER_URL)

    # 3. WebSocket server + UDP bridge berjalan bersamaan
    async with websockets.serve(ws_handler, "0.0.0.0", WS_PORT):
        print()
        print("=" * 52)
        print("  2D SENSOR MAPPER — SERVER AKTIF")
        print("=" * 52)
        print(f"  Web  : {BROWSER_URL}")
        print(f"  WS   : ws://0.0.0.0:{WS_PORT}")
        print(f"  UDP  : port {UDP_PORT}  (dari ESP32/Raspberry Pi)")
        print("=" * 52)
        print("  Tekan Ctrl+C untuk menghentikan\n")
        await udp_bridge()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[!] Server dihentikan.")
