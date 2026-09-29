import pandas as pd
import os
import argparse

def hitung_evaluasi(csv_path):
    if not os.path.exists(csv_path):
        print(f"[!] File {csv_path} tidak ditemukan.")
        return

    try:
        df = pd.read_csv(csv_path)
    except Exception as e:
        print(f"[!] Gagal membaca file CSV: {e}")
        return

    # Pastikan kolom yang dibutuhkan ada
    required_cols = ["NWA_Phantom_Terdeteksi", "Anomali_Asli_Manual"]
    for col in required_cols:
        if col not in df.columns:
            print(f"[!] Kolom {col} tidak ditemukan di CSV.")
            return

    # Cek apakah pengguna sudah mengisi kolom manual
    # Jika semua masih kosong/NaN, beritahu untuk mengisi
    if df["Anomali_Asli_Manual"].isnull().all():
        print("[!] Belum ada data Anomali_Asli_Manual yang diisi.")
        print("    Silakan buka file CSV ini di Excel, lalu isi angka 1 untuk baris yang merupakan Anomali Asli, dan 0 untuk yang bukan.")
        return

    # Isi NaN dengan 0 dan ubah ke integer
    df["Anomali_Asli_Manual"] = pd.to_numeric(df["Anomali_Asli_Manual"], errors='coerce').fillna(0).astype(int)
    df["NWA_Phantom_Terdeteksi"] = pd.to_numeric(df["NWA_Phantom_Terdeteksi"], errors='coerce').fillna(0).astype(int)

    # ---------------------------------------------------------
    # PERHITUNGAN SESUAI RUMUS SKRIPSI (MANUAL GROUND TRUTH)
    # ---------------------------------------------------------
    y_true = df["Anomali_Asli_Manual"].values
    y_pred = df["NWA_Phantom_Terdeteksi"].values

    # Total Titik Semu Terdeteksi (Ground Truth Anomali yang ada di data)
    total_titik_semu_terdeteksi = sum(y_true)
    
    # Jumlah Titik Semu Terbuang (Anomali yang BERHASIL diidentifikasi NWA) -> True Positive
    jumlah_titik_semu_terbuang = sum((y_true == 1) & (y_pred == 1))
    
    # Menghitung False Positive (Kesalahan NWA membuang titik benar) - opsional sebagai info
    salah_buang = sum((y_true == 0) & (y_pred == 1))

    if total_titik_semu_terdeteksi > 0:
        tingkat_keberhasilan = (jumlah_titik_semu_terbuang / total_titik_semu_terdeteksi) * 100
    else:
        tingkat_keberhasilan = 0.0

    print("==================================================")
    print("  HASIL PERHITUNGAN SKRIPSI (LABEL MANUAL EXCEL)  ")
    print("==================================================")
    print(f"Total Titik Keseluruhan         : {len(df)}")
    print(f"Total Titik Semu Terdeteksi (A) : {total_titik_semu_terdeteksi}")
    print(f"Jumlah Titik Semu Terbuang  (B) : {jumlah_titik_semu_terbuang}")
    print("--------------------------------------------------")
    
    if total_titik_semu_terdeteksi == 0:
        print("[!] Catatan: Anda mengisi kolom manual dengan 0 semua, artinya tidak ada anomali.")
    
    print(f"Tingkat Keberhasilan (%)        = (B / A) * 100%")
    if total_titik_semu_terdeteksi > 0:
        print(f"                                = ({jumlah_titik_semu_terbuang} / {total_titik_semu_terdeteksi}) * 100%")
    print(f"                                = {tingkat_keberhasilan:.2f} %\n")
    
    print("Tambahan Info Evaluasi Algoritma:")
    print(f"- Anomali yang GAGAL dibuang (False Negative)           : {total_titik_semu_terdeteksi - jumlah_titik_semu_terbuang}")
    print(f"- Titik Valid yang SALAH dibuang NWA (False Positive)   : {salah_buang}")
    print("==================================================")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Hitung Evaluasi NWA dari CSV manual")
    parser.add_argument("--csv", type=str, help="Path ke file evaluasi_nwa.csv", required=True)
    args = parser.parse_args()

    hitung_evaluasi(args.csv)
