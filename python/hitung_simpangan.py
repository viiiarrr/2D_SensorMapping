import argparse

def hitung_simpangan(dimensi_peta, dimensi_asli):
    # Rumus: ((Dimensi_peta - Dimensi_asli) / Dimensi_asli) * 100%
    # Menggunakan absolut agar error selalu bernilai positif (seberapa melesetnya)
    # Anda bisa menghapus abs() jika ingin melihat error negatif (artinya peta lebih kecil dari asli)
    simpangan = ((dimensi_peta - dimensi_asli) / dimensi_asli) * 100
    return simpangan

if __name__ == "__main__":
    print("==================================================")
    print("       KALKULATOR SIMPANGAN DIMENSI (SKRIPSI)     ")
    print("==================================================")
    print("Ketik 'q' atau 'exit' kapan saja untuk keluar.\n")
    
    while True:
        try:
            peta_input = input("Masukkan Dimensi_peta (Hasil dari Terminal / cm): ")
            if peta_input.lower() in ['q', 'exit']: break
            
            asli_input = input("Masukkan Dimensi_asli (Hasil ukur meteran fisik / cm): ")
            if asli_input.lower() in ['q', 'exit']: break
            
            peta = float(peta_input)
            asli = float(asli_input)
            
            if asli == 0:
                print("[!] Dimensi asli tidak boleh 0.\n")
                continue
                
            simpangan = hitung_simpangan(peta, asli)
            
            print("\n--------------------------------------------------")
            print(f"Dimensi Peta Digital : {peta} cm")
            print(f"Dimensi Fisik Asli   : {asli} cm")
            print(f"Simpangan Dimensi (%) = (({peta} - {asli}) / {asli}) * 100%")
            print(f"                      = {simpangan:+.2f} %")
            print("--------------------------------------------------\n")
            
        except ValueError:
            print("[!] Harap masukkan angka yang valid (gunakan titik untuk desimal).\n")
