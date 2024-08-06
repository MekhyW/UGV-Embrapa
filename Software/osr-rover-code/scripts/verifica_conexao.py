import os
import time
import subprocess

USB_VENDOR_ID = "0483"
USB_PRODUCT_ID = "572b"
INTERVAL = 5

def check_usb_device(vendor_id, product_id):
    result = subprocess.run(['lsusb'], stdout=subprocess.PIPE)
    return f"{vendor_id}:{product_id}" in result.stdout.decode('utf-8')

def stop_rover_processes():
    # Substitua "programa_do_rover" pelo nome dos processos que você deseja parar
    os.system("pkill -f osr_launch")

if __name__ == "__main__":
    while True:
        if check_usb_device(USB_VENDOR_ID, USB_PRODUCT_ID):
            print("Receiver conectado.")
        else:
            print("Perda de sinal do receiver! Parando todos os processos...")
            stop_rover_processes()
            break
        time.sleep(INTERVAL)

