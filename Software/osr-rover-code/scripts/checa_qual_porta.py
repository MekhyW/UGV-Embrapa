import serial.tools.list_ports

def find_ws2000_port():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if "0483:572b" in port.hwid:  # Substitua pelo Vendor ID e Product ID do seu dispositivo
            return port.device
    return None

serial_port = find_ws2000_port()
if serial_port:
    print(f"Receiver WS2000 encontrado na porta: {serial_port}")
else:
    print("Receiver WS2000 não encontrado.")
