import serial.tools.list_ports

port_list = serial.tools.list_ports.comports()

for port in port_list:
    print(f"{port.device} ({port.description}): {port.name}")