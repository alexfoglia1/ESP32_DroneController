import socket
import struct
import matplotlib.pyplot as plt
import time

# ---- Configurazione ----
PORT = 1234
BUFFER_SIZE = 1024
MAX_POINTS = 100

# ---- Setup UDP socket ----
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('', PORT))
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
sock.setblocking(False)

# ---- Buffer dati ----
t_buf = []
ax_buf, ay_buf, az_buf = [], [], []
gx_buf, gy_buf, gz_buf = [], [], []

# ---- Setup plot ----
plt.ion()
fig, (acc_ax, gyro_ax) = plt.subplots(2, 1, figsize=(10, 6))
acc_ax.set_title('Accelerometer (ax, ay, az)')
gyro_ax.set_title('Gyroscope (gx, gy, gz)')

line_ax, = acc_ax.plot([], [], 'r', label='ax')
line_ay, = acc_ax.plot([], [], 'g', label='ay')
line_az, = acc_ax.plot([], [], 'b', label='az')

line_gx, = gyro_ax.plot([], [], 'r', label='gx')
line_gy, = gyro_ax.plot([], [], 'g', label='gy')
line_gz, = gyro_ax.plot([], [], 'b', label='gz')

acc_ax.legend(loc='upper left')
gyro_ax.legend(loc='upper left')

start_time = time.monotonic()

print(f"In ascolto su UDP port {PORT}...")

while True:
    try:
        sock.sendto(bytearray([3]), ("192.168.1.9", 1234))
        data, _ = sock.recvfrom(BUFFER_SIZE)
        if len(data) < 12:
            continue

        # Decodifica i 3 valori float
        gx,gy,gz = struct.unpack('<fff', data[:12])
        
        #print("gx: {}, gy: {}, gz: {}".format(gx, gy, gz))
        
        #sock.sendto(bytearray([1]), ("172.20.10.12", 1234))
        #data, _ = sock.recvfrom(BUFFER_SIZE)
        #if len(data) < 12:
        #    continue
        #ax,ay,az = struct.unpack('<fff', data[:12])
        
        #print("ax: {}, ay: {}, az: {}".format(ax, ay, az))
        #continue
        now = time.monotonic() - start_time

        # Aggiunta dati ai buffer
        t_buf.append(now)
        
        #ax_buf.append(ax)
        #ay_buf.append(ay)
        #az_buf.append(az)
        gx_buf.append(gx)
        gy_buf.append(gy)
        gz_buf.append(gz)

        # Mantieni la dimensione massima
        for buf in [t_buf, ax_buf, ay_buf, az_buf, gx_buf, gy_buf, gz_buf]:
            if len(buf) > MAX_POINTS:
                buf.pop(0)

        # Aggiorna i dati dei plot
        #line_ax.set_data(t_buf, ax_buf)
        #line_ay.set_data(t_buf, ay_buf)
        #line_az.set_data(t_buf, az_buf)

        line_gx.set_data(t_buf, gx_buf)
        line_gy.set_data(t_buf, gy_buf)
        line_gz.set_data(t_buf, gz_buf)

        #acc_ax.relim()
        #acc_ax.autoscale_view()
        gyro_ax.relim()
        gyro_ax.autoscale_view()

        plt.tight_layout()
        plt.pause(0.01)

    except BlockingIOError:
        plt.pause(0.01)
    except KeyboardInterrupt:
        print("\nInterrotto dall'utente. Uscita.")
        break
