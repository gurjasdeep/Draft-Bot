import math
import tkinter as tk
from tkinter.scrolledtext import ScrolledText

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None


BAUD_RATE = 115200
MAX_COMMAND_LENGTH = 63

root = tk.Tk()
root.title("DraftBot Controller")
root.geometry("850x650")
root.minsize(650, 500)

serial_port = None
response_buffer = ""
port_var = tk.StringVar()
center_x_var = tk.StringVar(value="90")
center_y_var = tk.StringVar(value="130")
size_var = tk.StringVar(value="40")
custom_command_var = tk.StringVar()
status_var = tk.StringVar(value="Select a serial port and connect.")


def set_status(message):
    status_var.set(message)


def write_monitor(text, tag=None):
    monitor.configure(state=tk.NORMAL)
    monitor.insert(tk.END, text, tag or ())
    monitor.see(tk.END)
    monitor.configure(state=tk.DISABLED)


def refresh_ports():
    if list_ports is None:
        set_status("Install PySerial with: python -m pip install -r requirements.txt")
        return

    ports = [port.device for port in list_ports.comports()]
    port_menu["menu"].delete(0, tk.END)
    for port_name in ports:
        port_menu["menu"].add_command(
            label=port_name,
            command=lambda name=port_name: port_var.set(name),
        )

    if port_var.get() not in ports:
        port_var.set(ports[0] if ports else "")
    if not ports:
        set_status("No serial ports found.")


def disconnect():
    global serial_port
    if serial_port is not None:
        try:
            serial_port.close()
        except OSError as error:
            write_monitor(f"[disconnect error] {error}\n", "error")
        serial_port = None
    connect_button.configure(text="Connect")
    connection_label.configure(text="Disconnected")


def close_window():
    disconnect()
    root.destroy()


def connect():
    global serial_port
    if serial is None:
        set_status("PySerial is not installed. See README.md for setup.")
        return

    if serial_port is not None:
        disconnect()
        set_status("Disconnected.")
        return

    port_name = port_var.get()
    if not port_name:
        set_status("Choose a serial port first.")
        return

    try:
        serial_port = serial.Serial(port_name, BAUD_RATE, timeout=0)
    except OSError as error:
        set_status(f"Could not connect: {error}")
        write_monitor(f"[connection error] {error}\n", "error")
        return

    connect_button.configure(text="Disconnect")
    connection_label.configure(text=f"Connected to {port_name} at {BAUD_RATE} baud")
    set_status("Connected. Send HOME or HOME TEST before moving.")


def send_command(command):
    command = command.strip()
    if not command:
        set_status("Command is empty.")
        return False
    if "\n" in command or "\r" in command:
        set_status("Enter one command at a time.")
        return False
    if len(command.encode("utf-8")) > MAX_COMMAND_LENGTH:
        set_status(f"Command is too long (maximum {MAX_COMMAND_LENGTH} bytes).")
        return False
    if serial_port is None or not serial_port.is_open:
        set_status("Connect to the Arduino before sending commands.")
        return False

    try:
        serial_port.write(command.encode("utf-8") + b"\n")
    except OSError as error:
        set_status(f"Send failed: {error}")
        write_monitor(f"[send error] {error}\n", "error")
        disconnect()
        return False

    write_monitor(f">> {command}\n", "sent")
    set_status(f"Sent: {command}")
    return True


def read_number(variable, name):
    try:
        value = float(variable.get())
    except ValueError as error:
        raise ValueError(f"{name} must be a number.") from error
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite.")
    return value


def format_number(value):
    return f"{value:g}"


def send_shape(shape_name):
    try:
        center_x = read_number(center_x_var, "Center X")
        center_y = read_number(center_y_var, "Center Y")
        size = read_number(size_var, "Size")
    except ValueError as error:
        set_status(str(error))
        return

    if size <= 0:
        set_status("Size must be greater than zero.")
        return

    send_command(
        f"SHAPE {shape_name} {format_number(center_x)} "
        f"{format_number(center_y)} {format_number(size)}"
    )


def send_move():
    try:
        x = read_number(center_x_var, "X")
        y = read_number(center_y_var, "Y")
    except ValueError as error:
        set_status(str(error))
        return
    send_command(f"MOVE {format_number(x)} {format_number(y)}")


def send_custom_command(event=None):
    _ = event
    if send_command(custom_command_var.get()):
        custom_command_var.set("")


def handle_response(line):
    message = line.strip()
    if message.startswith("ERROR "):
        set_status(f"Arduino error: {message[6:]}")
        write_monitor(f"[error] {message}\n", "error")
    elif message == "READY":
        set_status("Arduino is ready.")
    elif message == "OK HOME":
        set_status("Homing complete.")
    elif message == "OK PEN UP":
        set_status("Pen is up.")
    elif message == "OK PEN DOWN":
        set_status("Pen is down.")
    elif message.startswith("DONE "):
        set_status(f"Completed: {message[5:]}")
    elif message.startswith("OK ") and message.endswith(" STARTED"):
        set_status(f"Arduino accepted command: {message[3:]}")


def poll_serial():
    global response_buffer
    if serial_port is not None and serial_port.is_open:
        try:
            waiting = serial_port.in_waiting
            if waiting:
                text = serial_port.read(waiting).decode("utf-8", errors="replace")
                write_monitor(text)
                response_buffer += text

                while "\n" in response_buffer:
                    line, response_buffer = response_buffer.split("\n", 1)
                    handle_response(line)

                if len(response_buffer) > 256:
                    response_buffer = ""
        except OSError as error:
            write_monitor(f"\n[serial error] {error}\n", "error")
            set_status(f"Serial connection lost: {error}")
            disconnect()

    root.after(50, poll_serial)


menu_bar = tk.Menu(root)
actions_menu = tk.Menu(menu_bar, tearoff=False)
actions_menu.add_command(label="HOME TEST", command=lambda: send_command("HOME TEST"))
actions_menu.add_separator()
actions_menu.add_command(label="PEN UP", command=lambda: send_command("PEN UP"))
actions_menu.add_command(label="PEN DOWN", command=lambda: send_command("PEN DOWN"))
menu_bar.add_cascade(label="Actions", menu=actions_menu)
root.config(menu=menu_bar)

connection_frame = tk.LabelFrame(root, text="Serial connection", padx=8, pady=8)
connection_frame.pack(fill=tk.X, padx=10, pady=6)
port_var.set("")
port_menu = tk.OptionMenu(connection_frame, port_var, "")
port_menu.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 6))
tk.Button(connection_frame, text="Refresh", command=refresh_ports).pack(
    side=tk.LEFT, padx=4
)
connect_button = tk.Button(connection_frame, text="Connect", command=connect)
connect_button.pack(side=tk.LEFT, padx=4)
connection_label = tk.Label(connection_frame, text="Disconnected")
connection_label.pack(side=tk.LEFT, padx=8)

controls_frame = tk.LabelFrame(root, text="Shape and move", padx=8, pady=8)
controls_frame.pack(fill=tk.X, padx=10, pady=6)
tk.Label(controls_frame, text="Center X (mm)").grid(row=0, column=0, sticky=tk.W)
tk.Entry(controls_frame, textvariable=center_x_var, width=10).grid(
    row=0, column=1, padx=5
)
tk.Label(controls_frame, text="Center Y (mm)").grid(row=0, column=2, sticky=tk.W)
tk.Entry(controls_frame, textvariable=center_y_var, width=10).grid(
    row=0, column=3, padx=5
)
tk.Label(controls_frame, text="Size (mm)").grid(row=0, column=4, sticky=tk.W)
tk.Entry(controls_frame, textvariable=size_var, width=10).grid(
    row=0, column=5, padx=5
)
tk.Button(
    controls_frame,
    text="Square",
    command=lambda: send_shape("SQUARE"),
).grid(row=1, column=0, columnspan=2, sticky=tk.EW, padx=3, pady=(8, 2))
tk.Button(
    controls_frame,
    text="Equilateral triangle",
    command=lambda: send_shape("TRIANGLE"),
).grid(row=1, column=2, columnspan=2, sticky=tk.EW, padx=3, pady=(8, 2))
tk.Button(
    controls_frame,
    text="Circle",
    command=lambda: send_shape("CIRCLE"),
).grid(row=1, column=4, columnspan=2, sticky=tk.EW, padx=3, pady=(8, 2))
tk.Label(
    controls_frame,
    text="Size: square side, triangle height, or circle diameter.",
).grid(row=2, column=0, columnspan=6, sticky=tk.W, pady=(5, 0))
tk.Button(
    controls_frame,
    text="Move to X, Y",
    command=send_move,
).grid(row=3, column=0, columnspan=2, sticky=tk.EW, padx=3, pady=(8, 2))
tk.Label(controls_frame, textvariable=status_var, anchor=tk.W).grid(
    row=3, column=2, columnspan=4, sticky=tk.EW, padx=5, pady=(8, 2)
)

monitor_frame = tk.LabelFrame(root, text="Serial monitor", padx=6, pady=6)
monitor_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=6)
monitor = ScrolledText(
    monitor_frame,
    height=12,
    state=tk.DISABLED,
    background="#101820",
    foreground="#d7e2e8",
    insertbackground="#ffffff",
)
monitor.pack(fill=tk.BOTH, expand=True)
monitor.tag_configure("sent", foreground="#83d483")
monitor.tag_configure("error", foreground="#ff8585")

custom_frame = tk.LabelFrame(root, text="Custom command", padx=8, pady=8)
custom_frame.pack(fill=tk.X, padx=10, pady=(0, 10))
custom_entry = tk.Entry(custom_frame, textvariable=custom_command_var)
custom_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 6))
custom_entry.bind("<Return>", send_custom_command)
tk.Button(custom_frame, text="Send", command=send_custom_command).pack(side=tk.LEFT)

refresh_ports()
poll_serial()
root.protocol("WM_DELETE_WINDOW", close_window)

if __name__ == "__main__":
    root.mainloop()
