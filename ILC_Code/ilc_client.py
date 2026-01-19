#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ILC Command Client (Python)
Connects to localhost:8765 and provides an interactive prompt.

Presets:
  preset drift  -> error 0.25; noise off
  preset lag    -> error 0.45; noise off
  preset deform -> error 0.65; noise off
  preset noise  -> error 0.40; noise on
"""

import socket

SERVER_IP = "127.0.0.1"
SERVER_PORT = 8765

HELP_TEXT = """
=== ILC Command Client ===
Available commands:
  start                     - Start the simulation
  stop                      - Stop the simulation
  reset                     - Reset ILC (clear learning)
  error <level>             - Set system error level (0.0 to 1.0)
  lr <rate>                 - Set learning rate (0.1 to 0.8)
  smooth <alpha>            - Set smoothing factor (0.1 to 1.0)
  noise on|off              - Enable/disable random noise

  shape <type> [args]       - Change reference shape
     circle [radius]
     ellipse [a b]
     square [side]
     star [outer inner]

  dome start|stop|reset|status - Control hemisphere dome building

  status                    - Show current simulation status
  plot3d [dz]               - Start/refresh live 3D trajectory view
  stl [filename]            - Export trajectories as STL
  help                      - Show this help
  quit                      - Exit client

Preset Scenarios:
  preset drift   - Mild drift error (error 0.25, noise off)
  preset lag     - Phase lag error (error 0.45, noise off)
  preset deform  - Severe deformation (error 0.65, noise off)
  preset noise   - Stochastic disturbance (error 0.4, noise on)
"""


class ILCClient:
    def __init__(self):
        self.sock = None

    def connect(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((SERVER_IP, SERVER_PORT))
        print(f"Connected to ILC simulator on {SERVER_IP}:{SERVER_PORT}")

    def disconnect(self):
        if self.sock:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None

    def send(self, cmd: str) -> str:
        if not self.sock:
            return "ERROR: Not connected\n"
        try:
            self.sock.sendall((cmd.strip() + "\n").encode("utf-8"))
            data = self.sock.recv(4096)
            return data.decode("utf-8", errors="ignore")
        except OSError:
            self.disconnect()
            return "ERROR: Connection lost\n"


def print_help():
    print(HELP_TEXT)


def apply_preset(client: ILCClient, name: str):
    name = name.lower()
    if name == "drift":
        print(client.send("error 0.25"), end="")
        print(client.send("noise off"), end="")
        print("Preset 'drift' applied: 25% error, no noise")
    elif name == "lag":
        print(client.send("error 0.45"), end="")
        print(client.send("noise off"), end="")
        print("Preset 'lag' applied: 45% error, no noise")
    elif name == "deform":
        print(client.send("error 0.65"), end="")
        print(client.send("noise off"), end="")
        print("Preset 'deform' applied: 65% error, no noise")
    elif name == "noise":
        print(client.send("error 0.4"), end="")
        print(client.send("noise on"), end="")
        print("Preset 'noise' applied: 40% error, noise enabled")
    else:
        print("Unknown preset. Available: drift, lag, deform, noise")


def main():
    print("=== ILC Simulator Command Client (Python) ===")
    print("Connecting to simulator...")
    client = ILCClient()
    try:
        client.connect()
    except OSError:
        print("\nFailed to connect. Make sure the simulator is running.")
        return

    print_help()

    while True:
        try:
            line = input("\nilc> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nDisconnecting...")
            break

        if not line:
            continue

        if line in ("quit", "exit"):
            print("Disconnecting...")
            break

        if line in ("help", "?"):
            print_help()
            continue

        if line.startswith("preset"):
            parts = line.split()
            if len(parts) >= 2:
                apply_preset(client, parts[1])
            else:
                print("Usage: preset <drift|lag|deform|noise>")
            continue

        resp = client.send(line)
        print(resp, end="")

    client.disconnect()


if __name__ == "__main__":
    main()
