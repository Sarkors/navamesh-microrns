import RNS
import time

r = RNS.Reticulum()
time.sleep(1)

dest_hash = bytes.fromhex("858f16f7f3993ff59d36fa073555600f")

if not RNS.Transport.has_path(dest_hash):
    print("Requesting path...")
    RNS.Transport.request_path(dest_hash)
    timeout = time.time() + 15
    while not RNS.Transport.has_path(dest_hash) and time.time() < timeout:
        time.sleep(0.1)

if not RNS.Transport.has_path(dest_hash):
    print("No path found")
    exit(1)

print("Path found. Recalling identity...")
identity = RNS.Identity.recall(dest_hash, _no_use=True)

if identity is None:
    print("Identity not known — reset the RAK and try again")
    exit(1)

print("Got identity. Sending Power On...")
dest = RNS.Destination(identity, RNS.Destination.OUT, RNS.Destination.SINGLE, "navamesh", "gatekeeper")
packet = RNS.Packet(dest, "Power On".encode("utf-8"))

try:
    packet.send()
    print("Sent — watch the RAK serial monitor")
except EOFError:
    print("Sent (callback error ignored — check RAK monitor anyway)")