"""Fetch a GigE Vision camera's GenICam XML over GVCP.

Reads the bootstrap registers for the vendor and model names and the First
URL register (GigE Vision 2.x, section 28) for where the XML lives, then
reads it out with READMEM. Only `Local:` URLs are handled; every Basler GigE
camera stores its XML that way.

usage: genicam_xml.py <camera-ip> <out-dir>
prints the path of the XML it wrote, named <Vendor>-<Model>.xml
"""

import io
import os
import socket
import struct
import sys
import zipfile

GVCP_PORT = 3956
READMEM_CMD = 0x0084
MANUFACTURER_NAME = 0x0048
MODEL_NAME = 0x0068
FIRST_URL = 0x0200
MAX_READ = 512


class Gvcp:
    def __init__(self, ip):
        self.ip = ip
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(1.0)
        self.req_id = 0

    def readmem(self, addr, count):
        padded = (count + 3) & ~3
        for _ in range(5):
            self.req_id = self.req_id % 0xFFFF + 1
            cmd = struct.pack(">BBHHHIHH", 0x42, 0x01, READMEM_CMD, 8,
                              self.req_id, addr, 0, padded)
            self.sock.sendto(cmd, (self.ip, GVCP_PORT))
            try:
                ack, _ = self.sock.recvfrom(2048)
            except socket.timeout:
                continue
            status, _, _, ack_id = struct.unpack(">HHHH", ack[:8])
            if ack_id != self.req_id:
                continue
            if status != 0:
                raise RuntimeError(f"READMEM 0x{addr:x}: status 0x{status:04x}")
            return ack[12:12 + count]
        raise RuntimeError(f"READMEM 0x{addr:x}: no answer from {self.ip}")

    def string(self, addr, size):
        return self.readmem(addr, size).split(b"\0", 1)[0].decode().strip()

    def block(self, addr, size):
        out = bytearray()
        while len(out) < size:
            n = min(MAX_READ, size - len(out))
            out += self.readmem(addr + len(out), n)
        return bytes(out)


def main():
    ip, out_dir = sys.argv[1:3]
    gvcp = Gvcp(ip)
    vendor = gvcp.string(MANUFACTURER_NAME, 32)
    model = gvcp.string(MODEL_NAME, 32)
    url = gvcp.string(FIRST_URL, 512)
    scheme, rest = url.split(":", 1)
    if scheme.lower() != "local":
        sys.exit(f"{url}: only Local: URLs are supported")
    name, addr, size = rest.split(";")[:3]
    blob = gvcp.block(int(addr, 16), int(size, 16))
    if name.lower().endswith(".zip"):
        with zipfile.ZipFile(io.BytesIO(blob)) as z:
            member = next(n for n in z.namelist() if n.lower().endswith(".xml"))
            blob = z.read(member)
    path = os.path.join(out_dir, f"{vendor}-{model}.xml".replace(" ", "_"))
    with open(path, "wb") as f:
        f.write(blob)
    print(path)


if __name__ == "__main__":
    main()
