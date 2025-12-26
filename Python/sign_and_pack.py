import struct
import hashlib
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import ec
from cryptography.hazmat.backends import default_backend

# ---------------- CONFIG ----------------
APP_BIN = "Digital_Signature/app.bin"
PRIVATE_KEY_PEM = "Digital_Signature/private.pem"
OUT_BIN = "Digital_Signature/signed_app.bin"

MAGIC = 0xDEADBEEF
# ---------------------------------------

# 1️⃣ Read app.bin
with open(APP_BIN, "rb") as f:
    app_data = f.read()

app_size = len(app_data)
print(f"[+] App size: {app_size} bytes")

# 2️⃣ Compute SHA-256 of app.bin
hash_bytes = hashlib.sha256(app_data).digest()
print("[+] SHA-256 calculated")

# 3️⃣ Load private key
with open(PRIVATE_KEY_PEM, "rb") as f:
    private_key = serialization.load_pem_private_key(
        f.read(),
        password=None,
        backend=default_backend()
    )

# 4️⃣ Sign hash (ECDSA P-256)
signature_der = private_key.sign(
    hash_bytes,
    ec.ECDSA(hashes.SHA256())
)

# Convert DER signature → raw (r || s)
from cryptography.hazmat.primitives.asymmetric.utils import decode_dss_signature
r, s = decode_dss_signature(signature_der)
signature_raw = r.to_bytes(32, "big") + s.to_bytes(32, "big")

print("[+] Signature created")

# 5️⃣ Build header (little-endian)
header = (
    struct.pack("<I", MAGIC) +
    struct.pack("<I", app_size) +
    hash_bytes +
    signature_raw
)

print(f"[+] Header size: {len(header)} bytes")

# 6️⃣ Write final signed binary
with open(OUT_BIN, "wb") as f:
    f.write(header)
    f.write(app_data)

print(f"[✓] Signed firmware created: {OUT_BIN}")
