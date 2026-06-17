import base64

payload = "JEdQR0dBLDA2NTAzNy4wMCwyMTA2LjkxODUzLFMsMDU1MTguNDg2NDksRSwxLDAzLA=="

print(base64.b64decode(payload).decode())