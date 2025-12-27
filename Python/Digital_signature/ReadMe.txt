File to get Signature_app.bin  --> sign_and_pack.py --> Python file

STM32 file -> VSC_Bootloader_AB_Test --> Bootlader_main() : loads file in flash
                                         check_headers() : check the signature and jump to app location





Using Open SSL create private key and public key

openssl ecparam -name prime256v1 -genkey -noout -out private.pem
openssl ec -in private.pem -pubout -out public.pem



C:\Users\RD 2\Desktop\Digital_signature>openssl dgst -sha256 app.bin
SHA2-256(app.bin)= 4efba058dbd216d30be04edc7ecf04a9d39d9c55e778c1541c01323283ed9a59

C:\Users\RD 2\Desktop\Digital_signature>openssl ec -in public.pem -pubin -text -noout
Public-Key: (256 bit)
pub:
    04:e4:d2:33:c9:11:1d:4d:1d:a6:5f:93:2f:9b:21:
    11:c1:3a:51:db:81:96:6d:38:c0:fa:cc:81:91:a5:
    9a:50:03:41:ac:7b:e9:22:21:1f:54:86:46:4f:93:
    ef:a1:35:33:11:56:14:19:b6:d9:1f:27:d8:eb:f2:
    16:84:03:72:c8
ASN1 OID: prime256v1
NIST CURVE: P-256


Generate .bin file from .elf
arm-none-eabi-objcopy -O binary app.elf app.bin


Digitally sign the .bin file as (signature_app.bin) using python script.

Below is the signature_app.bin generation procedure




on MCU:

1. Include the public key
2. create header
3. update linker script file to start location of the application to run
4. Flash the signature_app.bin file via bootloader