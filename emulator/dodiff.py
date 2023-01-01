#!/usr/bin/python3
import subprocess

print("Hello!")

def handler(signum, frame):
    print("Ctrl-c pressed")
    exit(1)

#diff <(../mini-rv32ima-master/mini-rv32ima/mini-rv32ima -f ../mini-rv32ima-master/buildroot/output/images/Image -m 0x6000000 -l) <(./riscv-emulator ../mini-rv32ima-master/buildroot/output/images/Image)

proc1 = subprocess.Popen(['../mini-rv32ima-master/mini-rv32ima/mini-rv32ima',
    '-f','../mini-rv32ima-master/buildroot/output/images/Image','-m','0x4000000','-l','-s'],
    stdout=subprocess.PIPE)

proc2 = subprocess.Popen(['./riscv-emulator',
    '../mini-rv32ima-master/buildroot/output/images/Image'],
    stdout=subprocess.PIPE)

common = ""

while True:
    line1 = proc1.stdout.readline()
    if not line1:
        break

    line2 = proc2.stdout.readline()
    if not line2:
        break

    if line1 == line2:
        common = line1
    else:
        print("= ", common)
        print("< ", line1)
        print("> ", line2)
