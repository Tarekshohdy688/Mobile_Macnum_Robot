#!/usr/bin/env python3
import zmq
c = zmq.Context()
s = c.socket(zmq.PAIR)
s.connect('tcp://127.0.0.1:1234')

x = [5.3232155220,6.192138452]

y,z = x

print(f"{y},{z}".encode("utf-8"))
# while True:
#     print(s.recv().decode("utf-8"))

a = "5.5511518,6.1151848,2.15645516514"

a = a.split(",")

print(float(a[0]))
print(float(a[1]))
print(float(a[2]))
