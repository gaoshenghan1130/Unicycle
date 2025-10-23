import can

bus = can.interface.Bus(channel='slcan', bustype='slcan')
for msg in bus:
    print(msg)