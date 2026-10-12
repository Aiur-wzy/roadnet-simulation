#!/usr/bin/env python3
import math
length=80.0; vehicle_length=5.0; min_gap=2.5
space=vehicle_length+min_gap
cap=max(1, math.floor((max(0.0,length)+min_gap)/space)+1)
accepted=sum(1 for flow in range(cap+1) if flow < cap)
print(f'length={length} vehicleLength={vehicle_length} minGap={min_gap} vehicleSpace={space} formulaCapacity={cap} acceptedBeforeFull={accepted} thirteenthAccepted={12 < cap}')
assert cap==12 and accepted==12 and not (12 < cap)
