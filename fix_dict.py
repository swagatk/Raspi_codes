xmax = 2.0
ymax = 2.0
gap1 = 0.15
gap2 = 0.3
gap3 = 0.5

for i, y in enumerate([gap1, gap1+gap2, gap1+gap2*2, gap1+gap2*2+gap3, gap1+gap2*3+gap3, gap1+gap2*4+gap3]):
    print(f"Tag {i+6}: y={y:.2f}")

for i, x in enumerate([xmax-gap1, xmax-gap1-gap2, xmax-gap1-gap2*2, xmax-gap1-gap2*2-gap3, xmax-gap1-gap2*3-gap3, xmax-gap1-gap2*4-gap3]):
    print(f"Tag {12+i}: x={x:.2f}")

