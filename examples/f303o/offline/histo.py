#!/usr/bin/env python3

# collect pulse runs and show as histogram counts
# expects input lines such as: R track d1 m0 elapsed 55

bins = [1000 * [0], 1000 * [0]]

with open('ticks.txt') as f:
    for line in f:
        v = line.split()
        try:
            if v[1] == 'track':
                sig = int(v[2][1])
                val = int(v[5])
                bins[sig][val] += 1
        except:
            print(line)

for b in [0,1]:
    t = sum(bins[b])
    for i in range(0, 1000):
        if bins[b][i] > t/1000+1:
            print(f'{i:3}',bins[b][i])
    print(20*'X', t, 's', t//59, 'm', round(t//59/60, 2), 'h')
