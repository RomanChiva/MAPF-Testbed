import itertools

a = [1,2,3,4,5]

b = itertools.combinations(a,2)

for i,j in b:
    print(i,j)