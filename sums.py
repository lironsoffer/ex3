import os

print("----------------------------------------------------------------")
outs = os.listdir("outputs")
for out in outs:
    file = open("outputs/" + out)
    print(file)
    lines = file.readlines()
    print(lines[1])
    sums = 0
    s_sums=0
    nums = 0
    for line in lines:
        if line.startswith("from server:  reward"):
            splitted = line.split()
            for part in splitted:
                if part.isdigit():
                    sums += int(part)
                    s_sums += int(part)**2
            nums += 1
    avg = sums/nums
    var = (s_sums/(nums-1))-((sums**2)/(nums*(nums-1)))
    print(avg)
    print(var)
    
            
