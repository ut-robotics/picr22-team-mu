lowerBound = 0
upperBound = None

guess = 1

while True:
    print(f"Proovi {guess}")
    i = input("Kas praegune pakkumine on liiga suur?")
    if 'y' in i.lower():
        upperBound = guess
        guess = (upperBound + lowerBound) / 2
    else:
        if upperBound == None:
            guess *= 2
        else:
            lowerBound = guess
            guess = (upperBound + lowerBound) / 2