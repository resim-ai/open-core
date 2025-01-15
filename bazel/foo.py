import sys

print(sys.argv[1])
with open(sys.argv[1], "w") as fp:
    fp.write("JKL:")
