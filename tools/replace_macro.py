import sys

if __name__ == "__main__":
    if len(sys.argv) != 3:
        sys.exit("usage: python3 {} <folder> <filename>".format(sys.argv[0]))

    with open("memory.v") as f:
        s = f.readlines()
        for i in range(len(s)):
            if s[i].find('define INITIAL_DATA_FILE') >= 0:
                s[i] = '`define INITIAL_DATA_FILE "{}"\n'.format(sys.argv[2])
            elif s[i].find('define INITIAL_DATA_DIR') >= 0:
                s[i] = '`define INITIAL_DATA_DIR "{}"\n'.format(sys.argv[1])

    with open("memory.v", "w") as f:
        for l in s:
            f.write(l)
