
def forward(speed, duration):
    print 'This should go forward at ' + speed + ' meters for second, during ' + duration + ' seconds.'

def rotate(angle, duration):
    print 'This should rotate clockwise at ' + angle + ' radians for second, during ' + duration + ' seconds.'


def file__parse(file_path):
    f = open(file_path)
    for line in f:
        line_arr = line.split()
        if line_arr[0] == 'move':
            forward(line_arr[2], line_arr[3])
        elif line_arr[0] == 'rotate':
            rotate(line_arr[1], line_arr[2])
        else:
            print "The entered command: " + str(line_arr[:])+ " is illegal."



if __name__ == "__main__":
    file__parse('file.txt')