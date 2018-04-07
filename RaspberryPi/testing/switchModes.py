import RPi.GPIO as io


MUX_1_PIN = 18
MUX_2_PIN = 16

io.setmode(io.BOARD)
io.setup(MUX_1_PIN, io.OUT)
io.setup(MUX_2_PIN, io.OUT)
io.output(MUX_1_PIN, 0)
io.output(MUX_2_PIN, 0)

print 'GPIO Setup!'

currentMode = 'FLIGHT'
#gpio ops

while True:
        print 'MODE : ',currentMode
        print "Mode Change? (y/n)?"
        ch = raw_input()
        if(ch == 'y'):
                if currentMode == 'UNDERWATER':
                        currentMode = 'FLIGHT'
                        #gpio ops
                        io.output(MUX_1_PIN, 0)
                        io.output(MUX_2_PIN, 0)
                else :
                        currentMode = 'UNDERWATER'
                        #gpio ops
                        io.output(MUX_1_PIN, 1)
                        io.output(MUX_2_PIN, 1)
