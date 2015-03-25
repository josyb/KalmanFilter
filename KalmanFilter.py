'''
Created on 10 Feb 2015

Copyright (c) 2015- Josy Boelen

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

'''

import os, random
import matplotlib.pyplot as plt
from myhdl import *
import hdlutils


def accu(Clk, D, Q, ena , sclr):
    ''' an add-accumulate '''
    @always_seq(Clk.posedge, reset = None)
    def adder():
        if sclr or ena:                 # combining 'loads, clears and enables'this way generates a clock enable for the registers and saves a mux
            if sclr:                    # overrides ena
                Q.next = 0
            else:
                Q.next = Q + D

    return adder


def shiftl(Clk, D, Q, ld, ena):
    ''' to shift one factor to the left '''
    WIDTH_S = len(Q)

    @always_seq(Clk.posedge, reset = None)
    def shift():
        if ld or ena:
            if ld:                                                # overrides ena
                Q.next = D                                        # will do a zero-extend
            else:
                Q.next = concat(Q[WIDTH_S - 1:], intbv(0)[1:])    # shift 1 position left, dropping the highest bit first, adding a '0' in the lowest bit


    return shift


def shiftr(Clk, D, Q, ld, ena):
    ''' to shift the other factor to th right '''
    @always_seq(Clk.posedge, reset = None)
    def shift():
        if ld or ena:
            if ld:                      # overrides ena
                Q.next = D              # will do a zero-extend
            else:
                Q.next = Q[:1]          # this truncates the lowest bit


    return shift


def KalmanFilter(WIDTH_DQ, WIDTH_COEFF, Clk, Reset, D, StrobeIn, Q, StrobeOut, CoeffA, CoeffB):
    ''' a low-resource usage Kalman Filter
        (re-) using a single adder / accumulator
    '''

    kf_state = enum( "kf_IDLE", "kf_Ynm1A", "kf_XnB", "kf_ROUND", "kf_RESULT" )
    smkfp , smkfn       = [Signal( kf_state.kf_IDLE ) for _ in range( 2 )]
    ldyn                =  Signal(bool(0))
    yn                  =  Signal(intbv(0)[WIDTH_DQ:])
    addDB               =  Signal(intbv(0)[WIDTH_DQ+WIDTH_COEFF:])
    addena, addsclr     = [Signal(bool(0)) for _ in range(2)]
    add_Q               =  Signal(intbv(0)[WIDTH_DQ+WIDTH_COEFF+2:])    # the accumulator must cater for the 2 additions too
    shiftad             =  Signal(intbv(0)[WIDTH_DQ+WIDTH_COEFF:])
    shifta_Q            =  Signal(intbv(0)[WIDTH_DQ+WIDTH_COEFF:])
    shiftld, shiftshift = [Signal(bool(0)) for _ in range(2)]
    shiftcd             =  Signal(intbv(0)[WIDTH_COEFF:])
    shiftc_Q            =  Signal(intbv(0)[WIDTH_COEFF:])

    #  instantiate the accumulator-adder and the two shift-registers
    add    = accu(Clk, addDB, add_Q, addena, addsclr)
    shifta = shiftl(Clk, shiftad, shifta_Q, shiftld, shiftshift)
    shiftc = shiftr(Clk, shiftcd, shiftc_Q, shiftld, shiftshift)

    # We build our state-machine in two processes (actually three ...)
    # Whatever they tell you, this will be the quickest and fastest, always
    # Yes, it is more typing, and yes you have to watch out for latches if you forget to specify
    # the next state somewhere in the if-then-else tree
    # But in the end, they are easier to maintain than the so-called one-process state-machine where
    # you often have to add a combinatorial process and decode the actual state and factor the inputs in (again)
    # to generate combinatorial (some call them 'asynchronous') outputs.
    @always_comb
    def smcomb():
        addena.next     = 0
        addsclr.next    = 0
        shiftld.next    = 0
        shiftshift.next = 0
        ldyn.next       = 0
        # These default assignments save a multiplexer (each)
        # We apply the arguments for the first multiply
        shiftad.next    = D         # starting off with b.x(n) saves us from (possibly) having to register the input data
        shiftcd.next    = CoeffB
        addDB.next      = shifta_Q

        # assign the output data
        Q.next = yn

        # the sequencer state machine
        if smkfp == kf_state.kf_IDLE:
            if StrobeIn:
                smkfn.next = kf_state.kf_XnB        # this is the first step
                shiftld.next = 1                    # load the shift-registers with the (default) selection for b.x(n)
                addsclr.next = 1                    # clear the adder / accumulator (only once, this time)
            else:
                smkfn.next = kf_state.kf_IDLE

        elif smkfp == kf_state.kf_XnB:
            addena.next = shiftc_Q[0]               # but only if we have a '1' for this round
            if shiftc_Q == 0:                       # we have shifted out all '1' bits
                smkfn.next   = kf_state.kf_Ynm1A    # proceed to a.y(n-1)
                shiftad.next = yn                   # we will shift the previous result left
                shiftcd.next = CoeffA               # and the coefficient A to the right
                shiftld.next = 1                    # load both shift-registers
                                                    # actually  we don't clear the 'accumulator' so we don't need to store the result
                                                    # and don't have to execute a separate adding step at the end ...
            else :
                smkfn.next = kf_state.kf_XnB        # next bit
                shiftshift.next = 1                 # shift both shift-registers one step (left or right)


        elif smkfp == kf_state.kf_Ynm1A:
            addena.next = shiftc_Q[0]               # but only if we have a '1' for this round
            if shiftc_Q == 0:                       # we have shifted out all '1' bits
                smkfn.next = kf_state.kf_ROUND      # proceed to the rouding step
            else :
                smkfn.next      = kf_state.kf_Ynm1A # next addition in this multiplication
                shiftshift.next = 1                 # shift both shift-registers one step (left or right)

        elif smkfp == kf_state.kf_ROUND:
            smkfn.next  = kf_state.kf_RESULT        # proceeed to the last step, as adding the 'rounding value'is a single addition only
            addDB.next  = 2**(WIDTH_COEFF - 1)      # to round to nearest integer, but this creates an error in regard to the theoretically calculated result
            addena.next = 1

        elif smkfp == kf_state.kf_RESULT:
            smkfn.next = kf_state.kf_IDLE           # we're done, go await next startpulse
            ldyn.next  = 1                          # tell it is the final result


    # the registered part of the state machine has a reset
    @always_seq(Clk.posedge, reset=Reset)
    def smreg():
        smkfp.next     = smkfn


    # where the others don't need to, as they will follow suit
    @always_seq(Clk.posedge, reset=None)
    def smsync():
        StrobeOut.next = 0                          # default
        if ldyn:
            StrobeOut.next = 1                      # this is a single pulse only
            yn.next = add_Q[:WIDTH_COEFF]           # but this will register the result of our computation, note that we are truncating the lower bits
                                                    # to scale the result back to the original width


    return add, shifta, shiftc, smcomb, smreg, smsync


def test_KalmanFilter():
    ''' simulate '''
    hw_inst = KalmanFilter(WIDTH_DQ, WIDTH_COEFF, Clk, Reset, D, StrobeIn, Q, StrobeOut, CoeffA, CoeffB)

    tCK = 10
    ClkCount = Signal(intbv(0)[32:])

    @instance
    def clkgen():
        yield hdlutils.genClk(Clk, tCK, ClkCount)


    @instance
    def resetgen():
        yield hdlutils.genReset(Clk, tCK, Reset)


    @instance
    def stimulusin():
        yield hdlutils.delayclks(Clk, tCK, 5)
        CoeffB.next = KalmanCoeffB
        CoeffA.next = KalmanCoeffA
        for i in range( SimRounds ):
            D.next = td[i]
            yield hdlutils.pulsesig(Clk, tCK, StrobeIn, 1, 1)
            yield hdlutils.delayclks(Clk, tCK, 2 * (WIDTH_DQ + WIDTH_COEFF) + 10)


    @instance
    def monitor():
        for _ in range(SimRounds):
            yield hdlutils.waitsig(Clk, tCK, StrobeOut, True)
            qr.append(int(Q))
        raise StopSimulation

    return instances()


def convert():
    # force std_logic_vectors instead of unsigned in Interface
    toVHDL.numeric_ports = False
    # Convert
    toVHDL(KalmanFilter, WIDTH_DQ, WIDTH_COEFF, Clk, Reset, D, StrobeIn, Q, StrobeOut, CoeffA, CoeffB)
    toVerilog(KalmanFilter, WIDTH_DQ, WIDTH_COEFF, Clk, Reset, D, StrobeIn, Q, StrobeOut, CoeffA, CoeffB)


class KalmanSimple(object):
    ''' inspired by Greg Czerniak, see: http://greg.czerniak.info/guides/kalman1/ and http://greg.czerniak.info/guides/kalman1/kalman1.py.txt '''

    def __init__(self, q, r ,p, iv):
        self.q = q      # process noise variance
        self.r = r      # measurement noise covariance
        self.p = p      # estimation error covariance
        self.x = iv     # valuexx
        self.k = 0      # don't know the kalman gain yet

    def update(self, m):
        # prediction update
        self.p += self.q
        # measurement update
        self.k = self.p / (self.p + self.r)
        self.x = self.x + self.k * (m - self.x)
        self.p = (1 - self.k) * self.p
        return self.x,self.k


if __name__ == '__main__':
    ''' we define the constants and signals globally for both conversion and simulation '''
    WIDTH_DQ                = 12
    WIDTH_COEFF             = 10
    Clk                     = Signal(bool(0))
    Reset                   = ResetSignal(0, active=1, async=True)
    D                       = Signal(intbv(0)[WIDTH_DQ:])
    Q                       = Signal(intbv(0)[WIDTH_DQ:])
    CoeffA, CoeffB          = [Signal(intbv(0)[WIDTH_COEFF:]) for _ in range(2)]
    StrobeIn , StrobeOut    = [ Signal(bool(0)) for _ in range(2) ]


    KalmanCoeff  = 0.009
    SimRounds    = 1024
    KalmanCoeffB = int(round(KalmanCoeff * 2 ** WIDTH_COEFF))
    KalmanCoeffA = int(round(2 ** WIDTH_COEFF - KalmanCoeff * 2 ** WIDTH_COEFF))
    if KalmanCoeffA + KalmanCoeffB < 2**WIDTH_COEFF:
        print 'Kalman Filter Coefficients ({:6f}, {:6f}) are less than 1! Expect a DC-offset Error' .format( 1024.0 / KalmanCoeffA, 1024.0 / KalmanCoeffB)
    elif KalmanCoeffA + KalmanCoeffB > 2**WIDTH_COEFF:
        print 'Kalman Filter Coefficients ({:6f}, {:6f}) are greater than 1! Expect an overshoot, or a run-away?' .format( 1024.0 / KalmanCoeffA, 1024.0 / KalmanCoeffB)

    # Build a list of testdata = average + noise
    td = [] # testdat to feed
    tr = [] # floating point calculation of the output
    yn = 0  # maximum error
    random.seed('C-Cam') # we want a 'repeatable randomness'
    for i in range(SimRounds):
        td.append( int(random.gauss( 0x800,  0x100 )) )
        yn = yn * (1.0 - KalmanCoeff) + td[i] * KalmanCoeff
        tr.append( yn )
    qr = []

    hdlutils.simulate(600000, test_KalmanFilter)
    convert()       # must do this before plotting etc otherwise a conflicts raises an exception

    # plot the results against the theory
    sk = KalmanSimple( 0.001, 32.0, 10.0, 0)

    average = [0x800 for _ in range(SimRounds)]
    kalmansx = []
    kalmansk = []

    for i in range(SimRounds):
        x,k = sk.update( td[i] )
        kalmansx.append( x )
        kalmansk.append( k )

    fig, ax1 = plt.subplots()
    ax1.axis( [0, SimRounds, 0, 4095])
    lines =  ax1.plot( range(SimRounds), average, 'b', label = 'Average')
    lines += ax1.plot( range(SimRounds), td,      'c', label = 'Measured')
    lines += ax1.plot( range(SimRounds), tr,      'g', label = 'Calculated (Floating Point)' )
    lines += ax1.plot( range(SimRounds), qr,      'm', label = 'RTL Result')
    lines += ax1.plot( range(SimRounds), kalmansx,'y', label = "Kalman Simple" )
    ax1.set_ylabel('DNU')

    ax2 = ax1.twinx()
    ax2.axis( [0, SimRounds, 0, 0.4095])
    lines += ax2.plot( range(SimRounds), kalmansk,'r' , label = 'Kalman Simple Gain')
    ax2.set_ylabel('kalman simple gain', color='r')
    for tl in ax2.get_yticklabels():
        tl.set_color('r')

    labs =[l.get_label() for l in lines]
    plt.legend( lines, labs, loc = 0, frameon = False)
    plt.grid()
    plt.title('Kalman Filter')
    plt.savefig( "KalmanFilter-plot.pdf", format = 'pdf') # this doesn't look as good as the one you save 'manually' from the display window
    plt.show()

    print 'All done!'

