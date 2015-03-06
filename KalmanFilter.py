'''
Created on 10 Feb 2015

@author: Josy

# Copyright (c) 2015- Josy Boelen
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
'''

import os, random 
import matplotlib.pyplot as plt
from myhdl import *
import hdlutils


def accu(Clk, D, Q, ena , sclr):  
    lq = Signal(intbv(0)[len(Q):])
    
    @always_seq(Clk.posedge, reset = None)
    def adder():
        if sclr or ena:
            if sclr:
                # overrides ena
                lq.next = 0
            else:
                lq.next  = lq + D
                
    @always_comb
    def assignouts():
        Q.next = lq
        
    return adder, assignouts


def shiftl(Clk, D, Q, ld, ena):
    WIDTH_S = len(Q)
    lq = Signal(intbv(0)[WIDTH_S:])
    
    @always_seq(Clk.posedge, reset = None)
    def shift():
        if ld or ena:
            if ld:
                # overrides shift
                lq.next = D # will do a zero-extend
            else:
                lq.next[:1] = lq[WIDTH_S - 1:]
                lq.next[0] = 0

    @always_comb
    def assignouts():
        Q.next = lq
            
    return shift, assignouts


def shiftr(Clk, D, Q, ld, ena):
    lq = Signal(intbv(0)[len(Q):])
    
    @always_seq(Clk.posedge, reset = None)
    def shift():
        if ld or ena:
            if ld:
                lq.next = D # will do a zero-extend
            else:
                lq.next = lq[:1]

    @always_comb
    def assignouts():
        Q.next = lq
                
    return shift, assignouts


def KalmanFilter(WIDTH_DQ, WIDTH_COEFF, Clk, Reset, D, StrobeIn, Q, StrobeOut, CoeffA, CoeffB):
    ''' a low-resource usage Kalman Filter  
        (re-) using a single adder / accumulator
    '''
    
    kf_state = enum( "kf_IDLE", "kf_Ynm1A", "kf_XnB", "kf_ROUND", "kf_RESULT" )
    smkfp , smkfn = [Signal( kf_state.kf_IDLE ) for _ in range( 2 )]
    ldyn = Signal(bool(0))
    yn = Signal(intbv(0)[WIDTH_DQ:])
    addDB = Signal(intbv(0)[WIDTH_DQ+WIDTH_COEFF:])
    addena, addsclr = [Signal(bool(0)) for _ in range(2)]
    add_Q = Signal(intbv(0)[WIDTH_DQ+WIDTH_COEFF+1:])
    shiftad = Signal(intbv(0)[WIDTH_DQ+WIDTH_COEFF:])
    shifta_Q = Signal(intbv(0)[WIDTH_DQ+WIDTH_COEFF:])
    shiftld, shiftshift = [Signal(bool(0)) for _ in range(2)] 
    shiftcd = Signal(intbv(0)[WIDTH_DQ:])
    shiftc_Q = Signal(intbv(0)[WIDTH_DQ+WIDTH_COEFF:])

    add = accu(Clk, addDB, add_Q, addena, addsclr)
    shifta = shiftl(Clk, shiftad, shifta_Q, shiftld, shiftshift)
    shiftc = shiftr(Clk, shiftcd, shiftc_Q, shiftld, shiftshift)
    
    @always_comb
    def smcomb():
        addena.next = 0
        addsclr.next = 0
        shiftld.next = 0
        shiftshift.next = 0
        ldyn.next = 0
        # these default assignments save a multiplexer (each)
        shiftad.next = D
        shiftcd.next = CoeffB
        addDB.next = shifta_Q
        
        # assign the output data
        Q.next = yn

        # the sequencer state machine
        if smkfp == kf_state.kf_IDLE:
            if StrobeIn:
                smkfn.next = kf_state.kf_XnB    # this is the first step
                shiftld.next = 1                # load the shift-registers with the default selection
                addsclr.next = 1                # clear the adder / accumulator
            else:
                smkfn.next = kf_state.kf_IDLE
         
        elif smkfp == kf_state.kf_XnB:
            addena.next = shiftc_Q[0]           # but only if we have a '1' for this round
            if shiftc_Q == 0:                   # we have shifted out all '1' bits
                smkfn.next = kf_state.kf_Ynm1A
                shiftad.next = yn               # we will shift the previous result left
                shiftcd.next = CoeffA           # and the coefficients to the right
                shiftld.next = 1                # load both shift-registers
                                                # actually  we don't clear the 'accumulator' so we don't need to store the result
                                                # and don't have to execute a separate adding step at the end ...
            else :
                smkfn.next = kf_state.kf_XnB    
                shiftshift.next = 1             # shift both shift-registers one step (left or right)

             
        elif smkfp == kf_state.kf_Ynm1A:
            addena.next = shiftc_Q[0]           # but only if we have a '1' for this round
            if shiftc_Q == 0:
                smkfn.next = kf_state.kf_ROUND
            else :
                smkfn.next = kf_state.kf_Ynm1A    
                shiftshift.next = 1

        elif smkfp == kf_state.kf_ROUND:
            smkfn.next = kf_state.kf_RESULT
            addDB.next = 2**(WIDTH_COEFF - 1)   # to round to nearest integer, but this creates an error in regard to the theoretically calculated result
            addena.next = 1
                
        elif smkfp == kf_state.kf_RESULT:
            smkfn.next = kf_state.kf_IDLE
            ldyn.next = 1                   # tell it is the final result
  
        
    @always_seq(Clk.posedge, reset=Reset)
    def smprocess():
        smkfp.next = smkfn
        StrobeOut.next = 0
        if ldyn:
            StrobeOut.next = 1
  
            
    @always_seq(Clk.posedge, reset=None)
    def registers():
        if ldyn:
            yn.next = add_Q[:WIDTH_COEFF]
 
            
    return add, shifta, shiftc, smcomb, smprocess, registers 
     

def test_KalmanFilter():
    hw_inst = KalmanFilter(WIDTH_DQ, WIDTH_COEFF, Clk, Reset, D, StrobeIn, Q, StrobeOut, CoeffA, CoeffB)

    tCK = 10
    ClkCount = Signal(intbv(0)[32:])
    random.seed('C-Cam')
    
 
            
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
    def __init__(self, q, r ,p, iv):
        self.q = q      # process noise variance
        self.r = r      # measurement noise covariance
        self.p = p      # estimation error covariance
        self.x = iv     # value
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
    WIDTH_DQ = 12
    WIDTH_COEFF = 10
    KalmanCoeff = 0.009
    SimRounds = 1024
    KalmanCoeffB = int(round(KalmanCoeff * 2 ** WIDTH_COEFF))
    KalmanCoeffA = int(round(2 ** WIDTH_COEFF - KalmanCoeff * 2 ** WIDTH_COEFF))
    if KalmanCoeffA + KalmanCoeffB != 2**WIDTH_COEFF:
        print 'Kalman Filter Coefficients ({:6f}, {:6f}) do not add up to 1! Expect a DC-offset Error' .format( 1024.0 / KalmanCoeffA, 1024.0 / KalmanCoeffB)
        
    td = []
    tr = []
    yn = 0
    for i in range(SimRounds):
        td.append( int(random.gauss( 0x800,  0x100 )) )
        yn = yn * (1.0 - KalmanCoeff) + td[i] * KalmanCoeff
        tr.append( yn )
    qr = []
    Clk = Signal(bool(0))
    Reset = ResetSignal(0, active=1, async=True)
    D = Signal(intbv(0)[WIDTH_DQ:])
    Q = Signal(intbv(0)[WIDTH_DQ:])
    CoeffA, CoeffB = [Signal(intbv(0)[WIDTH_COEFF:]) for _ in range(2)]
    StrobeIn , StrobeOut  = [ Signal(bool(0)) for _ in range(2) ]
    
#     os.chdir( "./out")
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
    lines = ax1.plot( range(SimRounds), average, 'b', label = 'Average')
    lines += ax1.plot( range(SimRounds), td, 'c', label = 'Measured')
    lines += ax1.plot(range(SimRounds), tr, 'g', label = 'Calculated' )
    lines += ax1.plot(range(SimRounds), qr, 'm', label = 'RTL Result')
    lines += ax1.plot(range(SimRounds), kalmansx,'y' , label = "Kalman Simple" )
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

