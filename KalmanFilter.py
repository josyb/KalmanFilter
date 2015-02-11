'''
Created on 10 Feb 2015

@author: Josy
'''

import os
from myhdl import *
import hdlutils


def alu(Clk, DA, DB, Q, ena , sclr):  
    @always_seq(Clk.posedge, reset = None)
    def adder():
        if sclr or ena:
            if sclr:
                # overrides ena
                Q.next = 0
            else:
                Q.next  = DA + DB
    
    return adder

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
        (re-) using a single addder
    '''
    
    # we have two state machines
    # one sequencing the Kalman filter operation
    # the second sequencing the multiplication
    kf_state = enum( "kf_IDLE", "kf_Ynm1A", "kf_XnB", "kf_Yn", "kf_ROUND", "kf_RESULT" )
    smkfp , smkfn = [Signal( kf_state.kf_IDLE ) for _ in range( 2 )]
    mpy_state = enum( "mpy_IDLE", "mpy_MPY" )
    smmpyp , smmpyn = [Signal( mpy_state.mpy_IDLE ) for _ in range( 2 )]
    startmpy = Signal(bool(0))
    mpydone = Signal(bool(0))
    xnb = Signal(intbv(0)[WIDTH_DQ + WIDTH_COEFF:])
    ldyn = Signal(bool(0))
    ldxnb = Signal(bool(0))
    yn = Signal(intbv(0)[WIDTH_DQ + WIDTH_COEFF:])
    addDA, addDB = [Signal(intbv(0)[WIDTH_DQ+WIDTH_COEFF*2:]) for _ in range(2)]
    addena, addsclr = [Signal(bool(0)) for _ in range(2)]
    add_Q = Signal(intbv(0)[WIDTH_DQ+WIDTH_COEFF*2+1:])
    shiftad = Signal(intbv(0)[WIDTH_DQ+WIDTH_COEFF*2:])
    shifta_Q = Signal(intbv(0)[WIDTH_DQ+WIDTH_COEFF*2:])
    shiftld, shiftshift = [Signal(bool(0)) for _ in range(2)] 
    shiftcd = Signal(intbv(0)[WIDTH_DQ:])
    shiftc_Q = Signal(intbv(0)[WIDTH_DQ+WIDTH_COEFF:])
    mpycountsld, mpycountcnten = [Signal(bool(0)) for _ in range(2)]
    mpycountdata, mpycount = [Signal(intbv(0)[hdlutils.widthu(WIDTH_COEFF*2):]) for _ in range(2)]

    add = alu(Clk, addDA, addDB, add_Q, addena, addsclr)
    shifta = shiftl(Clk, shiftad, shifta_Q, shiftld, shiftshift)
    shiftc = shiftr(Clk, shiftcd, shiftc_Q, shiftld, shiftshift)
    
    @always_comb
    def smcomb():
        addena.next = 0
        addsclr.next = 0
        startmpy.next = 0
        mpydone.next = 0
        shiftld.next = 0
        shiftshift.next = 0
        shiftad.next = 0
        shiftcd.next = 0
        ldxnb.next = 0
        mpycountsld.next = 0
        mpycountcnten.next = 0
        ldyn.next = 0
        mpycountdata.next = 0
        
        # assign the output data
        Q.next = yn[:WIDTH_COEFF]

        # the sequencer state machine
        if smkfp == kf_state.kf_IDLE:
            if StrobeIn:
                smkfn.next = kf_state.kf_XnB
                mpycountdata.next = WIDTH_COEFF
                mpycountsld.next = 1
                shiftad.next = D
                shiftcd.next = CoeffB
                shiftld.next = 1
                addsclr.next = 1
                startmpy.next = 1
            else:
                smkfn.next = kf_state.kf_IDLE
         
        elif smkfp == kf_state.kf_XnB:
            addDA.next = add_Q
            addDB.next = shifta_Q
            if mpydone:
                smkfn.next = kf_state.kf_Ynm1A
                ldxnb.next = 1    
                mpycountdata.next = WIDTH_COEFF
                mpycountsld.next = 1
                shiftad.next = yn           # we will shift the data left
                shiftcd.next = CoeffA       # and the coefficients to the right
                shiftld.next = 1            # load bot shiftregisters
                addsclr.next = 1            # and clear the adder, because it's output will be fed back
                startmpy.next = 1        
            else :
                smkfn.next = kf_state.kf_XnB    

             
        elif smkfp == kf_state.kf_Ynm1A:
            addDA.next = add_Q              # feed the adder's result back
            addDB.next = shifta_Q           # the shifted-left data to add
            if mpydone:
                smkfn.next = kf_state.kf_Yn
            else :
                smkfn.next = kf_state.kf_Ynm1A    
                              
        elif smkfp == kf_state.kf_Yn:
            smkfn.next = kf_state.kf_ROUND
            addDA.next = add_Q[:WIDTH_COEFF]              # use the last result
            addDB.next = xnb                # and add the stored previous result
            addena.next = 1
            
        elif smkfp == kf_state.kf_ROUND:
            smkfn.next = kf_state.kf_RESULT
            addDA.next = add_Q                  # use the last result
            addDB.next = 0 #2**(WIDTH_COEFF - 1)   # to round to nearest integer
            addena.next = 1
                
        elif smkfp == kf_state.kf_RESULT:
            smkfn.next = kf_state.kf_IDLE
            ldyn.next = 1                   # tell it is the final result
        
        # the multiply state machine
        if smmpyp == mpy_state.mpy_IDLE:
            if startmpy:
                smmpyn.next = mpy_state.mpy_MPY                
            else:
                smmpyn.next = mpy_state.mpy_IDLE
                
        elif smmpyp == mpy_state.mpy_MPY :
            if mpycount == 0:
                mpydone.next = 1
                if startmpy:
                    smmpyn.next = mpy_state.mpy_MPY                
                else:
                    smmpyn.next = mpy_state.mpy_IDLE    
            else:
                smmpyn.next = mpy_state.mpy_MPY
                addena.next = shiftc_Q[0]       # but only if we have a '1' for this round
                mpycountcnten.next = 1
                shiftshift.next = 1
            
        
    @always_seq(Clk.posedge, reset=Reset)
    def smprocess():
        smkfp.next = smkfn
        smmpyp.next = smmpyn
        StrobeOut.next = 0
        if ldyn:
            StrobeOut.next = 1
            
    @always_seq(Clk.posedge, reset=None)
    def registers():
        if mpycountsld or mpycountcnten:
            if mpycountsld:                  
                mpycount.next = mpycountdata      #
            else:
                mpycount.next = mpycount -1

        if ldxnb:
            xnb.next = add_Q
            
        if ldyn:
            yn.next = add_Q
            
    return add, shifta, shiftc, smcomb, smprocess, registers 
     

def test_KalmanFilter():
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
        yield hdlutils.MMdelay(Clk, tCK, 5)
        CoeffB.next = KalmanCoeffB
        CoeffA.next = KalmanCoeffA
        D.next =  0x800
        for _ in range( SimRounds ):
            yield hdlutils.pulsesig(Clk, tCK, StrobeIn, 1, 1)
            yield hdlutils.MMdelay(Clk, tCK, 2 * (WIDTH_DQ + WIDTH_COEFF) + 10)
        raise StopSimulation
    
    return instances()


def convert():
    # force std_logic_vectors instead of unsigned in Interface
    toVHDL.numeric_ports = False
    # Convert
    toVHDL(KalmanFilter, WIDTH_DQ, WIDTH_COEFF, Clk, Reset, D, StrobeIn, Q, StrobeOut, CoeffA, CoeffB)
    toVerilog(KalmanFilter, WIDTH_DQ, WIDTH_COEFF, Clk, Reset, D, StrobeIn, Q, StrobeOut, CoeffA, CoeffB)


if __name__ == '__main__':
    WIDTH_DQ = 12
    WIDTH_COEFF = 10
    KalmanCoeff = 0.19
    SimRounds = 256
    KalmanCoeffB = int(round(KalmanCoeff * 2 ** WIDTH_COEFF))
    KalmanCoeffA = int(round(2 ** WIDTH_COEFF - 1 - KalmanCoeff * 2 ** WIDTH_COEFF))
    print KalmanCoeffA, KalmanCoeffB
    Clk = Signal(bool(0))
    Reset = ResetSignal(0, active=1, async=True)
    D = Signal(intbv(0)[WIDTH_DQ:])
    Q = Signal(intbv(0)[WIDTH_DQ:])
    CoeffA, CoeffB = [Signal(intbv(0)[WIDTH_COEFF:]) for _ in range(2)]
    StrobeIn , StrobeOut  = [ Signal(bool(0)) for _ in range(2) ]
    
    os.chdir( "./out")
    hdlutils.simulate(600000, test_KalmanFilter)
    convert()

