#!/usr/bin/env python
# Developed by: Michael Buettner (buettner@cs.washington.edu)
#Modified for USRP2 by: Yuanqing Zheng (yuanqing1@ntu.edu.sg)

from gnuradio import digital
from gnuradio import eng_notation
from gnuradio import gr, gru
from gnuradio import uhd
from gnuradio.wxgui import scopesink2
from gnuradio.wxgui import scopesink2
from gnuradio.eng_option import eng_option
from gnuradio.gr import firdes
from grc_gnuradio import wxgui as grc_wxgui
from optparse import OptionParser
from string import split
from string import strip
from string import atoi
import time
import os
import math
import rfid
import wx
from threading import Timer


log_file = open("log_out.log", "a")


class top_block(grc_wxgui.top_block_gui):
    '''
    Constructor (__init__)
    Initialises the GUI, sets PHY layer parameters, sets up blocks and connects them into a flow graph
    '''

    def __init__(self):

        # GUI setup
        grc_wxgui.top_block_gui.__init__(self, title="Grc Wisp Reader")

        self.wxgui_scopesink2_0 = scopesink2.scope_sink_f(
            self.GetWin(),
            title="Scope Plot",
            sample_rate=1e6,
            v_scale=0,
            v_offset=0,
            t_scale=0,
            ac_couple=False,
            xy_mode=False,
            num_inputs=1,
            trig_mode=gr.gr_TRIG_MODE_AUTO,
            y_axis_label="Counts",
        )
        self.Add(self.wxgui_scopesink2_0.win)

        # Constants
        amplitude = 1
        interp_rate = 128
        dec_rate = 16 #		dec_rate = 8
        sw_dec = 2 #	sw_dec = 4

        num_taps = int(64000 / ( (dec_rate * 4) * 256 ))  #Filter matched to 1/4 of the 256 kHz tag cycle  #		num_taps = int(64000 / ( (dec_rate * 4) * 40 )) #Filter matched to 1/4 of the 40 kHz tag cycle

        taps = [complex(1, 1)] * num_taps

        matched_filt = gr.fir_filter_ccc(sw_dec, taps);

        to_mag = gr.complex_to_mag()

        center = rfid.center_ff(4)  #		center = rfid.center_ff(10)

        mm = rfid.clock_recovery_zc_ff(4, 1);
        self.reader = rfid.reader_f(int(128e6 / interp_rate));

        tag_decoder = rfid.tag_decoder_f()

        command_gate = rfid.command_gate_cc(12, 60, 64000000 / dec_rate / sw_dec)         #		command_gate = rfid.command_gate_cc(12, 250, 64000000 / dec_rate / sw_dec)

        to_complex = gr.float_to_complex()
        amp = gr.multiply_const_ff(amplitude)

        ##################################################
        # Blocks
        ##################################################
        freq = 915e6
        rx_gain = 1

        # Transmitter setup
        tx = uhd.usrp_sink(
            device_addr="",
            io_type=uhd.io_type.COMPLEX_FLOAT32,
            num_channels=1,
        )
        tx.set_samp_rate(128e6 / interp_rate)
        tx.set_center_freq(freq, 0)

        # Receiver setup
        rx = uhd.usrp_source(
            device_addr="",
            io_type=uhd.io_type.COMPLEX_FLOAT32,
            num_channels=1,
        )
        rx.set_samp_rate(64e6 / dec_rate)
        rx.set_center_freq(freq, 0)
        rx.set_gain(rx_gain, 0)

        # Command gate
        command_gate.set_ctrl_out(self.reader.ctrl_q())

        # Tag decoder
        tag_decoder.set_ctrl_out(self.reader.ctrl_q())

        #########Build Graph
        self.connect(rx, matched_filt)
        self.connect(matched_filt, command_gate)

        self.connect(command_gate, to_mag)
        #		self.connect(command_gate, agc)         #		agc = gr.agc2_cc(0.3, 1e-3, 1, 1, 100)
        #		self.connect(agc, to_mag)

        self.connect(to_mag, center, mm, tag_decoder)
        #		self.connect(to_mag, center, matched_filt_tag_decode, tag_decoder)
        self.connect(tag_decoder, self.reader)
        self.connect(self.reader, amp)
        self.connect(amp, to_complex)
        self.connect(to_complex, tx)

    #################


def main():
    #    gr.enable_realtime_scheduling()
    tb = top_block()

    tb.Run(True)
    while 1:
        c = raw_input("'Q' to quit. L to get log.\n")
        if c == "q":
            break

        if c == "L" or c == "l":
            log_file.write("T,CMD,ERROR,BITS,SNR\n")
            log = tb.reader.get_log() # get_log defined in rfid_reader_f.h, return type is gr_msg_queue_sptr from gr

            # The loop below is implemented like this for performance reasons, even though it would look nicer with a while true and a break
            msg = log.delete_head_nowait()
            while msg !=0:
                print_log_msg(msg, log_file)
                msg = log.delete_head_nowait()

    tb.Stop(True)


def print_log_msg(msg, log_file):
    LOG_START_CYCLE, LOG_QUERY, LOG_ACK, LOG_QREP, LOG_NAK, LOG_REQ_RN, LOG_READ, LOG_RN16, LOG_EPC, LOG_HANDLE, LOG_DATA, LOG_EMPTY, LOG_COLLISION, LOG_OKAY, LOG_ERROR = range(
        15)

    fRed = chr(27) + '[31m'
    fBlue = chr(27) + '[34m'
    fReset = chr(27) + '[0m'

    fields = split(strip(msg.to_string()), " ")

    if msg.type() == LOG_START_CYCLE:
        print "%s\t Started Cycle" % (fields[-1])
        log_file.write(fields[-1] + ",START_CYCLE,0,0,0\n");
    elif msg.type() == LOG_QUERY:
        print "%s\t Query" % (fields[-1])
        log_file.write(fields[-1] + ",QUERY,0,0,0\n");
    elif msg.type() == LOG_QREP:
        print "%s\t QRep" % (fields[-1])
        log_file.write(fields[-1] + ",QREP,0,0,0\n");
    elif msg.type() == LOG_ACK:
        print "%s\t ACK" % (fields[-1])
        log_file.write(fields[-1] + ",ACK,0,0,0\n");
    elif msg.type() == LOG_NAK:
        print "%s\t NAK" % (fields[-1])
        log_file.write(fields[-1] + ",NAK,0,0,0\n");
    elif msg.type() == LOG_EMPTY:
        snr = strip(fields[0])
        print "%s\t    - Empty Slot - " % (fields[-1])
        log_file.write(fields[-1] + ",EMPTY,0,0," + snr + "\n");
    elif msg.type() == LOG_COLLISION:
        print "%s\t    - Collision - " % (fields[-1])
        log_file.write(fields[-1] + ",COLLISION,0,0,0\n");
    elif msg.type() == LOG_RN16:
        rn16 = fields[0].split(",")[0]
        snr = strip(fields[0].split(",")[1])
        tmp = int(rn16, 2)
        if msg.arg2() == LOG_ERROR:
            print "%s\t    %s RN16 w/ Error: %04X%s" % (fields[-1], fRed, tmp, fReset)
            log_file.write(fields[-1] + ",RN16,1," + "%04X" % tmp + "," + snr + "\n");
        else:
            print "%s\t    %s RN16: %04X%s" % (fields[-1], fBlue, tmp, fReset)
            log_file.write(fields[-1] + ",RN16,0," + "%04X" % tmp + "," + snr + "\n");
    elif msg.type() == LOG_EPC:
        epc = fields[0].split(",")[0]
        snr = strip(fields[0].split(",")[1])
        epc = epc[16:112]
        tmp = int(epc, 2)
        if msg.arg2() == LOG_ERROR:
            print "%s\t    %s EPC w/ Error: %024X%s" % (fields[-1], fRed, tmp, fReset)
            log_file.write(fields[-1] + ",EPC,1," + "%024X" % tmp + "," + snr + "\n");
        else:
            print "%s\t    %s EPC: %024X%s" % (fields[-1], fBlue, tmp, fReset)
            log_file.write(fields[-1] + ",EPC,0," + "%024X" % tmp + "," + snr + "\n");

if __name__ == '__main__':
    main()


