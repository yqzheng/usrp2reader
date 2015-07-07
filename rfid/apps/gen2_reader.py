#!/usr/bin/env python
# Developed by: Michael Buettner (buettner@cs.washington.edu)

from gnuradio import gr, gru
from gnuradio import usrp
from gnuradio import eng_notation
from gnuradio.eng_option import eng_option
from string import split
from string import strip
from string import atoi
import time
import os
import math
import rfid

log_file = open("log_out.log", "a")

class my_top_block(gr.top_block):
    def __init__(self):
        gr.top_block.__init__(self)

        filt_out = gr.file_sink(gr.sizeof_gr_complex, "./filt.out")
        ffilt_out = gr.file_sink(gr.sizeof_float, "./ffilt.out")
        f_sink = gr.file_sink(gr.sizeof_gr_complex, 'f_sink.out')
        f_sink2 = gr.file_sink(gr.sizeof_gr_complex, 'f_sink2.out')

        AMPLITUDE = 5000
        OMEGA = 5
        MU = 0.25
        GAIN_MU = 0.25
        GAIN_OMEGA = .25 * GAIN_MU * GAIN_MU
        OMEGA_RELATIVE_LIMIT = .05
        INTERP_RATE = 256
        DEC_RATE = 16
        SW_DEC = 5
        FREQ = 915e6

        self.reader = rfid.reader_f(int(128e6 / INTERP_RATE))

        rx = get_rx(DEC_RATE, FREQ)

        matched_filt = get_finite_impulse_response_filter(SW_DEC, DEC_RATE)

        command_gate = get_command_gate(SW_DEC, DEC_RATE, self.reader)
        agc = gr.agc2_cc(0.3, 1e-3, 1, 1, 100)
        to_mag = gr.complex_to_mag()
        center = rfid.center_ff(10)
        mm = gr.clock_recovery_mm_ff(OMEGA, GAIN_OMEGA, MU, GAIN_MU, OMEGA_RELATIVE_LIMIT)
        tag_decoder = get_tag_decoder(self.reader)
        amp = gr.multiply_const_ff(AMPLITUDE)
        to_complex = gr.float_to_complex()
        tx = get_tx(DEC_RATE, FREQ)

        # Build Graph
        self.connect(rx, matched_filt)
        self.connect(matched_filt, command_gate)
        self.connect(command_gate, agc)
        self.connect(agc, to_mag)
        self.connect(to_mag, center, mm, tag_decoder)
        self.connect(tag_decoder, self.reader, amp, to_complex, tx)
        # End Build Graph

        self.connect(matched_filt, filt_out)

def get_tx(dec_rate, freq):
    freq = 915e6
    tx = usrp.sink_c(fusb_block_size=512, fusb_nblocks=4)
    tx.set_interp_rate(256)
    tx_subdev = (0, 0)
    tx.set_mux(usrp.determine_tx_mux_value(tx, tx_subdev))
    subdev = usrp.selected_subdev(tx, tx_subdev)
    subdev.set_enable(True)
    subdev.set_gain(subdev.gain_range()[2])
    t = tx.tune(subdev.which(), subdev, freq)
    if not t:
        print "Couldn't set tx freq"
    return tx


def get_rx(dec_rate, freq):
    rx_gain = 20
    rx = usrp.source_c(0, dec_rate, fusb_block_size=512, fusb_nblocks=4)
    rx_subdev_spec = (1, 0)
    rx.set_mux(usrp.determine_rx_mux_value(rx, rx_subdev_spec))
    rx_subdev = usrp.selected_subdev(rx, rx_subdev_spec)
    rx_subdev.set_gain(rx_gain)
    rx_subdev.set_auto_tr(False)
    rx_subdev.set_enable(True)

    r = usrp.tune(rx, 0, rx_subdev, freq)
    if not r:
        print "Couldn't set rx freq"
    return rx

def get_finite_impulse_response_filter(sw_dec, dec_rate):
    # Filter matched to 1/4 of the 40 kHz tag cycle
    num_taps = int(64000 / ((dec_rate * 4) * 40))
    taps = [complex(1, 1)] * num_taps
    return gr.fir_filter_ccc(sw_dec, taps)

def get_command_gate(sw_dec, dec_rate, reader):
    command_gate = rfid.command_gate_cc(12, 250, 64000000 / dec_rate / sw_dec)
    command_gate.set_ctrl_out(reader.ctrl_q())
    return command_gate

def get_tag_decoder(reader):
    tag_decoder = rfid.tag_decoder_f()
    tag_decoder.set_ctrl_out(reader.ctrl_q())
    return tag_decoder

def main():
    tb = my_top_block()

    tb.start()
    while 1:

        c = raw_input("'Q' to quit. L to get log.\n")
        if c == "q":
            break

        if c == "L" or c == "l":
            log_file.write("T,CMD,ERROR,BITS,SNR\n")
            log = tb.reader.get_log()
            print "Log has %s Entries" % (str(log.count()))
            i = log.count()

            for k in range(0, i):
                msg = log.delete_head_nowait()
                print_log_msg(msg, log_file)

    tb.stop()


def print_log_msg(msg, log_file):
    LOG_START_CYCLE, LOG_QUERY, LOG_ACK, LOG_QREP, LOG_NAK, LOG_REQ_RN, LOG_READ, LOG_RN16, LOG_EPC, LOG_HANDLE, LOG_DATA, LOG_EMPTY, LOG_COLLISION, LOG_OKAY, LOG_ERROR = range(
        15)

    fRed = chr(27) + '[31m'
    fBlue = chr(27) + '[34m'
    fReset = chr(27) + '[0m'

    if msg.type() == LOG_START_CYCLE:
        fields = split(strip(msg.to_string()), " ")
        print "%s\t Started Cycle" % (fields[-1])
        log_file.write(fields[-1] + ",START_CYCLE,0,0,0\n")

    if msg.type() == LOG_QUERY:
        fields = split(strip(msg.to_string()), " ")
        print "%s\t Query" % (fields[-1])
        log_file.write(fields[-1] + ",QUERY,0,0,0\n")

    if msg.type() == LOG_QREP:
        fields = split(strip(msg.to_string()), " ")
        print "%s\t QRep" % (fields[-1])
        log_file.write(fields[-1] + ",QREP,0,0,0\n")

    if msg.type() == LOG_ACK:
        fields = split(strip(msg.to_string()), " ")
        print "%s\t ACK" % (fields[-1])
        log_file.write(fields[-1] + ",ACK,0,0,0\n")

    if msg.type() == LOG_NAK:
        fields = split(strip(msg.to_string()), " ")
        print "%s\t NAK" % (fields[-1])
        log_file.write(fields[-1] + ",NAK,0,0,0\n")

    if msg.type() == LOG_RN16:
        fields = split(strip(msg.to_string()), " ")
        rn16 = fields[0].split(",")[0]
        snr = strip(fields[0].split(",")[1])
        tmp = int(rn16, 2)

        if msg.arg2() == LOG_ERROR:

            print "%s\t    %s RN16 w/ Error: %04X%s" % (fields[-1], fRed, tmp, fReset)
            log_file.write(fields[-1] + ",RN16,1," + "%04X" % tmp + "," + snr + "\n")
        else:
            print "%s\t    %s RN16: %04X%s" % (fields[-1], fBlue, tmp, fReset)
            log_file.write(fields[-1] + ",RN16,0," + "%04X" % tmp + "," + snr + "\n")

    if msg.type() == LOG_EPC:
        fields = split(strip(msg.to_string()), " ")
        epc = fields[0].split(",")[0]
        snr = strip(fields[0].split(",")[1])
        epc = epc[16:112]

        tmp = int(epc, 2)
        if msg.arg2() == LOG_ERROR:
            print "%s\t    %s EPC w/ Error: %024X%s" % (fields[-1], fRed, tmp, fReset)
            log_file.write(fields[-1] + ",EPC,1," + "%024X" % tmp + "," + snr + "\n")
        else:
            print "%s\t    %s EPC: %024X%s" % (fields[-1], fBlue, tmp, fReset)
            log_file.write(fields[-1] + ",EPC,0," + "%024X" % tmp + "," + snr + "\n")

    if msg.type() == LOG_EMPTY:
        fields = split(strip(msg.to_string()), " ")
        snr = strip(fields[0])
        print "%s\t    - Empty Slot - " % (fields[-1])
        log_file.write(fields[-1] + ",EMPTY,0,0," + snr + "\n")

    if msg.type() == LOG_COLLISION:
        fields = split(strip(msg.to_string()), " ")
        print "%s\t    - Collision - " % (fields[-1])
        log_file.write(fields[-1] + ",COLLISION,0,0,0\n")


if __name__ == '__main__':
    main()
