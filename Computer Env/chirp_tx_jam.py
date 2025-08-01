#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: chirp_tx_jam
# Author: arash
# GNU Radio version: 3.10.10.0

from gnuradio import analog
from gnuradio import blocks
from gnuradio import filter
from gnuradio.filter import firdes
from gnuradio import gr
from gnuradio.fft import window
import sys
import signal
from argparse import ArgumentParser
from gnuradio.eng_arg import eng_float, intx
from gnuradio import eng_notation
from gnuradio import radar




class chirp_tx_jam(gr.top_block):

    def __init__(self, SNR=100, frame_size=32):
        gr.top_block.__init__(self, "chirp_tx_jam", catch_exceptions=True)

        ##################################################
        # Variables
        ##################################################
        self.samp_rate = samp_rate = int(20e6)
        self.noise_var = noise_var = 1
        self.frame_size = frame_size
        self.SNR = SNR
        self.LO = LO = 5800000000
        self.Gain = Gain = 62

        ##################################################
        # Blocks
        ##################################################

        self.radar_signal_generator_fmcw_c_0 = radar.signal_generator_fmcw_c(samp_rate, (int(0.2*1e6)), (int(0.2*1e6)), 0, -int(500e3), int(1e6), 1, "packet_len")
        self.low_pass_filter_0 = filter.fir_filter_ccf(
            1,
            firdes.low_pass(
                1,
                samp_rate,
                int(100e3),
                int(10e3),
                window.WIN_HAMMING,
                6.76))
        self.high_pass_filter_0_0 = filter.fir_filter_ccf(
            1,
            firdes.high_pass(
                1,
                samp_rate,
                int(130e3),
                10000,
                window.WIN_HAMMING,
                6.76))
        self.high_pass_filter_0 = filter.fir_filter_ccf(
            1,
            firdes.high_pass(
                1,
                samp_rate,
                int(130e3),
                10000,
                window.WIN_HAMMING,
                6.76))
        self.blocks_throttle2_0 = blocks.throttle( gr.sizeof_gr_complex*1, samp_rate, True, 0 if "auto" == "auto" else max( int(float(0.1) * samp_rate) if "auto" == "time" else int(0.1), 1) )
        self.blocks_stream_to_vector_0_1_0 = blocks.stream_to_vector(gr.sizeof_float*1, frame_size)
        self.blocks_stream_to_vector_0_0_0_0 = blocks.stream_to_vector(gr.sizeof_float*1, frame_size)
        self.blocks_stream_to_vector_0_0 = blocks.stream_to_vector(gr.sizeof_float*1, frame_size)
        self.blocks_stream_to_vector_0 = blocks.stream_to_vector(gr.sizeof_float*1, frame_size)
        self.blocks_complex_to_real_0_0_0 = blocks.complex_to_real(1)
        self.blocks_complex_to_real_0 = blocks.complex_to_real(1)
        self.blocks_complex_to_imag_0_0_0 = blocks.complex_to_imag(1)
        self.blocks_complex_to_imag_0 = blocks.complex_to_imag(1)
        self.analog_noise_source_x_0 = analog.noise_source_c(analog.GR_GAUSSIAN, (1/(10**(SNR/10))), 0)
        self.Q_noise = blocks.probe_signal_vf(frame_size)
        self.Q_chirp = blocks.probe_signal_vf(frame_size)
        self.I_noise = blocks.probe_signal_vf(frame_size)
        self.I_chirp = blocks.probe_signal_vf(frame_size)


        ##################################################
        # Connections
        ##################################################
        self.connect((self.analog_noise_source_x_0, 0), (self.low_pass_filter_0, 0))
        self.connect((self.blocks_complex_to_imag_0, 0), (self.blocks_stream_to_vector_0_0, 0))
        self.connect((self.blocks_complex_to_imag_0_0_0, 0), (self.blocks_stream_to_vector_0_0_0_0, 0))
        self.connect((self.blocks_complex_to_real_0, 0), (self.blocks_stream_to_vector_0, 0))
        self.connect((self.blocks_complex_to_real_0_0_0, 0), (self.blocks_stream_to_vector_0_1_0, 0))
        self.connect((self.blocks_stream_to_vector_0, 0), (self.I_chirp, 0))
        self.connect((self.blocks_stream_to_vector_0_0, 0), (self.Q_chirp, 0))
        self.connect((self.blocks_stream_to_vector_0_0_0_0, 0), (self.Q_noise, 0))
        self.connect((self.blocks_stream_to_vector_0_1_0, 0), (self.I_noise, 0))
        self.connect((self.blocks_throttle2_0, 0), (self.high_pass_filter_0, 0))
        self.connect((self.high_pass_filter_0, 0), (self.blocks_complex_to_imag_0, 0))
        self.connect((self.high_pass_filter_0, 0), (self.blocks_complex_to_real_0, 0))
        self.connect((self.high_pass_filter_0_0, 0), (self.blocks_complex_to_imag_0_0_0, 0))
        self.connect((self.high_pass_filter_0_0, 0), (self.blocks_complex_to_real_0_0_0, 0))
        self.connect((self.low_pass_filter_0, 0), (self.high_pass_filter_0_0, 0))
        self.connect((self.radar_signal_generator_fmcw_c_0, 0), (self.blocks_throttle2_0, 0))


    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.blocks_throttle2_0.set_sample_rate(self.samp_rate)
        self.high_pass_filter_0.set_taps(firdes.high_pass(1, self.samp_rate, int(130e3), 10000, window.WIN_HAMMING, 6.76))
        self.high_pass_filter_0_0.set_taps(firdes.high_pass(1, self.samp_rate, int(130e3), 10000, window.WIN_HAMMING, 6.76))
        self.low_pass_filter_0.set_taps(firdes.low_pass(1, self.samp_rate, int(100e3), int(10e3), window.WIN_HAMMING, 6.76))

    def get_noise_var(self):
        return self.noise_var

    def set_noise_var(self, noise_var):
        self.noise_var = noise_var

    def get_frame_size(self):
        return self.frame_size

    def set_frame_size(self, frame_size):
        self.frame_size = frame_size

    def get_SNR(self):
        return self.SNR

    def set_SNR(self, SNR):
        self.SNR = SNR
        self.analog_noise_source_x_0.set_amplitude((1/(10**(self.SNR/10))))

    def get_LO(self):
        return self.LO

    def set_LO(self, LO):
        self.LO = LO

    def get_Gain(self):
        return self.Gain

    def get_I_noise(self):
        return self.I_noise
    def get_Q_noise(self):
        return self.Q_noise
    def get_I_chirp(self):
        return self.I_chirp
    def get_Q_chirp(self):
        return self.Q_chirp




def main(top_block_cls=chirp_tx_jam, options=None):
    tb = top_block_cls()

    def sig_handler(sig=None, frame=None):
        tb.stop()
        tb.wait()

        sys.exit(0)

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    tb.start()

    try:
        input('Press Enter to quit: ')
    except EOFError:
        pass
    tb.stop()
    tb.wait()


if __name__ == '__main__':
    main()
