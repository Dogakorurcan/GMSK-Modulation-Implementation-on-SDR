# GMSK Transmitter / Receiver over USRP B200

A MATLAB implementation of a Gaussian Minimum Shift Keying (GMSK) transceiver over two USRP B200 radios. Unlike a toolbox-based version, the modulator and demodulator are built from scratch: a Gaussian pulse-shaping filter derived from the Q-function, manual phase integration for modulation, and a differential (non-coherent) phase detector for demodulation.

## Files

- `GMSK_tx.m` — builds the frame, applies manual GMSK modulation, upconverts to a digital IF, and streams it continuously via USRP.
- `GMSK_rx.m` — streams and buffers received samples, applies AGC, estimates and corrects CFO, detects the frame by correlation, demodulates differentially, and reports the BER.

## Frame structure

Sync (64 bits) → Preamble (63 bits) → Pilot (32 bits) → Data (500 bits) → Pilot (32 bits), zero-padded on both ends before transmission.

## How it works

**TX:** bits → bipolar NRZ → Gaussian pulse shaping (`BT = 0.3`, span `L = 4`) → phase integration → GMSK waveform → upconvert to 30 kHz IF → transmit.

**RX:** receive & buffer → down-convert → AGC → blind CFO estimation (autocorrelation) and correction → frame sync via cross-correlation → phase de-rotation → differential demodulation with eye-diagram-based bit slicing → BER.

## Requirements

- MATLAB + Communications Toolbox + Support Package for USRP Radio
- 2× USRP B200 (or compatible), one TX / one RX

## Usage

1. Set your own `SerialNum` in both scripts.
2. Run `GMSK_tx.m`, press Enter to start streaming.
3. Run `GMSK_rx.m` on the second radio — it reports CFO, shows constellation/eye-diagram plots, and prints the final BER once a frame is decoded.

## License

Add a license (MIT is a common choice for portfolio projects) before publishing.
