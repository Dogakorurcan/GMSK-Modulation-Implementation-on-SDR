%% GMSK ZERO-PADDING (TX)
num_bits_data = 500;
Sps = 8;
BT = 0.3;
L = 4;

%% Zero Padding
zero_Len = 100 * Sps;
tx_zeros = zeros(zero_Len, 1);

%% CLOCK SYNC & PREAMBLE & PILOTS
Sync_Len = 64;
tx_sync_bits = repmat([1; 0], Sync_Len/2, 1);

rng(999);
tx_preamble_bits = randi([0 1], 63, 1);

pilot_Len = 32;
rng(12345);
tx_pilot_bits = randi([0 1], pilot_Len, 1);

rng(123)
data_bits = randi([0 1], num_bits_data, 1);
save('tx_data_bit.mat', 'data_bits')

%% 3. FRAME
tx_frame_bits = [tx_sync_bits; tx_preamble_bits; tx_pilot_bits; data_bits; tx_pilot_bits];

%% 4. GMSK (DSP)
fprintf('1. Modulation: Converting data to a manual GMSK waveform...\n');

% 4.a: Bipolar (NRZ) (0 -> -1, 1 -> +1)
tx_nrz = tx_frame_bits * 2 - 1;

% 4.b: Upsampling
tx_up = zeros(length(tx_nrz) * Sps, 1);
tx_up(1:Sps:end) = tx_nrz;

% 4.c: Gauss (Q-Function)
t = (-L/2 : 1/Sps : L/2).';
% Q-function (erfc)
Q = @(x) 0.5 * erfc(x / sqrt(2));
% GMSK g(t)
c = sqrt(log(2));
g = (1/2) * ( Q(2*pi*BT*(t - 0.5)/c) - Q(2*pi*BT*(t + 0.5)/c) );
g = g / sum(g);

freq_pulse = conv(tx_up, g, 'same');

% 4.e: I/Q
phase = cumsum(freq_pulse) * (pi/2);
tx_baseband = exp(1j * phase);

% Zero padding
tx_waveform = [tx_zeros; tx_baseband; tx_zeros];

%% 5. DDC
masterclockrate = 20e6;
interpolate = 256;
Fs = masterclockrate / interpolate;
f_IF = 30e3;
fprintf('2. DDC: Shifting signal to %d Hz intermediate frequency (IF)...\n', f_IF);

N_tx = length(tx_waveform);
t_tx = (0:N_tx-1).' / Fs;

% Mixing
mixer_signal = exp(+1j * 2 * pi * f_IF * t_tx);
tx_IF_waveform = tx_waveform .* mixer_signal;

%% 6. USRP (BEACON MODE)
fprintf('Setting up USRP connection...\n');
tx_radio = comm.SDRuTransmitter(...
    'Platform',             'B200', ...
    'SerialNum',            '31FD9D5', ...
    'CenterFrequency',      400e6, ...
    'MasterClockRate',      masterclockrate, ...
    'Gain',                 50, ...
    'InterpolationFactor',  interpolate, ...
    'TransportDataType',    'int16');

fprintf('TX ready. Will transmit continuously, resetting each iteration.\n');
input('Press ENTER to start >> ');
while true
    tx_radio(tx_IF_waveform);
end
