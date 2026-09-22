%% FULL STREAMING RX – THE ULTIMATE COHERENT GMSK RECEIVER
clc; clear; close all;

num_bits_data = 500;
Sps = 8;
Fs = 20e6/256;
f_IF = 30e3;
pilot_Len = 32;
preamble_Len = 63;
Sync_Len = 64;
BT = 0.3;
L = 4;

%% 2. (SYNC + PREAMBLE)
rng(999);
tx_preamble_bits = randi([0 1], preamble_Len, 1);
tx_sync_bits = repmat([1; 0], Sync_Len/2, 1);

ref_header_bits = [tx_sync_bits; tx_preamble_bits];
tx_ref_wave = generate_manual_gmsk(ref_header_bits, Sps, BT, L);

%% ================= 3. SPECTRUM ANALYZERS =================
specIF = spectrumAnalyzer('SampleRate', Fs, 'Title', 'RX IF Signal (30 kHz)', 'YLimits', [-140 -40]);
specBB = spectrumAnalyzer('SampleRate', Fs, 'Title', 'RX Baseband (GMSK Spectrum)', 'YLimits', [-140 -40]);
%% ================= 3. USRP RX =================
rx_radio = comm.SDRuReceiver( ...
    'Platform','B200', ...
    'SerialNum','31FD9C8', ...
    'CenterFrequency',400e6, ...
    'MasterClockRate',20e6, ...
    'DecimationFactor',256, ...
    'SamplesPerFrame',15000, ...
    'OutputDataType','double', ...
    'Gain', 55);

disp('=== STREAMING RX STARTED ===');

rx_buffer = complex([]);
MAX_BUF = 30000;

%% ================= 4. MAIN RX LOOP =================
plot_counter = 0;
while true
    rx_new = rx_radio();
    if isempty(rx_new)
        continue;
    end

    rx_buffer = [rx_buffer; rx_new];
    if length(rx_buffer) > MAX_BUF
        rx_buffer = rx_buffer(end-MAX_BUF+1:end);
    end
    if length(rx_buffer) < 15000
        continue;
    end

    %% ---- DDC ----
    t = (0:length(rx_buffer)-1).' / Fs;
    rx_bb = rx_buffer .* exp(-1j*2*pi*f_IF*t);
    plot_counter = plot_counter + 1;
     if mod(plot_counter, 3) == 0
        specIF(rx_new);
        specBB(rx_bb);
     end
    %% ---- AGC ----
    rx_bb = rx_bb / rms(rx_bb);

    %% ---- BLIND CFO ESTIMATION ----
    diff_cfo = rx_bb(2:end) .* conj(rx_bb(1:end-1));
    CFO = angle(sum(diff_cfo)) * (Fs / (2 * pi));

    % CFO CORRECTION (De-rotation)
    n = (0:length(rx_bb)-1).' / Fs;
    rx_bb_cfo = rx_bb .* exp(-1j * 2 * pi * CFO * n);

    %% ---- FRAME SYNCHRONIZATION ----
    [corr_val, lags] = xcorr(rx_bb_cfo, tx_ref_wave);
    [peak_val, peak_idx] = max(abs(corr_val));

    THRESH = 15 * mean(abs(corr_val));

    if peak_val < THRESH
        continue;
    end

    start_idx = lags(peak_idx) + 1;
    if start_idx < 1
       start_idx = 1;
    end
    fprintf('\n>>> FRAME FOUND! Lag: %d | Peak: %.2f | CFO: %.2f Hz\n', start_idx-1, peak_val, CFO);

    %% ---- BULK FRAME EXTRACTION ----
    total_frame_bits = Sync_Len + preamble_Len + pilot_Len + num_bits_data + pilot_Len;
    total_frame_samples = total_frame_bits * Sps;

    if start_idx + total_frame_samples - 1 > length(rx_bb_cfo)
        continue;
    end
    rx_frame_wave = rx_bb_cfo(start_idx : start_idx + total_frame_samples - 1);

    %% ---- BULK PHASE CORRECTION ----
    bulk_phase_offset = angle(corr_val(peak_idx));
    rx_frame_corrected = rx_frame_wave .* exp(-1j * bulk_phase_offset);

    %% ---- DEMODULATION (ROBUST DIFFERENTIAL) ----
    rx_delayed = [zeros(Sps, 1); rx_frame_corrected(1:end-Sps)];
    phase_diff_signal = angle(rx_frame_corrected .* conj(rx_delayed));

    eyediagram(phase_diff_signal(1:800), Sps*2);

    rx_all_bits = zeros(total_frame_bits, 1);
    for k = 1:total_frame_bits
        sample_idx = (k - 1) * Sps + 4;

        if sample_idx > length(phase_diff_signal)
            sample_idx = length(phase_diff_signal);
        end
        if phase_diff_signal(sample_idx) > 0
            rx_all_bits(k) = 1;
        else
            rx_all_bits(k) = 0;
        end
    end
        % 1. Raw Signal from USRP
    scatterplot(rx_new);
    title('1. Raw Received Signal (rx\_new)');

    % 2. Baseband Signal (Shifted to 0 Hz after DDC)
    scatterplot(rx_bb);
    title('2. Baseband Signal (rx\_bb)');

    % 3. CFO (Carrier Frequency Offset) Corrected Signal
    scatterplot(rx_bb_cfo);
    title('3. CFO Corrected (rx\_bb\_cfo)');

    % 4. Extracted Frame after Synchronization
    scatterplot(rx_frame_wave);
    title('4. Extracted Frame (rx\_frame\_wave)');

    % 5. Bulk Phase Corrected (De-rotated) Frame
    % This is where the MSK/GMSK circle should be seen most clearly
    scatterplot(rx_frame_corrected);
    title('5. Phase Corrected Frame (rx\_frame\_corrected)');

    % 6. Demodulated Bits
    % Note: These are no longer IQ symbols, but 1s and 0s, so they cluster only at (0,0) and (1,0).
    scatterplot(rx_all_bits);
    title('6. Demodulated Bits (rx\_all\_bits)');
    %% ---- BIT SLICING ----
    bit_idx_data_start = Sync_Len + preamble_Len + pilot_Len + 1;
    bit_idx_data_end   = bit_idx_data_start + num_bits_data - 1;

    rx_data_bits = rx_all_bits(bit_idx_data_start : bit_idx_data_end);

    disp('--- DATA DECODED ---');
    break
end
release(rx_radio);
disp('=== RX SESSION FINISHED ===');

%% ---- BER ----
rng(123);
tx_bits_actual = randi([0 1], num_bits_data, 1);
tx_bits_actual = tx_bits_actual(:);
rx_data_bits = rx_data_bits(:);

min_len = min(length(tx_bits_actual), length(rx_data_bits));
tx_bits_actual = logical(tx_bits_actual(1:min_len));
rx_data_bits = logical(rx_data_bits(1:min_len));

[numErrors, ber] = biterr(tx_bits_actual, rx_data_bits);
fprintf('\n================================================\n');
fprintf('   BER ANALYSIS RESULTS:\n');
fprintf('   Total Bits Transmitted : %d\n', length(tx_bits_actual));
fprintf('   Bit Errors              : %d\n', numErrors);
fprintf('   Bit Error Rate (BER)    : %.4f (%%%.2f)\n', ber, ber*100);
fprintf('================================================\n');

function wave = generate_manual_gmsk(bits, Sps, BT, L)
    nrz = bits * 2 - 1;
    up = zeros(length(nrz) * Sps, 1);
    up(1:Sps:end) = nrz;

    t = (-L/2 : 1/Sps : L/2).';
    Q = @(x) 0.5 * erfc(x / sqrt(2));
    c = sqrt(log(2));
    g = (1/2) * ( Q(2*pi*BT*(t - 0.5)/c) - Q(2*pi*BT*(t + 0.5)/c) );
    g = g / sum(g);

    freq_pulse = conv(up, g, 'same');
    phase = cumsum(freq_pulse) * (pi/2);
    wave = exp(1j * phase);
end
