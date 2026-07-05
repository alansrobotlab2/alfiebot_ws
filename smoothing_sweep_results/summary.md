| # | Config | MSE | Jerk | Base Jerk | R.Arm Jerk | Head Jerk | Base HF% | R.Arm HF% | Head HF% |
|---|--------|-----|------|-----------|------------|-----------|----------|-----------|----------|
| D1_no_filter | No post-filter (ensemble+EMA only) | 0.0062 | 0.81x | 0.38x | 0.62x | 1.78x | 10.5% | 0.1% | 0.8% |
| D2_savgol_w3 | SavGol window=3 | 0.0053 | 0.77x | 0.36x | 0.62x | 1.77x | 7.2% | 0.1% | 0.8% |
| D3_savgol_w5 | SavGol window=5 (current) | 0.0056 | 0.90x | 0.45x | 0.85x | 2.12x | 18.5% | 0.2% | 1.2% |
| D4_savgol_w7 | SavGol window=7 | 0.0047 | 0.89x | 0.47x | 0.86x | 2.09x | 13.6% | 0.2% | 1.1% |
| D5_butter_7hz | Butterworth 7Hz cutoff | 0.0045 | 0.76x | 0.41x | 0.68x | 1.82x | 12.0% | 0.1% | 0.9% |
| D6_butter_5hz | Butterworth 5Hz cutoff | 0.0052 | 0.85x | 0.46x | 0.65x | 2.07x | 17.5% | 0.1% | 1.0% |
| D7_butter_3hz | Butterworth 3Hz cutoff | 0.0043 | 0.72x | 0.38x | 0.49x | 1.39x | 13.6% | 0.1% | 0.9% |