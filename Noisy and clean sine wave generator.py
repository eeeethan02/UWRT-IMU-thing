import numpy as np
import matplotlib.pyplot as plt

# Parameters for the sine wave
frequency = 5  # Frequency of the sine wave in Hz
amplitude = 1  # Amplitude of the sine wave
duration = 2   # Duration in seconds
sampling_rate = 1000  # Sampling rate in Hz

# Generate the time axis
t = np.linspace(0, duration, int(sampling_rate * duration), endpoint=False)

# Generate the sine wave
sine_wave = amplitude * np.sin(2 * np.pi * frequency * t) + 4

# Add noise to the sine wave
noise_amplitude = 0.5
noise = noise_amplitude * np.random.normal(size=sine_wave.shape)
noisy_sine_wave = sine_wave + noise

# Save the clean sine wave data to a CSV file
clean_data = np.column_stack((t, sine_wave))  # Combine time and clean sine wave
np.savetxt("clean_sine_wave.csv", clean_data, delimiter=",", header="Time,Clean Sine Wave", comments='')

# Save the noisy sine wave data to a separate CSV file
noisy_data = np.column_stack((t, noisy_sine_wave))  # Combine time and noisy sine wave
np.savetxt("noisy_sine_wave.csv", noisy_data, delimiter=",", header="Time,Noisy Sine Wave", comments='')

# Plot the clean and noisy sine wave
plt.figure(figsize=(10, 6))
plt.plot(t, sine_wave, label="Clean Sine Wave")
plt.plot(t, noisy_sine_wave, label="Noisy Sine Wave", alpha=0.7)
plt.xlabel("Time [s]")
plt.ylabel("Amplitude")
plt.title("Sine Wave with Added Noise")
plt.legend()
plt.show()
