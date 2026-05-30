#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, Ordering};
use daisy_embassy::{audio::HALF_DMA_BUFFER_LENGTH, hal, new_daisy_board};
use defmt::{info, unwrap};
use embassy_executor::Spawner;
use embassy_stm32::{
    Peri,
    adc::{Adc, SampleTime},
    peripherals::{ADC1, PC0},
};
use embassy_time::Timer;
use micromath::F32Ext;
use {defmt_rtt as _, panic_probe as _};

const SAMPLE_RATE: f32 = 48000.0;
const BASE_FREQUENCY: f32 = 55.0; // A1 (55 Hz at 0V)
const CV_SCALE: f32 = 3.3; // ADC voltage range (V)
const SMOOTHING_FACTOR: f32 = 0.98; // Frequency smoothing (0.0 = no smoothing, 0.99 = heavy)

static CV_VOLTAGE: AtomicU32 = AtomicU32::new(0);

#[derive(Clone, Copy)]
enum Waveform {
    Sine,
    Triangle,
    Sawtooth,
    Square,
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("CV Oscillator Starting...");

    let config = daisy_embassy::default_rcc();
    let p = hal::init(config);
    let board = new_daisy_board!(p);

    let adc = Adc::new(p.ADC1);
    spawner.spawn(cv_reader_task(adc, board.pins.d15)).unwrap();

    let interface = board
        .audio_peripherals
        .prepare_interface(Default::default())
        .await;

    let mut interface = unwrap!(interface.start_interface().await);

    info!("Audio interface started");

    let mut phase: f32 = 0.0;
    let mut current_freq: f32 = BASE_FREQUENCY;
    let mut buffer = [0u32; HALF_DMA_BUFFER_LENGTH];
    let waveform = Waveform::Triangle;

    unwrap!(
        interface
            .start_callback(|_input, output| {
                // Read CV voltage from atomic storage
                let cv_bits = CV_VOLTAGE.load(Ordering::Relaxed);
                let cv_voltage = f32::from_bits(cv_bits);

                // Calculate target frequency with 1V/octave formula
                // freq = base_freq × 2^(cv_volts)
                let target_freq = BASE_FREQUENCY * (2.0_f32).powf(cv_voltage);

                // Smooth frequency changes
                current_freq =
                    current_freq * SMOOTHING_FACTOR + target_freq * (1.0 - SMOOTHING_FACTOR);

                // Phase increment per sample
                let phase_increment = current_freq / SAMPLE_RATE;

                for chunk in buffer.chunks_mut(2) {
                    let sample = match waveform {
                        Waveform::Sine => sine_wave(phase),
                        Waveform::Triangle => triangle_wave(phase),
                        Waveform::Sawtooth => sawtooth_wave(phase),
                        Waveform::Square => square_wave(phase),
                    };

                    let sample_u32 = f32_to_u24(sample * 0.5); // Reduce volume to 50%

                    // Stereo
                    chunk[0] = sample_u32;
                    chunk[1] = sample_u32;

                    phase += phase_increment;
                    if phase >= 1.0 {
                        phase -= 1.0;
                    }
                }
                output.copy_from_slice(&buffer);
            })
            .await
    );
}

#[embassy_executor::task]
async fn cv_reader_task(mut adc: Adc<'static, ADC1>, mut pin: Peri<'static, PC0>) {
    info!("CV reader task started");

    loop {
        let adc_value = adc.blocking_read(&mut pin, SampleTime::CYCLES387_5);
        let voltage = (adc_value as f32 / 65535.0) * CV_SCALE;

        CV_VOLTAGE.store(voltage.to_bits(), Ordering::Relaxed);

        Timer::after_millis(1).await;
    }
}

fn sine_wave(phase: f32) -> f32 {
    // Use micromath for fast sine approximation
    (phase * 2.0 * core::f32::consts::PI).sin()
}

fn triangle_wave(phase: f32) -> f32 {
    if phase < 0.5 {
        4.0 * phase - 1.0
    } else {
        -4.0 * phase + 3.0
    }
}

fn sawtooth_wave(phase: f32) -> f32 {
    2.0 * phase - 1.0
}

fn square_wave(phase: f32) -> f32 {
    if phase < 0.5 { 1.0 } else { -1.0 }
}

/// Convert f32 sample (-1.0 to +1.0) to u32 (24-bit MSB-aligned)
#[inline(always)]
fn f32_to_u24(sample: f32) -> u32 {
    let clamped = sample.clamp(-1.0, 1.0);

    let scaled = (clamped * 8_388_607.0) as i32;

    // Shift left by 8 bits (MSB-aligned in 32-bit word)
    (scaled << 8) as u32
}
