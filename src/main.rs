#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

use core::mem;

use assign_resources::assign_resources;
use defmt_rtt as _;
use embassy_executor::{Executor, InterruptExecutor};
use embassy_rp::{
    bind_interrupts,
    gpio::{Level, Output},
    interrupt::{self, InterruptExt, Priority},
    peripherals,
    pio::{self, Config, FifoJoin, Pio, ShiftConfig, ShiftDirection},
    spi::{self, Spi},
    Peripheral,
};
use embassy_time::{Delay, Duration, Ticker};
use embedded_hal_bus::spi::ExclusiveDevice;
use fixed::traits::ToFixed;
use lsm6ds3tr::{
    interface::SpiInterface,
    registers::{GyroSampleRate, GyroScale},
    AccelSampleRate, AccelScale, AccelSettings, GyroSettings, LsmSettings, LSM6DS3TR,
};
use panic_probe as _;
use static_cell::StaticCell;

static INTERRUPT_EXECUTOR: InterruptExecutor = InterruptExecutor::new();
static DEFAULT_EXECUTOR: StaticCell<Executor> = StaticCell::new();

#[cortex_m_rt::interrupt]
unsafe fn SWI_IRQ_1() {
    INTERRUPT_EXECUTOR.on_interrupt()
}

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => pio::InterruptHandler<peripherals::PIO0>;
});

assign_resources! {
    imu: ImuResources {
        spi: SPI1,
        cs: PIN_13,
        sck: PIN_10,
        mosi: PIN_11,
        miso: PIN_12,
        tx_dma: DMA_CH2,
        rx_dma: DMA_CH3,
    }
    tone: ToneResources {
        data: PIN_6,
        bit_clock: PIN_7,
        lrck: PIN_8,
        pio: PIO0,
        dma: DMA_CH5,
    }
}

const SAMPLE_RATE: u32 = 9_000;
const BIT_DEPTH: u32 = 16;
const CHANNELS: u32 = 2;
const SAMPLE_COUNT: usize = SAMPLE_RATE as usize / 20;
const BUFFER_SIZE: usize = SAMPLE_COUNT * CHANNELS as usize;

#[cortex_m_rt::entry]
fn main() -> ! {
    let p = embassy_rp::init(Default::default());
    let resources = split_resources!(p);

    interrupt::SWI_IRQ_1.set_priority(Priority::P2);
    let interrupt_spawner = INTERRUPT_EXECUTOR.start(interrupt::SWI_IRQ_1);
    interrupt_spawner.must_spawn(tone_task(resources.tone));

    let executor = DEFAULT_EXECUTOR.init(Executor::new());
    executor.run(|spawner| spawner.must_spawn(low_priority_task(resources.imu)));
}

#[embassy_executor::task]
async fn low_priority_task(resources: ImuResources) {
    let mut config = spi::Config::default();
    config.frequency = 5_000_000;

    let spi = Spi::new(
        resources.spi,
        resources.sck,
        resources.mosi,
        resources.miso,
        resources.tx_dma,
        resources.rx_dma,
        config,
    );
    let spi_device = ExclusiveDevice::new(spi, Output::new(resources.cs, Level::High), Delay);

    let settings = LsmSettings::basic()
        .with_gyro(
            GyroSettings::new()
                .with_sample_rate(GyroSampleRate::_104Hz)
                .with_scale(GyroScale::_500DPS),
        )
        .with_accel(
            AccelSettings::new()
                .with_sample_rate(AccelSampleRate::_104Hz)
                .with_scale(AccelScale::_8G),
        );
    let mut imu = LSM6DS3TR::new(SpiInterface::new(spi_device)).with_settings(settings);
    imu.init().await.unwrap();

    let mut ticker = Ticker::every(Duration::from_hz(100));
    loop {
        ticker.next().await;

        let _gyro_data = imu.read_gyro().await.unwrap(); // Commenting this out fixes the problem
    }
}

include!(concat!(env!("OUT_DIR"), "/sine_lut.rs"));
include!(concat!(env!("OUT_DIR"), "/triangle_lut.rs"));
const LUT_SIZE: usize = SINE_LUT.len();

#[embassy_executor::task]
async fn tone_task(resources: ToneResources) {
    let mut pio = Pio::new(resources.pio, Irqs);

    #[rustfmt::skip]
    let pio_program = pio_proc::pio_asm!(
        ".side_set 2",
        "    set x, 14          side 0b01", // side 0bWB - W = Word Clock, B = Bit Clock
        "left_data:",
        "    out pins, 1        side 0b00",
        "    jmp x-- left_data  side 0b01",
        "    out pins 1         side 0b10",
        "    set x, 14          side 0b11",
        "right_data:",
        "    out pins 1         side 0b10",
        "    jmp x-- right_data side 0b11",
        "    out pins 1         side 0b00",
    );

    let data_pin = pio.common.make_pio_pin(resources.data);
    let bit_clock_pin = pio.common.make_pio_pin(resources.bit_clock);
    let left_right_clock_pin = pio.common.make_pio_pin(resources.lrck);

    let cfg = {
        let mut cfg = Config::default();
        cfg.use_program(
            &pio.common.load_program(&pio_program.program),
            &[&bit_clock_pin, &left_right_clock_pin],
        );
        cfg.set_out_pins(&[&data_pin]);
        let clock_frequency = SAMPLE_RATE * BIT_DEPTH * CHANNELS;
        cfg.clock_divider = (125_000_000. / clock_frequency as f64 / 2.).to_fixed();
        cfg.shift_out = ShiftConfig {
            threshold: 16,
            direction: ShiftDirection::Left,
            auto_fill: true,
        };
        cfg.fifo_join = FifoJoin::TxOnly;
        cfg
    };
    pio.sm0.set_config(&cfg);
    pio.sm0.set_enable(true);
    pio.sm0.set_pin_dirs(
        embassy_rp::pio::Direction::Out,
        &[&data_pin, &left_right_clock_pin, &bit_clock_pin],
    );

    let mut dma_ref = resources.dma.into_ref();
    let tx = pio.sm0.tx();

    // let mut generator = ToneGenerator::new();
    let mut generator = ToneGenerator::new();
    generator.set_volume(0.25);
    generator.set_frequency(400.0);

    let mut front_buffer = [0; BUFFER_SIZE];
    let mut back_buffer = [0; BUFFER_SIZE];

    loop {
        let dma_future = tx.dma_push(dma_ref.reborrow(), &front_buffer);
        generator.generate_samples(&mut back_buffer);
        dma_future.await;
        mem::swap(&mut back_buffer, &mut front_buffer);
    }
}

struct ToneGenerator {
    offset: i32,
    increment: i32,
    amplitude: u16,
    new_amplitude: Option<u16>,
    prev_val: i16,
}

impl ToneGenerator {
    fn new() -> Self {
        Self {
            offset: 0,
            increment: 0,
            amplitude: u16::MAX,
            new_amplitude: None,
            prev_val: 0,
        }
    }

    fn set_frequency(&mut self, frequency: f32) {
        let samp_per_cyc = SAMPLE_RATE as f32 / frequency;
        let fincr = LUT_SIZE as f32 / samp_per_cyc;
        let incr = (((1 << 24) as f32) * fincr) as i32;
        self.increment = incr;
    }

    fn set_volume(&mut self, amplitude: f32) {
        let amplitude = amplitude.clamp(0.0, 1.0);
        let amplitude = (amplitude * u16::MAX as f32) as u16;
        self.new_amplitude = Some(amplitude);
    }

    // Code from https://jamesmunns.com/blog/fixed-point-math/
    fn generate_samples(&mut self, buffer: &mut [u16]) {
        for frame in buffer.chunks_mut(2) {
            let val = self.offset as u32;
            let idx_now = ((val >> 24) & 0xFF) as u8;
            let idx_nxt = idx_now.wrapping_add(1);
            let base_val = SINE_LUT[idx_now as usize] as i32;
            let next_val = SINE_LUT[idx_nxt as usize] as i32;
            let off = ((val >> 16) & 0xFF) as i32;
            let cur_weight = base_val.wrapping_mul((LUT_SIZE as i32).wrapping_sub(off));
            let nxt_weight = next_val.wrapping_mul(off);
            let ttl_weight = cur_weight.wrapping_add(nxt_weight);
            let ttl_val = (ttl_weight >> 8) as i16;

            if let Some(new_amplitude) = self.new_amplitude {
                if ttl_val.signum() != self.prev_val.signum() {
                    // This is the first sample after a zero crossing,
                    // change the gain now so it's doesn't cause clicking.
                    self.amplitude = new_amplitude;
                    self.new_amplitude = None;
                }
            }
            self.prev_val = ttl_val;

            let ttl_val = ((ttl_val as i32 * self.amplitude as i32) >> 16) as u16;

            frame[0] = ttl_val;
            frame[1] = ttl_val;

            self.offset = self.offset.wrapping_add(self.increment);
        }
    }
}
