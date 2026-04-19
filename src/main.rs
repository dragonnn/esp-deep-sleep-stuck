#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use defmt::unwrap;
use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::timer::timg::TimerGroup;
use panic_rtt_target as _;

extern crate alloc;

mod ieee802154;

// Track how many deep-sleep cycles we've survived.
// Persists across deep sleep resets (lives in RTC FAST RAM).
#[esp_hal::ram(unstable(rtc_fast, persistent))]
static mut DEEP_SLEEP_COUNT: u32 = 0;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[embassy_executor::task]
pub async fn temp_task(temperature: esp_hal::tsens::TemperatureSensor<'static>) {
    loop {
        let temp = temperature.get_temperature().to_celsius();
        info!("Temperature: {}°C", temp);
        // Read temp very frequently to keep SAR ADC area active
        embassy_time::Timer::after(Duration::from_millis(10)).await;
    }
}

#[embassy_executor::task]
pub async fn led_task(mut led: esp_hal::gpio::Output<'static>) {
    loop {
        led.toggle();
        embassy_time::Timer::after(embassy_time::Duration::from_millis(100)).await;
    }
}

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    // generator version: 1.2.0

    rtt_target::rtt_init_defmt!();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 65536);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_interrupt =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

    let count = unsafe { DEEP_SLEEP_COUNT };
    warn!(
        "=== Boot #{} (deep sleep cycles survived so far) ===",
        count
    );

    let temperature =
        unwrap!(esp_hal::tsens::TemperatureSensor::new(peripherals.TSENS, Default::default()).ok());

    let led = esp_hal::gpio::Output::new(peripherals.GPIO15, false.into(), Default::default());

    spawner.spawn(temp_task(temperature).unwrap());
    spawner.spawn(led_task(led).unwrap());

    // Start IEEE 802.15.4 radio with rx_when_idle=true and frequent TX
    let ieee802154 = esp_radio::ieee802154::Ieee802154::new(peripherals.IEEE802154);
    spawner.spawn(crate::ieee802154::run(ieee802154, spawner).unwrap());

    let mut rtc = esp_hal::rtc_cntl::Rtc::new(peripherals.LPWR);

    // Let radio run for a bit, varying timing to hit different calibration phases
    let delay_ms = 200 + ((count * 37) % 300) as u64;
    Timer::after(Duration::from_millis(delay_ms)).await;

    // Replicate production shutdown flow:
    // 1. Signal ieee802154 task to exit → drops Ieee802154
    //    This triggers PhyClockGuard::drop() → enable_phy(false) → clk_i2c_mst_en = false
    //    But MAC hardware never gets Command::Stop, so it may still be active on regi2c bus
    warn!("Signaling radio shutdown...");
    crate::ieee802154::trigger_shutdown();
    crate::ieee802154::wait_radio_dropped().await;
    warn!("Radio dropped!");

    // 2. Short delay matching production (200ms)
    Timer::after(Duration::from_millis(200)).await;

    // Increment the persistent counter before we attempt deep sleep.
    unsafe {
        DEEP_SLEEP_COUNT = count + 1;
    }

    let timer = esp_hal::rtc_cntl::sleep::TimerWakeupSource::new(Duration::from_secs(1).into());
    warn!(
        "Attempting deep sleep #{} — if we hang, clk_i2c_mst_en is disabled but MAC is still active!",
        count
    );

    // This call can hang in regi2c_write during clock calibration because:
    // - clk_i2c_mst_en was disabled by PhyClockGuard::drop()
    // - MAC hardware was never stopped (no Command::Stop)
    // - regi2c_enable_block() re-enables clk_i2c_mst_en but the bus state is corrupted
    rtc.sleep_deep(&[&timer]);

    loop {
        info!("Hello, world!");
        Timer::after(Duration::from_secs(10)).await;
    }
}
