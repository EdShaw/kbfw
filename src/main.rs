#![no_std]
#![no_main]

use core::future::Future;
use core::pin::Pin;
use core::sync::atomic::{AtomicBool, Ordering};

use defmt::{debug, error, info, unreachable};
use embassy_executor::Spawner;
use embassy_futures::select::select_slice;
use embassy_rp::bind_interrupts;
use embassy_rp::block::ImageDef;
use embassy_rp::gpio::{AnyPin, Input, Output};
use embassy_rp::peripherals::{PIO0, USB};
use embassy_rp::pio;
use embassy_rp::pio::Pio;
use embassy_rp::pio_programs::ws2812::{PioWs2812, PioWs2812Program};
use embassy_rp::usb;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Ticker};
use embassy_usb::class::cdc_acm::CdcAcmClass;
use embassy_usb::class::hid::{
    HidReaderWriter, ReportId as HidReportId, RequestHandler as HidRequestHandler,
    State as HidState,
};
use embassy_usb::control::OutResponse;
use embassy_usb::{Builder as UsbBuilder, Handler as UsbHandler};
use futures::pin_mut;
use kbfw_embassy::keycode::KeyCode;
use smart_leds::RGB8;
use usbd_hid::descriptor::{KeyboardReport, SerializedDescriptor};
use {defmtusb as _, panic_probe as _};

use kbfw_embassy::matrix::Matrix;

#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: ImageDef = ImageDef::secure_exe();

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => usb::InterruptHandler<USB>;
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
});

/// Input a value 0 to 255 to get a color value
/// The colours are a transition r - g - b - back to r.
fn wheel(mut wheel_pos: u8) -> RGB8 {
    wheel_pos = 255 - wheel_pos;
    if wheel_pos < 85 {
        return (255 - wheel_pos * 3, 0, wheel_pos * 3).into();
    }
    if wheel_pos < 170 {
        wheel_pos -= 85;
        return (0, wheel_pos * 3, 255 - wheel_pos * 3).into();
    }
    wheel_pos -= 170;
    (wheel_pos * 3, 255 - wheel_pos * 3, 0).into()
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Start");
    let p = embassy_rp::init(Default::default());

    let driver = usb::Driver::new(p.USB, Irqs);

    let mut config = embassy_usb::Config::new(0x1209, 0xc0fe);
    config.manufacturer = Some("EdddCrab");
    config.product = Some("Paddd");
    config.serial_number = Some("12345678");
    config.max_power = 500;
    config.max_packet_size_0 = 64;

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    // You can also add a Microsoft OS descriptor.
    let mut msos_descriptor = [0; 256];
    let mut control_buf = [0; 64];

    let mut request_handler = MyRequestHandler {};
    let mut device_handler = MyDeviceHandler::new();

    let mut hid_state = HidState::new();
    let mut logger_state = embassy_usb::class::cdc_acm::State::new();
    let mut builder = UsbBuilder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut msos_descriptor,
        &mut control_buf,
    );

    builder.handler(&mut device_handler);

    // Create classes on the builder.
    let config = embassy_usb::class::hid::Config {
        report_descriptor: KeyboardReport::desc(),
        request_handler: None,
        poll_ms: 60,
        max_packet_size: 64,
    };
    let hid = HidReaderWriter::<_, 1, 8>::new(&mut builder, &mut hid_state, config);

    let logger_class = CdcAcmClass::new(&mut builder, &mut logger_state, 64);
    let usb_log = async {
        defmtusb::logger(logger_class.split().0, 64).await;

        crate::unreachable!()
    };

    // Build the builder.
    let mut usb = builder.build();

    // Run the USB device.
    let usb_fut = usb.run();

    let (reader, mut writer) = hid.split();

    // ROW2COL
    // Input: ["F4", "F5", "F6", "F7"],
    // Output: ["B1", "E6", "D7", "C6", "D4"]
    // Aka
    // Input: PIN_29, PIN_28, PIN_27, PIN_26
    // Output: PIN_22, PIN_7, PIN_6, PIN_5, PIN_4

    let key_event_channel = Channel::new();

    let matrix_inputs = [
        embassy_rp::gpio::Pin::degrade(p.PIN_29),
        embassy_rp::gpio::Pin::degrade(p.PIN_28),
        embassy_rp::gpio::Pin::degrade(p.PIN_27),
        embassy_rp::gpio::Pin::degrade(p.PIN_26),
    ]
    .map(|in_pin| {
        let mut in_pin = Input::new(AnyPin::from(in_pin), embassy_rp::gpio::Pull::Down);
        in_pin.set_schmitt(true);
        in_pin
    });
    let matrix_outputs = [
        embassy_rp::gpio::Pin::degrade(p.PIN_22),
        embassy_rp::gpio::Pin::degrade(p.PIN_7),
        embassy_rp::gpio::Pin::degrade(p.PIN_6),
        embassy_rp::gpio::Pin::degrade(p.PIN_5),
        embassy_rp::gpio::Pin::degrade(p.PIN_4),
    ]
    .map(|out_pin| Output::new(out_pin, embassy_rp::gpio::Level::Low));
    let mut matrix = Matrix::new(matrix_inputs, matrix_outputs);
    let scan_matrix = matrix.scan(key_event_channel.sender());

    // Do stuff with the class!
    let mut report = KeyboardReport::default();
    let report_writer = async {
        loop {
            info!("Waiting for key event");
            let ev = key_event_channel.receive().await;
            info!("Got key event: {}", ev);

            // TODO KEYMAP
            let keycode = KeyCode::KeyF;
            if ev.pressed {
                register_key(&mut report, keycode);
            } else {
                unregister_key(&mut report, keycode)
            }

            info!("Writing report: {} {}", report.modifier, report.keycodes);
            match writer.write_serialize(&report).await {
                Ok(()) => {}
                Err(e) => error!("Failed to send report: {:?}", e),
            };
        }
    };

    let report_reader = async {
        reader.run(false, &mut request_handler).await;
    };

    let Pio {
        mut common, sm0, ..
    } = Pio::new(p.PIO0, Irqs);

    // This is the number of leds in the string. Helpfully, the sparkfun thing plus and adafruit
    // feather boards for the 2040 both have one built in.
    const NUM_LEDS: usize = 1;
    let mut data = [RGB8::default(); NUM_LEDS];

    let program = PioWs2812Program::new(&mut common);
    let mut ws2812 = PioWs2812::new(&mut common, sm0, p.DMA_CH0, p.PIN_25, &program);

    let led_fut = async {
        let mut ticker = Ticker::every(Duration::from_millis(10));
        loop {
            for j in 0..(256 * 5) {
                for i in 0..NUM_LEDS {
                    data[i] =
                        wheel((((i * 256) as u16 / NUM_LEDS as u16 + j as u16) & 255) as u8) / 32;
                    debug!("R: {} G: {} B: {}", data[i].r, data[i].g, data[i].b);
                }
                ws2812.write(&data).await;

                ticker.next().await;
            }
        }
    };

    // Run everything concurrently.
    pin_mut!(usb_fut);
    pin_mut!(usb_log);
    pin_mut!(scan_matrix);
    pin_mut!(report_writer);
    pin_mut!(report_reader);
    pin_mut!(led_fut);
    let futures: [Pin<&mut dyn Future<Output = _>>; 6] = [
        usb_fut,
        usb_log,
        scan_matrix,
        report_writer,
        report_reader,
        led_fut,
    ];
    pin_mut!(futures);
    match select_slice(futures).await {}
}

// Program metadata for `picotool info`.
// This isn't needed, but it's recomended to have these minimal entries.
#[link_section = ".bi_entries"]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"kbfw"),
    embassy_rp::binary_info::rp_program_description!(c"Keyboard firmware"),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

struct MyRequestHandler {}

impl HidRequestHandler for MyRequestHandler {
    fn get_report(&mut self, id: HidReportId, _buf: &mut [u8]) -> Option<usize> {
        info!("Get report for {:?}", id);
        None
    }

    fn set_report(&mut self, id: HidReportId, data: &[u8]) -> OutResponse {
        info!("Set report for {:?}: {=[u8]}", id, data);
        OutResponse::Accepted
    }

    fn set_idle_ms(&mut self, id: Option<HidReportId>, dur: u32) {
        info!("Set idle rate for {:?} to {:?}", id, dur);
    }

    fn get_idle_ms(&mut self, id: Option<HidReportId>) -> Option<u32> {
        info!("Get idle rate for {:?}", id);
        None
    }
}

struct MyDeviceHandler {
    configured: AtomicBool,
}

impl MyDeviceHandler {
    fn new() -> Self {
        MyDeviceHandler {
            configured: AtomicBool::new(false),
        }
    }
}

impl UsbHandler for MyDeviceHandler {
    fn enabled(&mut self, enabled: bool) {
        self.configured.store(false, Ordering::Relaxed);
        if enabled {
            info!("Device enabled");
        } else {
            info!("Device disabled");
        }
    }

    fn reset(&mut self) {
        self.configured.store(false, Ordering::Relaxed);
        info!("Bus reset, the Vbus current limit is 100mA");
    }

    fn addressed(&mut self, addr: u8) {
        self.configured.store(false, Ordering::Relaxed);
        info!("USB address set to: {}", addr);
    }

    fn configured(&mut self, configured: bool) {
        self.configured.store(configured, Ordering::Relaxed);
        if configured {
            info!(
                "Device configured, it may now draw up to the configured current limit from Vbus."
            )
        } else {
            info!("Device is no longer configured, the Vbus current limit is 100mA.");
        }
    }
}

fn register_key(r: &mut KeyboardReport, key: KeyCode) {
    if key.is_modifier() {
        register_modifier(r, key.as_modifier_mask());
    } else if key.is_basic() {
        register_keycode(r, key);
    }
}

fn unregister_key(r: &mut KeyboardReport, key: KeyCode) {
    if key.is_modifier() {
        unregister_modifier(r, key.as_modifier_mask());
    } else if key.is_basic() {
        unregister_keycode(r, key);
    }
}

fn register_keycode(r: &mut KeyboardReport, key: KeyCode) {
    if let Some(k) = r.keycodes.iter_mut().find(|k| **k == KeyCode::None as u8) {
        *k = key as u8;
    }
}

fn unregister_keycode(r: &mut KeyboardReport, key: KeyCode) {
    if let Some(k) = r.keycodes.iter_mut().find(|k| **k == key as u8) {
        *k = KeyCode::None as u8;
    }
}

fn register_modifier(r: &mut KeyboardReport, modifier_mask: u8) {
    r.modifier |= modifier_mask;
}

fn unregister_modifier(r: &mut KeyboardReport, modifier_mask: u8) {
    r.modifier &= !modifier_mask;
}
