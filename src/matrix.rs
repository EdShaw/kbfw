use core::{
    convert::Infallible,
    pin::{pin, Pin},
    u16,
};

use defmt::{debug, trace, Format};
use embassy_futures::select::select_slice;
use embassy_rp::gpio::{Flex, Pull};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::Timer;
use embedded_hal_1::digital::{InputPin, OutputPin};
use embedded_hal_async::digital::Wait;

#[derive(Copy, Clone, Debug, Format, Default)]
pub struct KeyState {
    pub(crate) high: bool,
}

#[derive(Copy, Clone, Debug, Format)]
pub struct KeyEvent {
    pub key: u16,
    pub pressed: bool,
}

type KeyChannel<'a> = embassy_sync::channel::Sender<'a, CriticalSectionRawMutex, KeyEvent, 32>;

pub trait MatrixMapping<const INS_SIZE: usize> {
    fn map(self, out_idx: usize, in_idx: usize) -> u16;
}

#[derive(Clone, Copy, Debug, Format)]
pub struct MatrixOffset<const OFFSET: u16> {}
impl<const OFFSET: u16, const INS_SIZE: usize> MatrixMapping<INS_SIZE> for MatrixOffset<OFFSET> {
    fn map(self, out_idx: usize, in_idx: usize) -> u16 {
        (out_idx * INS_SIZE + in_idx) as u16 + OFFSET
    }
}

pub struct Matrix<'a, Out, M, const INS_SIZE: usize, const OUTS_SIZE: usize>
where
    Out: OutputPin<Error = Infallible>,
    M: MatrixMapping<INS_SIZE> + Copy,
{
    ins: [Flex<'a>; INS_SIZE],
    outs: [Out; OUTS_SIZE],

    key_states: [[KeyState; INS_SIZE]; OUTS_SIZE],
    mapping: M,
}

impl<'a, Out, M, const INS_SIZE: usize, const OUTS_SIZE: usize>
    Matrix<'a, Out, M, INS_SIZE, OUTS_SIZE>
where
    Out: OutputPin<Error = Infallible>,
    M: MatrixMapping<INS_SIZE> + Copy,
{
    pub fn new(ins: [Flex<'a>; INS_SIZE], outs: [Out; OUTS_SIZE], mapping: M) -> Self {
        const {
            assert!(INS_SIZE <= u8::MAX as usize);
            assert!(OUTS_SIZE <= u8::MAX as usize);
        };

        Self {
            ins,
            outs,
            key_states: [[KeyState::default(); INS_SIZE]; OUTS_SIZE],
            mapping,
        }
    }

    pub async fn scan(&mut self, event_channel: KeyChannel<'_>) -> ! {
        for out_pin in self.outs.iter_mut() {
            let Ok(()) = out_pin.set_low();
        }

        for in_pin in self.ins.iter_mut() {
            in_pin.set_low();
            in_pin.set_as_output();
        }

        loop {
            // self.wait_for_any_key().await;

            'active: loop {
                let mut sent = 0;
                for (out_idx, out_pin) in self.outs.iter_mut().enumerate() {
                    let Ok(()) = out_pin.set_high();
                    for in_pin in self.ins.iter_mut() {
                        in_pin.set_as_input();
                        in_pin.set_pull(Pull::Down);
                        in_pin.set_schmitt(true);
                    }

                    // TODO: Configurable
                    Timer::after_micros(1).await;

                    let row = self.ins.each_mut().map(|in_pin| {
                        let Ok(high) = in_pin.is_high();
                        high
                    });

                    // TODO: Debounce

                    trace!("Pins {}: {}", out_idx, row);

                    for (in_idx, high) in row.iter().cloned().enumerate() {
                        if self.key_states[out_idx][in_idx].high != high {
                            self.key_states[out_idx][in_idx].high = high;
                            event_channel
                                .send(KeyEvent {
                                    // TODO: Remapping
                                    key: self.mapping.map(out_idx, in_idx),
                                    pressed: high,
                                })
                                .await;
                            sent += 1;
                        }
                    }

                    let Ok(()) = out_pin.set_low();
                    for in_pin in self.ins.iter_mut() {
                        in_pin.set_as_output();
                        in_pin.set_pull(Pull::None);
                    }
                    Timer::after_micros(1).await;
                }
                trace!("States {}", self.key_states);

                // // Go idle
                // if sent == 0 {
                //     break 'active;
                // }

                // TODO: Configurable
                Timer::after_micros(1000).await;
            }
        }
    }

    async fn wait_for_any_key(&mut self) {
        // First, set all output pin to high
        for out in self.outs.iter_mut() {
            let Ok(()) = out.set_high();
        }

        Timer::after_micros(1).await;

        let futs: Pin<&mut [_; INS_SIZE]> =
            pin!(self.ins.each_mut().map(|in_pin| in_pin.wait_for_any_edge()));
        select_slice(futs).await.0;

        // Set all output pins back to low
        for out in self.outs.iter_mut() {
            let Ok(()) = out.set_low();
        }
    }
}
