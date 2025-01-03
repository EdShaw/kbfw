use core::{
    convert::Infallible,
    pin::{pin, Pin},
    u16,
};

use defmt::Format;
use embassy_futures::select::select_slice;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::Timer;
use embedded_hal_1::digital::{InputPin, OutputPin};
use embedded_hal_async::digital::Wait;

#[derive(Copy, Clone, Debug, Format, Default)]
pub struct KeyState {
    pub(crate) pressed: bool,
}

#[derive(Copy, Clone, Debug, Format)]
pub struct KeyEvent {
    pub key: u16,
    pub pressed: bool,
}

type KeyChannel<'a> = embassy_sync::channel::Sender<'a, CriticalSectionRawMutex, KeyEvent, 32>;

pub struct Matrix<In, Out, const INS_SIZE: usize, const OUTS_SIZE: usize>
where
    In: InputPin<Error = Infallible> + Wait<Error = Infallible>,
    Out: OutputPin<Error = Infallible>,
{
    ins: [In; INS_SIZE],
    outs: [Out; OUTS_SIZE],

    key_states: [[KeyState; INS_SIZE]; OUTS_SIZE],
}

impl<In, Out, const INS_SIZE: usize, const OUTS_SIZE: usize> Matrix<In, Out, INS_SIZE, OUTS_SIZE>
where
    In: InputPin<Error = Infallible> + Wait<Error = Infallible>,
    Out: OutputPin<Error = Infallible>,
{
    pub fn new(ins: [In; INS_SIZE], outs: [Out; OUTS_SIZE]) -> Self {
        const {
            assert!(INS_SIZE <= u8::MAX as usize);
            assert!(OUTS_SIZE <= u8::MAX as usize);
        };

        Self {
            ins,
            outs,
            key_states: [[KeyState::default(); INS_SIZE]; OUTS_SIZE],
        }
    }

    pub async fn scan(&mut self, event_channel: KeyChannel<'_>) -> ! {
        for out_pin in self.outs.iter_mut() {
            let Ok(()) = out_pin.set_low();
        }

        loop {
            self.wait_for_any_key().await;

            'active: loop {
                let mut sent = 0;
                for (out_idx, out_pin) in self.outs.iter_mut().enumerate() {
                    let Ok(()) = out_pin.set_high();

                    // TODO: Configurable
                    Timer::after_micros(1).await;

                    let row = self.ins.each_mut().map(|in_pin| {
                        let Ok(high) = in_pin.is_high();
                        high
                    });

                    // TODO: Debounce

                    for (in_idx, high) in row.iter().cloned().enumerate() {
                        event_channel
                            .send(KeyEvent {
                                // TODO: Remapping
                                key: Self::mapping(out_idx, in_idx),
                                pressed: high,
                            })
                            .await;
                        sent += 1;
                    }

                    let Ok(()) = out_pin.set_low();
                }

                // Go idle
                if sent == 0 {
                    break 'active;
                }

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
        let Ok(()) = select_slice(futs).await.0;

        // Set all output pins back to low
        for out in self.outs.iter_mut() {
            let Ok(()) = out.set_low();
        }
    }

    fn mapping(out_idx: usize, in_idx: usize) -> u16 {
        // Each Matrix should have it's own mapping, but w/e
        // Making this a associated type would be cool
        return (out_idx * INS_SIZE + in_idx) as u16;
    }
}
