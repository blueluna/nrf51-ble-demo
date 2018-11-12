#![no_main]
#![no_std]

extern crate panic_semihosting;

use core::cell::RefCell;
use core::ops::DerefMut;
use core::time::Duration;
use core::u32;

use byteorder::{ByteOrder, LittleEndian};

use nrf51_hal as hal;

use crate::hal::nrf51;
use crate::hal::nrf51::interrupt;
use crate::hal::prelude::*;
use crate::hal::gpio::{OpenDrain, Output};
use crate::hal::gpio::gpio::PIN7;

use nrf51_ble_demo as ble;

use crate::ble::ble::link::{LinkLayer, AddressKind, DeviceAddress};
use crate::ble::ble::link::ad_structure::{AdStructure, Flags};
use crate::ble::ble::link::MAX_PDU_SIZE;
use crate::ble::radio::{BleRadio, Baseband, PacketBuffer};

use cortex_m::interrupt::Mutex;
use cortex_m::peripheral::Peripherals;
use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;

struct BleHandler {
    baseband: Baseband,
    timer: nrf51::TIMER0,
    led: PIN7<Output<OpenDrain>>
}

static mut BLE_TX_BUF :PacketBuffer = [0; MAX_PDU_SIZE + 1];
static mut BLE_RX_BUF :PacketBuffer = [0; MAX_PDU_SIZE + 1];

static BLE: Mutex<RefCell<Option<BleHandler>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    if let Some(p) = nrf51::Peripherals::take() {
        // Configure high frequency clock to 16MHz
        p.CLOCK.xtalfreq.write(|w| w.xtalfreq()._16mhz());
        p.CLOCK.tasks_hfclkstart.write(|w| unsafe { w.bits(1) });
        while p.CLOCK.events_hfclkstarted.read().bits() == 0 {}
        // Configure low frequency clock to 32.768 kHz
        p.CLOCK.tasks_lfclkstart.write(|w| unsafe { w.bits(1) });
        while p.CLOCK.events_lfclkstarted.read().bits() == 0 {}
        p.CLOCK.events_lfclkstarted.write(|w| unsafe { w.bits(0) });
        // Read device address, BLE MAC
        let mut devaddr = [0u8; 6];
        let devaddr_lo = p.FICR.deviceaddr[0].read().bits();
        let devaddr_hi = p.FICR.deviceaddr[1].read().bits() as u16;
        LittleEndian::write_u32(&mut devaddr, devaddr_lo);
        LittleEndian::write_u16(&mut devaddr[4..], devaddr_hi);

        let devaddr_type = if p.FICR.deviceaddrtype.read().deviceaddrtype().is_public() {
            hprintln!("Public address").unwrap();
            AddressKind::Public
        } else {
            hprintln!("Random address").unwrap();
            AddressKind::Random
        };
        let device_address = DeviceAddress::new(devaddr, devaddr_type);

        let services = AdStructure::ServiceUuids16 {
            incomplete: false,
            uuids: &[0x181au16]
        };
        let mut ll = LinkLayer::new(device_address);
        ll.start_advertise(Duration::from_millis(200), &[
            AdStructure::Flags(Flags::discoverable()),
            AdStructure::CompleteLocalName("KIRE NOSSNEVS"),
            AdStructure::TxPowerLevel(4), // 4 dBm
            AdStructure::Appearance(1344), // Unofficial appearance Generic Sensor
            services,
        ]);

        // TIMER0 cfg, 32 bit @ 1 MHz
        // Mostly copied from the `nrf51-hal` crate.
        p.TIMER0.bitmode.write(|w| w.bitmode()._32bit());
        p.TIMER0.prescaler.write(|w| unsafe { w.prescaler().bits(4) });
        p.TIMER0.intenset.write(|w| w.compare0().set());
        p.TIMER0.shorts.write(|w| w
            .compare0_clear().enabled()
            .compare0_stop().enabled()
        );
        // Queue first baseband update
        cfg_timer(&p.TIMER0, Some(Duration::from_millis(10)));

        cortex_m::interrupt::free(move |cs| {
            let gpio = p.GPIO.split();
            let bb = Baseband::new(BleRadio::new(p.RADIO, &p.FICR,
                    unsafe { &mut BLE_TX_BUF }), unsafe { &mut BLE_RX_BUF }, ll);
            *BLE.borrow(cs).borrow_mut() = Some(BleHandler{
                baseband: bb,
                timer: p.TIMER0,
                led: gpio.pin7.into_open_drain_output()
            });
        });

        if let Some(mut p) = Peripherals::take() {
            p.NVIC.enable(nrf51::Interrupt::TIMER0);
            nrf51::NVIC::unpend(nrf51::Interrupt::TIMER0);
            p.NVIC.enable(nrf51::Interrupt::RADIO);
            nrf51::NVIC::unpend(nrf51::Interrupt::RADIO);
        }
    }

    loop {
        continue;
    }
}

interrupt!(RADIO, radio_event);
interrupt!(TIMER0, timer0_event);

fn radio_event() {
    cortex_m::interrupt::free(|cs| {
        if let Some(ble) = BLE.borrow(cs).borrow_mut().deref_mut() {
            if let Some(new_timeout) = ble.baseband.interrupt() {
                cfg_timer(&ble.timer, Some(new_timeout));
            }
            ble.led.set_low();
        }
    });
}

fn timer0_event() {
    cortex_m::interrupt::free(|cs| {
        if let Some(ble) = BLE.borrow(cs).borrow_mut().deref_mut() {
            let maybe_next_update = ble.baseband.update();
            cfg_timer(&ble.timer, maybe_next_update);
            ble.led.set_high();
        }
    });
}

/// Reconfigures TIMER0 to raise an interrupt after `duration` has elapsed.
///
/// TIMER0 is stopped if `duration` is `None`.
///
/// Note that if the timer has already queued an interrupt, the task will still be run after the
/// timer is stopped by this function.
fn cfg_timer(t: &nrf51::TIMER0, duration: Option<Duration>) {
    // Timer activation code is also copied from the `nrf51-hal` crate.
    if let Some(duration) = duration {
        assert!(duration.as_secs() < ((u32::MAX - duration.subsec_micros()) / 1_000_000) as u64);
        let us = (duration.as_secs() as u32) * 1_000_000 + duration.subsec_micros();
        t.cc[0].write(|w| unsafe { w.bits(us) });
        t.events_compare[0].reset();
        t.tasks_start.write(|w| unsafe { w.bits(1) });
    }
    else {
        t.events_compare[0].reset();
    }
}
