use cortex_m::peripheral::NVIC;
use nrf51_hal::gpio::{Disconnected, Level, Output, PushPull};
use nrf51_hal::gpio::p0::P0_20;
use crate::nrf51_hal::prelude::OutputPin;
use nrf51_pac::{GPIOTE, interrupt, Interrupt, PPI};
use crate::mutex::Mutex;
use crate::once_cell::OnceCell;

pub struct Motors {
    motor_values: [u16; 4],
    motor_max: u16,
    motor_min: u16,
    timer1: nrf51_pac::TIMER1,
    timer2: nrf51_pac::TIMER2,
    pin20: P0_20<Output<PushPull>>,
}

const MOTOR_0_PIN: u8 = 21;
const MOTOR_1_PIN: u8 = 23;
const MOTOR_2_PIN: u8 = 25;
const MOTOR_3_PIN: u8 = 29;

static MOTORS: Mutex<OnceCell<Motors>> = Mutex::new(OnceCell::uninitialized());

pub fn set_motor_limits(min: u16, max: u16) {
    let mut guard = MOTORS.lock();
    guard.motor_min = min;
    guard.motor_max = max;
}

pub fn get_motors() -> [u16; 4] {
    MOTORS.lock().motor_values
}

pub fn set_motors(val: [u16; 4]) {
    let mut guard = MOTORS.lock();
    guard.motor_values = val.map(|v| v.clamp(guard.motor_min, guard.motor_max));
}

pub(crate) fn initialize(
    timer1: nrf51_pac::TIMER1,
    timer2: nrf51_pac::TIMER2,
    nvic: &mut NVIC,
    ppi: &mut PPI,
    gpiote: &mut GPIOTE,
    pin20: P0_20<Disconnected>,
) {
    // Configure pin20
    let pin20 = pin20.into_push_pull_output(Level::Low);

    let mut motors = MOTORS.lock();
    motors.initialize(Motors {
        motor_values: [0; 4],
        motor_max: 0,
        motor_min: 800,
        timer1,
        timer2,
        pin20,
    });

    // Configure GPIOTE. GPIOTE is stands for GPIO tasks and events.
    // With it,
    gpiote.config[0].write(|w| unsafe {
        w.mode()
            .task()
            .psel()
            .bits(MOTOR_0_PIN)
            .polarity()
            .toggle()
            .outinit()
            .set_bit()
    });
    gpiote.config[1].write(|w| unsafe {
        w.mode()
            .task()
            .psel()
            .bits(MOTOR_1_PIN)
            .polarity()
            .toggle()
            .outinit()
            .set_bit()
    });
    gpiote.config[2].write(|w| unsafe {
        w.mode()
            .task()
            .psel()
            .bits(MOTOR_2_PIN)
            .polarity()
            .toggle()
            .outinit()
            .set_bit()
    });
    gpiote.config[3].write(|w| unsafe {
        w.mode()
            .task()
            .psel()
            .bits(MOTOR_3_PIN)
            .polarity()
            .toggle()
            .outinit()
            .set_bit()
    });

    // Configure timer 2
    motors.timer2.prescaler.write(|w| unsafe { w.prescaler().bits(1) }); //0.125us
    motors.timer2.intenset.write(|w| w.compare3().set_bit());
    motors.timer2.cc[0].write(|w| unsafe { w.bits(1000) });
    motors.timer2.cc[1].write(|w| unsafe { w.bits(1000) });
    motors.timer2.cc[3].write(|w| unsafe { w.bits(2500) });
    motors.timer2.shorts.write(|w| w.compare3_clear().set_bit());
    motors.timer2.tasks_clear.write(|w| unsafe { w.bits(1) });

    // Configure timer 1
    motors.timer1.prescaler.write(|w| unsafe { w.prescaler().bits(1) }); //0.125us
    motors.timer1.intenset.write(|w| w.compare3().set_bit());
    motors.timer1.cc[0].write(|w| unsafe { w.bits(1000) });
    motors.timer1.cc[1].write(|w| unsafe { w.bits(1000) });
    motors.timer1.cc[3].write(|w| unsafe { w.bits(2500) });
    motors.timer1.shorts.write(|w| w.compare3_clear().set_bit());
    motors.timer1.tasks_clear.write(|w| unsafe { w.bits(1) });

    motors.timer2.tasks_start.write(|w| unsafe { w.bits(1) });
    motors.timer1.tasks_start.write(|w| unsafe { w.bits(1) });

    // Link motor 0 - gpiote 0
    ppi.ch[0]
        .eep
        .write(|w| unsafe { w.bits(motors.timer1.events_compare[0].as_ptr() as u32) });
    ppi.ch[0]
        .tep
        .write(|w| unsafe { w.bits(gpiote.tasks_out[0].as_ptr() as u32) });
    ppi.ch[1]
        .eep
        .write(|w| unsafe { w.bits(motors.timer1.events_compare[3].as_ptr() as u32) });
    ppi.ch[1]
        .tep
        .write(|w| unsafe { w.bits(gpiote.tasks_out[0].as_ptr() as u32) });

    // Link motor 1 - gpiote 1
    ppi.ch[2]
        .eep
        .write(|w| unsafe { w.bits(motors.timer1.events_compare[1].as_ptr() as u32) });
    ppi.ch[2]
        .tep
        .write(|w| unsafe { w.bits(gpiote.tasks_out[1].as_ptr() as u32) });
    ppi.ch[3]
        .eep
        .write(|w| unsafe { w.bits(motors.timer1.events_compare[3].as_ptr() as u32) });
    ppi.ch[3]
        .tep
        .write(|w| unsafe { w.bits(gpiote.tasks_out[1].as_ptr() as u32) });

    // Link motor 2 - gpiote 2
    ppi.ch[4]
        .eep
        .write(|w| unsafe { w.bits(motors.timer2.events_compare[0].as_ptr() as u32) });
    ppi.ch[4]
        .tep
        .write(|w| unsafe { w.bits(gpiote.tasks_out[2].as_ptr() as u32) });
    ppi.ch[5]
        .eep
        .write(|w| unsafe { w.bits(motors.timer2.events_compare[3].as_ptr() as u32) });
    ppi.ch[5]
        .tep
        .write(|w| unsafe { w.bits(gpiote.tasks_out[2].as_ptr() as u32) });

    // Link motor 3 - gpiote 3
    ppi.ch[6]
        .eep
        .write(|w| unsafe { w.bits(motors.timer2.events_compare[1].as_ptr() as u32) });
    ppi.ch[6]
        .tep
        .write(|w| unsafe { w.bits(gpiote.tasks_out[3].as_ptr() as u32) });
    ppi.ch[7]
        .eep
        .write(|w| unsafe { w.bits(motors.timer2.events_compare[3].as_ptr() as u32) });
    ppi.ch[7]
        .tep
        .write(|w| unsafe { w.bits(gpiote.tasks_out[3].as_ptr() as u32) });

    ppi.chenset.write(|w| {
        w.ch0()
            .set_bit()
            .ch1()
            .set_bit()
            .ch2()
            .set_bit()
            .ch3()
            .set_bit()
            .ch4()
            .set_bit()
            .ch5()
            .set_bit()
            .ch6()
            .set_bit()
            .ch7()
            .set_bit()
    });

    // turn on timer 2 interrupts
    NVIC::unpend(Interrupt::TIMER2);
    // set their priority to 1, very high
    unsafe {
        nvic.set_priority(Interrupt::TIMER2, 1);
    }

    // turn on timer 2 interrupts
    NVIC::unpend(Interrupt::TIMER1);
    // set their priority to 1, very high
    unsafe {
        nvic.set_priority(Interrupt::TIMER1, 1);
    }
}



#[interrupt]
unsafe fn TIMER2() {
    // Safety: interrupts are already turned off here, since we are inside an interrupt
    let motors = unsafe {MOTORS.no_critical_section_lock()};
    if motors.timer2.events_compare[3].read().bits() != 0 {
        motors.timer2.events_compare[3].reset();
        //2500 * 0.125
        motors.timer2.tasks_capture[2].write(|w| w.bits(1));

        //TODO is this ever false?
        if motors.timer2.cc[2].read().bits() < 500 {
            motors.timer2.cc[0].write(|w| w.bits((1000 + motors.motor_values[2]) as u32));
            motors.timer2.cc[1].write(|w| w.bits((1000 + motors.motor_values[3]) as u32));
        }
    }
}

#[interrupt]
unsafe fn TIMER1() {
    // Safety: interrupts are already turned off here, since we are inside an interrupt
    let motors = unsafe {MOTORS.no_critical_section_lock()};
    if motors.timer1.events_compare[3].read().bits() != 0 {
        motors.timer1.events_compare[3].reset();
        motors.timer1.tasks_capture[2].write(|w| w.bits(1));

        //TODO is this ever false?
        if motors.timer1.cc[2].read().bits() < 500 {
            motors.pin20.set_high().unwrap();
            motors.timer1.cc[0].write(|w| w.bits((1000 + motors.motor_values[0]) as u32));
            motors.timer1.cc[1].write(|w| w.bits((1000 + motors.motor_values[1]) as u32));
            motors.pin20.set_low().unwrap();
        }
    }
}