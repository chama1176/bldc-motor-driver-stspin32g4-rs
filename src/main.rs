#![no_std]
#![no_main]

// pick a panicking behavior
use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
// use panic_abort as _; // requires nightly
// use panic_itm as _; // logs messages over ITM; requires ITM support
// use panic_semihosting as _; // logs messages to the host stderr; requires a debugger

use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;

mod g4test {
    use stm32g4::stm32g431;

    pub trait Led {
        fn on(&self);
        fn off(&self);
        fn toggle(&self);
    }    
    
    pub struct Led0 {
        perip: stm32g431::Peripherals,
    }
    
    impl Led for Led0 {
        fn on(&self) {
            let gpioc = &self.perip.GPIOC;
            gpioc.bsrr.write(|w| w.bs13().set());
        }
        fn off(&self) {
            let gpioc = &self.perip.GPIOC;
            gpioc.bsrr.write(|w| w.br13().reset());
        }
        fn toggle(&self) {
            
        }
    }
    impl Led0 {
        pub fn new() -> Self {
            // stm32f401モジュールより、ペリフェラルの入り口となるオブジェクトを取得する。
            Self{perip: stm32g431::Peripherals::take().unwrap()}
        }
        pub fn init(&self) {
            // GPIOCポートの電源投入(クロックの有効化)
            self.perip.RCC.ahb2enr.modify(|_,w| w.gpiocen().set_bit());

            // gpio初期化(PC13を出力に指定)
            let gpioc = &self.perip.GPIOC;
            gpioc.moder.modify(|_,w| w.moder13().output());
        }
    }
    
}



#[entry]
fn main() -> ! {
    use g4test::Led;

    // hprintln!("Hello, STM32G4!").unwrap();
    let led0 = g4test::Led0::new();
    led0.init();
    loop {
        loop {
            // hprintln!("Set Led High").unwrap();
            for _ in 0..50_000 {
                led0.on();
            }
            // hprintln!("Set Led Low").unwrap();
            for _ in 0..50_000 {
                led0.off();
            }
        }
    }
}