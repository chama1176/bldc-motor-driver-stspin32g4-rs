use core::cell::RefCell;
use core::fmt::{self, Write};

use cortex_m::interrupt::{free, Mutex};
use cortex_m::register;

#[allow(unused_imports)]
use cortex_m_semihosting::hprintln;
use stm32g4::stm32g431::CorePeripherals;
use stm32g4::stm32g431::Interrupt;
use stm32g4::stm32g431::Peripherals;
use stm32g4::stm32g431::FLASH;
use stm32g4::stm32g431::NVIC;

use crate::indicator::Indicator;
use motml::encoder::Encoder;
use motml::motor_driver::OutputStatus;
use motml::motor_driver::ThreePhaseCurrent;
use motml::motor_driver::ThreePhaseMotorDriver;
use motml::motor_driver::ThreePhaseValue;
use motml::motor_driver::ThreePhaseVoltage;
use motml::utils::Deg;



#[repr(C)]
union UnionFlashMemory {
    f1: [u32; 7],
    f2: f32,
}

pub fn clock_init(perip: &Peripherals, core_perip: &mut CorePeripherals) {
    perip.RCC.cr.modify(|_, w| w.hsebyp().bypassed());
    perip.RCC.cr.modify(|_, w| w.hseon().on());
    while perip.RCC.cr.read().hserdy().is_not_ready() {}

    // Disable the PLL
    perip.RCC.cr.modify(|_, w| w.pllon().off());
    // Wait until PLL is fully stopped
    while perip.RCC.cr.read().pllrdy().is_ready() {}
    // 48 / 12 x 70 / 2 = 140
    perip.RCC.pllcfgr.modify(|_, w| w.pllsrc().hse()); // 48MHz
    perip.RCC.pllcfgr.modify(|_, w| w.pllm().div12()); // /12
                                                       // perip.RCC.pllcfgr.modify(|_, w| w.plln().div85());
    perip.RCC.pllcfgr.modify(|_, w| w.plln().div70()); // x70
    perip.RCC.pllcfgr.modify(|_, w| w.pllr().div2()); // /2

    perip.RCC.cr.modify(|_, w| w.pllon().on());
    while perip.RCC.cr.read().pllrdy().is_not_ready() {}
    perip.RCC.pllcfgr.modify(|_, w| w.pllren().set_bit());

    perip
        .FLASH
        .acr
        .modify(|_, w| unsafe { w.latency().bits(4) });
    while perip.FLASH.acr.read().latency().bits() != 4 {
        // hprintln!("latency bit: {}", perip.FLASH.acr.read().latency().bits()).unwrap();
    }

    perip.RCC.cfgr.modify(|_, w| w.sw().pll());
    // perip.RCC.cfgr.modify(|_, w| w.sw().hse());
    // hprintln!("sw bit: {}", perip.RCC.cfgr.read().sw().bits()).unwrap();
    while !perip.RCC.cfgr.read().sw().is_pll() {}
    while !perip.RCC.cfgr.read().sws().is_pll() {
        // hprintln!("sw bit: {}", perip.RCC.cfgr.read().sw().bits()).unwrap();
        // hprintln!("sws bit: {}", perip.RCC.cfgr.read().sws().bits()).unwrap();
    }
    // while !perip.RCC.cfgr.read().sws().is_hse() {}

    perip.RCC.apb1enr1.modify(|_, w| w.tim3en().enabled());
    perip.RCC.apb1enr1.modify(|_, w| w.tim6en().enabled());

    // For main task
    let tim3 = &perip.TIM3;
    // tim3.psc.modify(|_, w| unsafe { w.bits(170 - 1) });
    tim3.psc.modify(|_, w| unsafe { w.bits(14 - 1) });
    tim3.arr.modify(|_, w| unsafe { w.bits(10_000 - 1) }); // 1kHz
    tim3.dier.modify(|_, w| w.uie().set_bit());
    tim3.cr1.modify(|_, w| w.cen().set_bit());

    unsafe {
        core_perip.NVIC.set_priority(Interrupt::TIM3, 0);
        NVIC::unmask(Interrupt::TIM3);
    }

    // For ADC
    let tim6 = &perip.TIM6;
    tim6.psc.modify(|_, w| unsafe { w.bits(14_000 - 1) });
    tim6.arr.modify(|_, w| unsafe { w.bits(10 - 1) }); // 1kHz
    tim6.dier.modify(|_, w| w.uie().set_bit());
    tim6.cr2.modify(|_, w| unsafe { w.mms().bits(0b010) });
}

pub fn dma_init(perip: &Peripherals, core_perip: &mut CorePeripherals, address: u32) {
    // DMAの電源投入(クロックの有効化)
    // perip.RCC.ahb1rstr.modify(|_, w| w.dmamux1rst().reset());
    // perip.RCC.ahb1rstr.modify(|_, w| w.dma1rst().reset());
    perip.RCC.ahb1rstr.modify(|_, w| w.dmamux1rst().set_bit());
    perip.RCC.ahb1rstr.modify(|_, w| w.dma1rst().set_bit());
    perip.RCC.ahb1rstr.modify(|_, w| w.dmamux1rst().clear_bit());
    perip.RCC.ahb1rstr.modify(|_, w| w.dma1rst().clear_bit());
    perip.RCC.ahb1enr.modify(|_, w| w.dmamuxen().set_bit());
    perip.RCC.ahb1enr.modify(|_, w| w.dma1en().set_bit());

    perip.DMA1.ccr1.modify(|_, w| unsafe { w.pl().bits(0b10) }); // priority level 2
    perip
        .DMA1
        .ccr1
        .modify(|_, w| unsafe { w.msize().bits(0b01) }); // 16bit
    perip
        .DMA1
        .ccr1
        .modify(|_, w| unsafe { w.psize().bits(0b01) }); // 16bit
    perip.DMA1.ccr1.modify(|_, w| w.circ().set_bit()); // circular mode
    perip.DMA1.ccr1.modify(|_, w| w.minc().set_bit()); // increment memory ptr
    perip.DMA1.ccr1.modify(|_, w| w.pinc().clear_bit()); // not increment periph  ptr
    perip.DMA1.ccr1.modify(|_, w| w.mem2mem().clear_bit()); // memory-to-memory mode
    perip.DMA1.ccr1.modify(|_, w| w.dir().clear_bit()); // read from peripheral
    perip.DMA1.ccr1.modify(|_, w| w.teie().clear_bit()); // transfer error interrupt enable
    perip.DMA1.ccr1.modify(|_, w| w.htie().clear_bit()); // half transfer interrupt enable
    perip.DMA1.ccr1.modify(|_, w| w.tcie().clear_bit()); // transfer complete interrupt enable

    // For category 2 devices:
    // • DMAMUX channels 0 to 5 are connected to DMA1 channels 1 to 6
    // • DMAMUX channels 6 to 11 are connected to DMA1 channels 1 to 6
    // DMA1 ch1 -> DMAMUX ch6
    perip
        .DMAMUX
        .c0cr
        .modify(|_, w| unsafe { w.dmareq_id().bits(36) }); // Table.91 36:ADC2
    perip.DMAMUX.c0cr.modify(|_, w| w.ege().set_bit()); // Enable generate event

    let adc = &perip.ADC2;
    let adc_data_register_addr = &adc.dr as *const _ as u32;
    // let adc_dma_buf_addr : u32 = adc_dma_buf as *const [u16; 4] as u32;
    // perip.DMA1.cpar1.modify(|_, w| unsafe { w.pa().bits(*adc.dr.as_ptr()) });   // peripheral address
    perip
        .DMA1
        .cpar1
        .modify(|_, w| unsafe { w.pa().bits(adc_data_register_addr) }); // peripheral address
                                                                        // perip.DMA1.cndtr1.modify(|_, w| unsafe { w.ndt().bits(adc_dma_buf.len() as u16) }); // num
    perip.DMA1.cndtr1.modify(|_, w| unsafe { w.ndt().bits(7) }); // num
                                                                 // perip.DMA1.cmar1.modify(|_, w| unsafe { w.ma().bits(adc_dma_buf_addr) });      // memory address
    perip
        .DMA1
        .cmar1
        .modify(|_, w| unsafe { w.ma().bits(address) }); // memory address

    // 割り込み設定
    // unsafe{
    //     core_perip.NVIC.set_priority(Interrupt::DMA1_CH1, 0);
    //     NVIC::unmask(Interrupt::DMA1_CH1);
    //     core_perip.NVIC.set_priority(Interrupt::ADC1_2, 0);
    //     NVIC::unmask(Interrupt::ADC1_2);
    // }
}

pub fn adc2_init(perip: &Peripherals) {
    // GPIOポートの電源投入(クロックの有効化)
    perip.RCC.ahb2enr.modify(|_, w| w.gpioaen().set_bit());
    perip.RCC.ahb2enr.modify(|_, w| w.gpiocen().set_bit());

    perip.RCC.ahb2enr.modify(|_, w| w.adc12en().set_bit());
    perip.RCC.ccipr.modify(|_, w| w.adc12sel().system()); // clock source setting

    // gpioモード変更
    perip.GPIOA.moder.modify(|_, w| w.moder0().analog());
    perip.GPIOA.moder.modify(|_, w| w.moder6().analog());
    perip.GPIOA.moder.modify(|_, w| w.moder7().analog());

    perip.GPIOC.moder.modify(|_, w| w.moder0().analog());
    perip.GPIOC.moder.modify(|_, w| w.moder1().analog());
    perip.GPIOC.moder.modify(|_, w| w.moder2().analog());
    perip.GPIOC.moder.modify(|_, w| w.moder3().analog());

    let adc = &perip.ADC2;
    adc.cfgr.modify(|_, w| w.res().bits12()); // Resolution setting
    adc.cfgr.modify(|_, w| w.align().right()); // Data align setting
    adc.cfgr.modify(|_, w| w.ovrmod().overwrite()); // Overrun mode

    adc.cfgr.modify(|_, w| w.cont().single()); // single or continuous
                                               // adc.cfgr.modify(|_, w| w.cont().continuous());   // single or continuous
    adc.cfgr.modify(|_, w| w.discen().disabled()); // single or continuous
                                                   // adc.cfgr.modify(|_, w| w.discen().enabled());   // single or continuous
                                                   // DISCEN = 1 and CONT = 1 is not allowed.
                                                   // adc.cfgr.modify(|_, w| w.discnum().bits(4-1));   // 0 means 1 length

    adc.cfgr.modify(|_, w| w.dmacfg().circular()); // dma oneshot or circular
    adc.cfgr.modify(|_, w| w.dmaen().enabled()); // dma enable
                                                 // 1周は実行したいが，常に変換しつづけるのは困る
    adc.cfgr.modify(|_, w| w.extsel().tim6_trgo()); // dma enable
    adc.cfgr.modify(|_, w| w.exten().rising_edge()); // dma enable
    adc.cfgr2.modify(|_, w| w.rovse().disabled()); // over sampling enable

    perip
        .ADC12_COMMON
        .ccr
        .modify(|_, w| unsafe { w.presc().bits(0b0010) }); // Clock prescaler setting

    adc.cr.modify(|_, w| w.deeppwd().disabled()); // Deep power down setting
    adc.cr.modify(|_, w| w.advregen().enabled()); // Voltage regulator setting
                                                  // adc.ier.modify(|_, w| w.eocie().enabled());   // End of regular conversion interrupt setting
    adc.ier.modify(|_, w| w.eocie().disabled()); // End of regular conversion interrupt setting
    adc.ier.modify(|_, w| w.ovrie().enabled()); // Overrun interrupt setting
                                                // // ADC voltage regulator start-up time 20us
    let mut t = perip.TIM3.cnt.read().cnt().bits();
    let prev = t;
    while t.wrapping_sub(prev) >= 10 {
        t = perip.TIM3.cnt.read().cnt().bits();
    }
    // P.604 21.4.8 calibration
    assert!(adc.cr.read().aden().is_enable() == false);
    adc.cr.modify(|_, w| w.adcal().calibration()); // Start calibration
    while !adc.cr.read().adcal().is_complete() {} // Wait for calibration complete

    // 1: Current W
    // 3: 1.5V Ref
    // 4: Current V
    // 6: Temp Coil
    // 7: AD0
    // 8: Temp FET
    // 9: Battery Voltage

    adc.smpr1.modify(|_, w| w.smp1().cycles24_5()); // sampling time selection
    adc.smpr1.modify(|_, w| w.smp3().cycles24_5()); // sampling time selection
    adc.smpr1.modify(|_, w| w.smp4().cycles24_5()); // sampling time selection
    adc.smpr1.modify(|_, w| w.smp6().cycles24_5()); // sampling time selection
    adc.smpr1.modify(|_, w| w.smp7().cycles24_5()); // sampling time selection
    adc.smpr1.modify(|_, w| w.smp8().cycles24_5()); // sampling time selection
    adc.smpr1.modify(|_, w| w.smp9().cycles24_5()); // sampling time selection

    adc.sqr1.modify(|_, w| w.l().bits(7 - 1)); // Regular channel sequence length. 0 means 1 length
    adc.sqr1.modify(|_, w| unsafe { w.sq1().bits(1) }); // 1st conversion in regular sequence
    adc.sqr1.modify(|_, w| unsafe { w.sq2().bits(3) }); // 1st conversion in regular sequence
    adc.sqr1.modify(|_, w| unsafe { w.sq3().bits(4) }); // 1st conversion in regular sequence
    adc.sqr1.modify(|_, w| unsafe { w.sq4().bits(6) }); // 1st conversion in regular sequence
    adc.sqr2.modify(|_, w| unsafe { w.sq5().bits(7) }); // 1st conversion in regular sequence
    adc.sqr2.modify(|_, w| unsafe { w.sq6().bits(8) }); // 1st conversion in regular sequence
    adc.sqr2.modify(|_, w| unsafe { w.sq7().bits(9) }); // 1st conversion in regular sequence
}

pub fn dma_adc2_start(perip: &Peripherals) {
    // enable DMA
    perip.DMA1.ccr1.modify(|_, w| w.en().set_bit());

    let adc = &perip.ADC2;
    // enable ADC
    adc.isr.modify(|_, w| w.adrdy().set_bit());
    adc.cr.modify(|_, w| w.aden().enable()); // ADC enable control
    while adc.isr.read().adrdy().is_not_ready() {
        // Wait for ADC ready
    }
    let tim6 = &perip.TIM6;
    tim6.cr1.modify(|_, w| w.cen().set_bit());

    // Start ADC
    adc.cr.modify(|_, w| w.adstart().start()); // ADC start
}

pub static G_PERIPHERAL: Mutex<RefCell<Option<stm32g4::stm32g431::Peripherals>>> =
    Mutex::new(RefCell::new(None));

pub fn init_g_peripheral(perip: Peripherals) {
    free(|cs| G_PERIPHERAL.borrow(cs).replace(Some(perip)));
}

pub struct FrashStorage {}
impl<'a> FrashStorage {
    pub fn new() -> Self {
        Self {}
    }
    pub fn write(&self) {
        free(|cs| match G_PERIPHERAL.borrow(cs).borrow().as_ref() {
            None => (),
            Some(perip) => {
                let flash = &perip.FLASH;
                // hprintln!("flash cr lock: {}", flash.cr.read().lock().bit_is_set()).unwrap();
                self.unlock_flash(&flash);
                // hprintln!("flash cr lock: {}", flash.cr.read().lock().bit_is_set()).unwrap();

                // Page erase
                // Check BSY in FLASH_SR is not set.
                while flash.sr.read().bsy().bit_is_set() {}
                // Check and clear all error programming flags due to a previous programming. If not, PGSERR is set
                self.check_and_clear_all_error(&flash);
                // In single bank mode (DBANK option bit is reset),
                // set the PER bit and select the page to erase (PNB).
                // The associated bank (BKER) in the Flash control register (FLASH_CR) must be kept cleared.
                // Set the STRT bit in the FLASH_CR register.
                // 一行で書かないとロックされてしまう
                flash.cr.write(|w| unsafe {w.pnb().bits(15).per().set_bit().strt().set_bit()});
                // hprintln!("per bit: {}", flash.cr.read().per().bits()).unwrap();
                // hprintln!("pnb bit: {}", flash.cr.read().pnb().bits()).unwrap();
                // hprintln!("strt bit: {}", flash.cr.read().strt().bits()).unwrap();
                //  Wait for the BSY bit to be cleared in the FLASH_SR register.
                while flash.sr.read().bsy().bit_is_set() {}
                // hprintln!("bsy bit: {}", flash.sr.read().bsy().bits()).unwrap();

                // FLASH programming
                // Check BSY in FLASH_SR is not set.
                while flash.sr.read().bsy().bit_is_set() {}
                // Check and clear all error programming flags due to a previous programming. If not, PGSERR is set
                self.check_and_clear_all_error(&flash);
                // Set the PG bit in the Flash control register (FLASH_CR)
                // EOPはEOPIEがセットされているときのみ更新される
                self.unlock_flash(&flash);
                flash.cr.write(|w| w.pg().set_bit().eopie().set_bit());
                hprintln!("pg bit: {}", flash.cr.read().pg().bits()).unwrap();
                // write double word 2 x 32bit
                // write first word. -> write second word
                let address = 0x0800_7800usize;
                let r1 = address as *mut u32;
                let r2 = (address+0x4) as *mut u32;
                unsafe {
                    *r1 = 0xCB;
                    *r2 = 0xDA;
                }
                // Wait for the BSY bit to be cleared in the FLASH_SR register.
                while flash.sr.read().bsy().bit_is_set() {}
                // Check that EOP flag is set in the FLASH_SR register (meaning that the programming operation has succeed), and clear it by software.
                while flash.sr.read().eop().bit_is_clear() {}
                // Writing 1 to clear.
                flash.sr.write(|w| w.eop().set_bit());
                

                // Clear the PG bit in the FLASH_CR register if there no more programming request anymore.
                self.unlock_flash(&flash);
                flash.cr.write(|w| w.pg().clear_bit());
                hprintln!("pg bit: {}", flash.cr.read().pg().bits()).unwrap();
            }
        });
    }
    fn check_and_clear_all_error(&self, flash: &FLASH) {
        if flash.sr.read().optverr().bit_is_set() {
            hprintln!("optverr error occured: {}", flash.sr.read().optverr().bits()).unwrap();
            flash.sr.write(|w| w.optverr().set_bit())
        }
        if flash.sr.read().rderr().bit_is_set() {
            hprintln!("rderr error occured: {}", flash.sr.read().rderr().bits()).unwrap();
            flash.sr.write(|w| w.rderr().set_bit())
        }
        if flash.sr.read().fasterr().bit_is_set() {
            hprintln!("fasterr error occured: {}", flash.sr.read().fasterr().bits()).unwrap();
            flash.sr.write(|w| w.fasterr().set_bit())
        }
        if flash.sr.read().miserr().bit_is_set() {
            hprintln!("miserr error occured: {}", flash.sr.read().miserr().bits()).unwrap();
            flash.sr.write(|w| w.miserr().set_bit())
        }
        if flash.sr.read().pgserr().bit_is_set() {
            hprintln!("pgserr error occured: {}", flash.sr.read().pgserr().bits()).unwrap();
            flash.sr.write(|w| w.pgserr().set_bit())
        }
        if flash.sr.read().sizerr().bit_is_set() {
            hprintln!("sizerr error occured: {}", flash.sr.read().sizerr().bits()).unwrap();
            flash.sr.write(|w| w.sizerr().set_bit())
        }
        if flash.sr.read().pgaerr().bit_is_set() {
            hprintln!("pgaerr error occured: {}", flash.sr.read().pgaerr().bits()).unwrap();
            flash.sr.write(|w| w.pgaerr().set_bit())
        }
        if flash.sr.read().wrperr().bit_is_set() {
            hprintln!("wrperr error occured: {}", flash.sr.read().wrperr().bits()).unwrap();
            flash.sr.write(|w| w.wrperr().set_bit())
        }
        if flash.sr.read().progerr().bit_is_set() {
            hprintln!("progerr error occured: {}", flash.sr.read().progerr().bits()).unwrap();
            flash.sr.write(|w| w.progerr().set_bit())
        }
        if flash.sr.read().operr().bit_is_set() {
            hprintln!("operr error occured: {}", flash.sr.read().operr().bits()).unwrap();
            flash.sr.write(|w| w.operr().set_bit())
        }
        if flash.sr.read().eop().bit_is_set() {
            hprintln!("eop error occured: {}", flash.sr.read().eop().bits()).unwrap();
            flash.sr.write(|w| w.eop().set_bit())
        }
    }
    fn unlock_flash(&self, flash: &FLASH) {
        // Unlocking the Flash memory
        // Check BSY in FLASH_SR is not set.
        while flash.sr.read().bsy().bit_is_set() {}
        // 1. Write KEY1 = 0x45670123 in the Flash key register (FLASH_KEYR)
        flash.keyr.write(|w| unsafe { w.bits(0x45670123) });
        // 2. Write KEY2 = 0xCDEF89AB in the FLASH_KEYR register.
        flash.keyr.write(|w| unsafe { w.bits(0xCDEF89AB) });
    }
}

pub struct Uart1 {}
impl<'a> Write for Uart1 {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        for c in s.bytes() {
            self.putc(c);
        }
        Ok(())
    }
}
impl<'a> Uart1 {
    pub fn new() -> Self {
        Self {}
    }
    pub fn init(&self) {
        free(|cs| match G_PERIPHERAL.borrow(cs).borrow().as_ref() {
            None => (),
            Some(perip) => {
                // GPIOポートの電源投入(クロックの有効化)
                perip.RCC.ahb2enr.modify(|_, w| w.gpiocen().set_bit());

                perip.RCC.apb2enr.modify(|_, w| w.usart1en().enabled());

                // gpioモード変更
                let gpioc = &perip.GPIOC;
                gpioc.moder.modify(|_, w| w.moder4().alternate());
                gpioc.moder.modify(|_, w| w.moder5().alternate());
                gpioc.afrl.modify(|_, w| w.afrl4().af7());
                gpioc.afrl.modify(|_, w| w.afrl5().af7());

                let uart = &perip.USART1;
                // Set over sampling mode
                uart.cr1.modify(|_, w| w.over8().clear_bit());
                // Set parity mode
                uart.cr1.modify(|_, w| w.pce().clear_bit());
                // Set word length
                uart.cr1.modify(|_, w| w.m0().clear_bit());
                uart.cr1.modify(|_, w| w.m1().clear_bit());
                // FIFO?
                // Set baud rate
                uart.brr.modify(|_, w| unsafe { w.bits(0x4BF) }); // 140MHz / 115200

                // Set stop bit
                uart.cr2.modify(|_, w| unsafe { w.stop().bits(0b00) });

                // Set uart enable
                uart.cr1.modify(|_, w| w.ue().set_bit());

                // Set uart recieve enable
                uart.cr1.modify(|_, w| w.re().set_bit());
                // Set uart transmitter enable
                uart.cr1.modify(|_, w| w.te().set_bit());
            }
        });
    }
    fn putc(&self, c: u8) {
        free(|cs| match G_PERIPHERAL.borrow(cs).borrow().as_ref() {
            None => (),
            Some(perip) => {
                let uart = &perip.USART1;
                uart.tdr.modify(|_, w| unsafe { w.tdr().bits(c.into()) });
                // while uart.isr.read().tc().bit_is_set() {}
                while uart.isr.read().txe().bit_is_clear() {}
            }
        });
    }
}

pub struct Spi3 {}
impl Spi3 {
    pub fn new() -> Self {
        Self {}
    }
    pub fn init(&self) {
        free(|cs| match G_PERIPHERAL.borrow(cs).borrow().as_ref() {
            None => (),
            Some(perip) => {
                // GPIOポートの電源投入(クロックの有効化)
                perip.RCC.ahb2enr.modify(|_, w| w.gpioben().set_bit());
                perip.RCC.apb1enr1.modify(|_, w| w.spi3en().enabled());

                // gpioモード変更
                let gpiob = &perip.GPIOB;
                gpiob.moder.modify(|_, w| w.moder6().output()); // CS pin
                gpiob.moder.modify(|_, w| w.moder5().alternate());
                gpiob.moder.modify(|_, w| w.moder4().alternate());
                gpiob.moder.modify(|_, w| w.moder3().alternate());
                gpiob.afrl.modify(|_, w| w.afrl5().af6());
                gpiob.afrl.modify(|_, w| w.afrl4().af6());
                gpiob.afrl.modify(|_, w| w.afrl3().af6());
                gpiob.ospeedr.modify(|_, w| w.ospeedr6().very_high_speed()); // CS pin
                gpiob.ospeedr.modify(|_, w| w.ospeedr5().very_high_speed());
                gpiob.ospeedr.modify(|_, w| w.ospeedr4().very_high_speed());
                gpiob.ospeedr.modify(|_, w| w.ospeedr3().very_high_speed());
                gpiob.otyper.modify(|_, w| w.ot6().push_pull()); // CS pin

                let spi = &perip.SPI3;
                spi.cr1.modify(|_, w| w.spe().clear_bit());

                // Set Baudrate
                spi.cr1.modify(|_, w| unsafe { w.br().bits(0b0111) }); // f_pclk / 256

                // Set Clock polarity
                spi.cr1.modify(|_, w| w.cpol().clear_bit()); // idle low

                // Set Clock phase
                spi.cr1.modify(|_, w| w.cpha().set_bit()); // second edge(down edge in-case idle is low)

                // Bidirectional data mode enable(half-duplex communication)
                spi.cr1.modify(|_, w| w.bidimode().clear_bit());
                // Set MSL LSB first
                spi.cr1.modify(|_, w| w.lsbfirst().clear_bit());
                // Set NSS management
                // Soft ware slave management
                spi.cr1.modify(|_, w| w.ssm().set_bit());
                // Internal slave select
                spi.cr1.modify(|_, w| w.ssi().set_bit());
                // Master configuration
                spi.cr1.modify(|_, w| w.mstr().set_bit());

                // Data size
                spi.cr2.modify(|_, w| unsafe { w.ds().bits(0b1111) }); // 16bit

                // SS output
                spi.cr2.modify(|_, w| w.ssoe().clear_bit());
                // Frame format
                spi.cr2.modify(|_, w| w.frf().clear_bit()); // Motorola mode

                // NSS pulse management
                spi.cr2.modify(|_, w| w.nssp().set_bit());
                //
                spi.cr1.modify(|_, w| w.spe().set_bit());
            }
        });
    }
    pub fn txrx(&self, c: u16) -> Option<u16> {
        free(|cs| match G_PERIPHERAL.borrow(cs).borrow().as_ref() {
            None => None,
            Some(perip) => {
                let gpiob = &perip.GPIOB;
                gpiob.bsrr.write(|w| w.br6().reset());
                let spi = &perip.SPI3;

                while spi.sr.read().txe().bit_is_clear() {}
                // send 8bit data automatically 2 times
                spi.dr.modify(|_, w| unsafe { w.dr().bits(c.into()) });

                while spi.sr.read().bsy().bit_is_set() {}
                while spi.sr.read().rxne().bit_is_clear() {}
                gpiob.bsrr.write(|w| w.bs6().set());

                let data = spi.dr.read().dr().bits();
                // hprintln!("dr: {:x}", data).unwrap();
                Some(data & 0x3FFF)
            }
        })
    }
}
impl Encoder<f32> for Spi3 {
    fn get_angle(&self) -> Option<f32> {
        let data: u16 = 0x3FFF | 0b0100_0000_0000_0000;
        let p: u16 = data.count_ones() as u16 % 2; // parity
        self.txrx(data | (p << 15));
        match self.txrx(data | (p << 15)) {
            None => None,
            Some(data) => {
                let deg = data as f32 / 16384.0 * 360.0;
                return Some(deg.invert_360().deg2rad());
            }
        }
    }
    fn reset_error(&self) {
        // clear error
        let data: u16 = 0x0001 | 0b0100_0000_0000_0000;
        let p: u16 = data.count_ones() as u16 % 2;
        self.txrx(data | (p << 15));
    }
}

pub struct BldcPwm {}
impl<'a> BldcPwm {
    pub fn new() -> Self {
        Self {}
    }
    pub fn init(&self) {
        free(|cs| match G_PERIPHERAL.borrow(cs).borrow().as_ref() {
            None => (),
            Some(perip) => {
                // GPIOポートの電源投入(クロックの有効化)
                perip.RCC.ahb2enr.modify(|_, w| w.gpioeen().set_bit());
                perip.RCC.ahb2enr.modify(|_, w| w.gpiocen().set_bit());

                // gpioモード変更
                // I2C
                let gpioc = &perip.GPIOC;
                gpioc.otyper.modify(|_, w| w.ot8().open_drain());
                gpioc.otyper.modify(|_, w| w.ot9().open_drain());
                gpioc.ospeedr.modify(|_, w| w.ospeedr8().very_high_speed());
                gpioc.ospeedr.modify(|_, w| w.ospeedr9().very_high_speed());
                gpioc.moder.modify(|_, w| w.moder8().alternate());
                gpioc.moder.modify(|_, w| w.moder9().alternate());
                gpioc.afrh.modify(|_, w| w.afrh8().af8());
                gpioc.afrh.modify(|_, w| w.afrh9().af8());
                // IO pin
                let gpio = &perip.GPIOE;
                gpio.moder.modify(|_, w| w.moder7().output());
                gpio.moder.modify(|_, w| w.moder14().input());
                gpio.moder.modify(|_, w| w.moder15().input());
                // PWM pin
                gpio.moder.modify(|_, w| w.moder8().alternate());
                gpio.moder.modify(|_, w| w.moder9().alternate());
                gpio.moder.modify(|_, w| w.moder10().alternate());
                gpio.moder.modify(|_, w| w.moder11().alternate());
                gpio.moder.modify(|_, w| w.moder12().alternate());
                gpio.moder.modify(|_, w| w.moder13().alternate());
                gpio.afrh.modify(|_, w| w.afrh8().af2());
                gpio.afrh.modify(|_, w| w.afrh9().af2());
                gpio.afrh.modify(|_, w| w.afrh10().af2());
                gpio.afrh.modify(|_, w| w.afrh11().af2());
                gpio.afrh.modify(|_, w| w.afrh12().af2());
                gpio.afrh.modify(|_, w| w.afrh13().af2());
                gpio.ospeedr.modify(|_, w| w.ospeedr8().very_high_speed());
                gpio.ospeedr.modify(|_, w| w.ospeedr9().very_high_speed());
                gpio.ospeedr.modify(|_, w| w.ospeedr10().very_high_speed());
                gpio.ospeedr.modify(|_, w| w.ospeedr11().very_high_speed());
                gpio.ospeedr.modify(|_, w| w.ospeedr12().very_high_speed());
                gpio.ospeedr.modify(|_, w| w.ospeedr13().very_high_speed());

                perip.RCC.ccipr.modify(|_, w| w.i2c3sel().pclk());
                perip.RCC.apb1enr1.modify(|_, w| w.i2c3en().enabled());
                perip.RCC.apb2enr.modify(|_, w| w.tim1en().enabled());

                let i2c = &perip.I2C3;
                i2c.cr1.modify(|_, w| w.pe().clear_bit());

                i2c.cr1.modify(|_, w| w.anfoff().disabled());
                i2c.cr1.modify(|_, w| w.dnf().no_filter());
                // 140MHz, presc:14-1->10MHz, t=100ns
                i2c.timingr.modify(|_, w| w.presc().bits(14 - 1));
                i2c.timingr.modify(|_, w| w.scll().bits(50 - 1)); // t_SCLL 5000ns
                i2c.timingr.modify(|_, w| w.sclh().bits(40 - 1)); // t_SCLH 4000ns
                i2c.timingr.modify(|_, w| w.sdadel().bits(5)); // 500ns
                i2c.timingr.modify(|_, w| w.scldel().bits(12 - 1)); // 1200ns

                i2c.cr1.modify(|_, w| w.nostretch().disabled());

                // Peripheral enable
                i2c.cr1.modify(|_, w| w.pe().set_bit());

                // For PWM
                let tim = &perip.TIM1;
                tim.psc.modify(|_, w| unsafe { w.bits(7 - 1) });
                tim.arr.modify(|_, w| unsafe { w.bits(800 - 1) }); // 25kHz
                                                                   // tim.dier.modify(|_, w| w.uie().set_bit());

                // OCxM mode
                tim.ccmr1_output().modify(|_, w| w.oc1m().pwm_mode1());
                tim.ccmr1_output().modify(|_, w| w.oc2m().pwm_mode1());
                tim.ccmr2_output().modify(|_, w| w.oc3m().pwm_mode1());
                // CCRx
                tim.ccr1.modify(|_, w| unsafe { w.ccr().bits(0) }); // x/800
                tim.ccr2.modify(|_, w| unsafe { w.ccr().bits(0) }); // x/800
                tim.ccr3.modify(|_, w| unsafe { w.ccr().bits(0) }); // x/800
                                                                    // CCxIE enable interrupt request

                // Set polarity
                // tim.ccer.modify(|_, w| w.cc1p().clear_bit());
                // tim.ccer.modify(|_, w| w.cc1np().clear_bit());
                // PWM mode
                // tim.cr1.modify(|_, w| unsafe { w.cms().bits(0b00) });

                // enable tim
                tim.cr1.modify(|_, w| w.cen().set_bit());
                // BDTR break and dead-time register
                tim.bdtr.modify(|_, w| w.moe().set_bit());
                // CCxE enable output
                tim.ccer.modify(|_, w| w.cc1e().set_bit());
                tim.ccer.modify(|_, w| w.cc1ne().set_bit());
                tim.ccer.modify(|_, w| w.cc2e().set_bit());
                tim.ccer.modify(|_, w| w.cc2ne().set_bit());
                tim.ccer.modify(|_, w| w.cc3e().set_bit());
                tim.ccer.modify(|_, w| w.cc3ne().set_bit());

                // Wait for ready
                while gpio.idr.read().idr14().is_low() && gpio.idr.read().idr15().is_high() {}

                // Unlock protected register.
                Self::write_2byte_i2c(&perip, 0x47, &[0x0B, 0x0F]);
                // Set gate drive voltage
                Self::write_2byte_i2c(&perip, 0x47, &[0x01, 0x00]);
                // Lock protected register.
                Self::write_2byte_i2c(&perip, 0x47, &[0x0B, 0x00]);
                // Clear Fault.
                Self::write_2byte_i2c(&perip, 0x47, &[0x09, 0xFF]);

                // Wake up
                gpio.bsrr.write(|w| w.bs7().set());
            }
        });
    }
    pub fn get_nfault_status(&self) -> bool {
        let mut result = false;
        free(|cs| match G_PERIPHERAL.borrow(cs).borrow().as_ref() {
            None => (),
            Some(perip) => {
                let gpio = &perip.GPIOE;
                result = gpio.idr.read().idr15().is_high();
            }
        });
        result
    }
    pub fn get_ready_status(&self) -> bool {
        let mut result = false;
        free(|cs| match G_PERIPHERAL.borrow(cs).borrow().as_ref() {
            None => (),
            Some(perip) => {
                let gpio = &perip.GPIOE;
                result = gpio.idr.read().idr14().is_high();
            }
        });
        result
    }
    /// Write 'data' to 'address's slave
    fn write_2byte_i2c(perip: &Peripherals, address: u16, data: &[u8]) {
        if data.len() != 2 {
            return;
        }

        free(|cs| match G_PERIPHERAL.borrow(cs).borrow().as_ref() {
            None => (),
            Some(perip) => {
                let i2c = &perip.I2C3;

                i2c.cr2.modify(|_, w| w.nbytes().bits(2));
                // Address
                i2c.cr2.modify(|_, w| w.sadd().bits(address << 1)); // 1000111
                i2c.cr2.modify(|_, w| w.add10().bit7());
                // Transfer direction
                i2c.cr2.modify(|_, w| w.rd_wrn().write());
                i2c.cr2.modify(|_, w| w.autoend().clear_bit());
                i2c.cr2.modify(|_, w| w.reload().clear_bit());
                while i2c.cr2.read().start().bit_is_set() {}
                i2c.cr2.modify(|_, w| w.start().set_bit());
                while i2c.isr.read().txis().bit_is_clear() {
                    // hprintln!("berr: {}", i2c.isr.read().berr().bit_is_set()).unwrap();
                    // hprintln!("arlo: {}", i2c.isr.read().arlo().bit_is_set()).unwrap();
                    // hprintln!("nackf: {}", i2c.isr.read().nackf().bit_is_set()).unwrap();
                }
                i2c.txdr.modify(|_, w| w.txdata().bits(data[0]));
                while i2c.isr.read().txis().bit_is_clear() {
                    // hprintln!("berr: {}", i2c.isr.read().berr().bit_is_set()).unwrap();
                    // hprintln!("arlo: {}", i2c.isr.read().arlo().bit_is_set()).unwrap();
                    // hprintln!("nackf: {}", i2c.isr.read().nackf().bit_is_set()).unwrap();
                }
                i2c.txdr.modify(|_, w| w.txdata().bits(data[1]));

                while i2c.isr.read().tc().bit_is_clear() {
                    // hprintln!("is_busy: {}", i2c.isr.read().busy().is_busy()).unwrap();
                    // hprintln!("nbytes: {}", i2c.cr2.read().nbytes().bits()).unwrap();
                }
                i2c.cr2.modify(|_, w| w.stop().stop());
            }
        });
    }
}

impl ThreePhaseMotorDriver for BldcPwm {
    fn enable(&self) {}
    fn disable(&self) {}
    /// 0~1
    fn set_pwm(&self, value: ThreePhaseVoltage<f32>) {
        free(|cs| match G_PERIPHERAL.borrow(cs).borrow().as_ref() {
            None => (),
            Some(perip) => {
                let tim = &perip.TIM1;
                tim.ccr1
                    .modify(|_, w| unsafe { w.ccr().bits((value.v_u * 800.) as u32) }); // x/800
                tim.ccr2
                    .modify(|_, w| unsafe { w.ccr().bits((value.v_v * 800.) as u32) }); // x/800
                tim.ccr3
                    .modify(|_, w| unsafe { w.ccr().bits((value.v_w * 800.) as u32) });
                // x/800
            }
        });
    }
    fn modify_pwm_output(&self, value: ThreePhaseValue<OutputStatus>) {
        free(|cs| match G_PERIPHERAL.borrow(cs).borrow().as_ref() {
            None => (),
            Some(perip) => {
                let tim = &perip.TIM1;
                match value.u {
                    OutputStatus::Enable => {
                        // CCxE enable output
                        tim.ccer.modify(|_, w| w.cc1e().set_bit());
                        tim.ccer.modify(|_, w| w.cc1ne().set_bit());
                    }
                    OutputStatus::Disable => {
                        // CCxE enable output
                        tim.ccer.modify(|_, w| w.cc1e().clear_bit());
                        tim.ccer.modify(|_, w| w.cc1ne().clear_bit());
                    }
                }
                match value.v {
                    OutputStatus::Enable => {
                        // CCxE enable output
                        tim.ccer.modify(|_, w| w.cc2e().set_bit());
                        tim.ccer.modify(|_, w| w.cc2ne().set_bit());
                    }
                    OutputStatus::Disable => {
                        // CCxE enable output
                        tim.ccer.modify(|_, w| w.cc2e().clear_bit());
                        tim.ccer.modify(|_, w| w.cc2ne().clear_bit());
                    }
                }
                match value.w {
                    OutputStatus::Enable => {
                        // CCxE enable output
                        tim.ccer.modify(|_, w| w.cc3e().set_bit());
                        tim.ccer.modify(|_, w| w.cc3ne().set_bit());
                    }
                    OutputStatus::Disable => {
                        // CCxE enable output
                        tim.ccer.modify(|_, w| w.cc3e().clear_bit());
                        tim.ccer.modify(|_, w| w.cc3ne().clear_bit());
                    }
                }
            }
        });
    }
}

pub struct Led0 {}
impl<'a> Indicator for Led0 {
    fn on(&self) {
        free(|cs| match G_PERIPHERAL.borrow(cs).borrow().as_ref() {
            None => (),
            Some(perip) => {
                let gpio = &perip.GPIOA;
                gpio.bsrr.write(|w| w.bs12().set());
            }
        });
    }
    fn off(&self) {
        free(|cs| match G_PERIPHERAL.borrow(cs).borrow().as_ref() {
            None => (),
            Some(perip) => {
                let gpio = &perip.GPIOA;
                gpio.bsrr.write(|w| w.br12().reset());
            }
        });
    }
    fn toggle(&self) {
        free(|cs| match G_PERIPHERAL.borrow(cs).borrow().as_ref() {
            None => (),
            Some(perip) => {
                let gpio = &perip.GPIOA;
                if gpio.odr.read().odr12().is_low() {
                    gpio.bsrr.write(|w| w.bs12().set());
                } else {
                    gpio.bsrr.write(|w| w.br12().reset());
                }
            }
        });
    }
}
impl Led0 {
    pub fn new() -> Self {
        Self {}
    }
    pub fn init(&self) {
        free(|cs| {
            match G_PERIPHERAL.borrow(cs).borrow().as_ref() {
                None => (),
                Some(perip) => {
                    let gpio = &perip.GPIOA;
                    // GPIOポートの電源投入(クロックの有効化)
                    perip.RCC.ahb2enr.modify(|_, w| w.gpioaen().set_bit());

                    // gpioモード変更
                    let gpio = &perip.GPIOA;
                    gpio.moder.modify(|_, w| w.moder12().output());
                }
            }
        });
    }
}

pub struct Led1 {}
impl<'a> Indicator for Led1 {
    fn on(&self) {
        free(|cs| match G_PERIPHERAL.borrow(cs).borrow().as_ref() {
            None => (),
            Some(perip) => {
                let gpio = &perip.GPIOA;
                gpio.bsrr.write(|w| w.bs11().set());
            }
        });
    }
    fn off(&self) {
        free(|cs| match G_PERIPHERAL.borrow(cs).borrow().as_ref() {
            None => (),
            Some(perip) => {
                let gpio = &perip.GPIOA;
                gpio.bsrr.write(|w| w.br11().reset());
            }
        });
    }
    fn toggle(&self) {
        free(|cs| match G_PERIPHERAL.borrow(cs).borrow().as_ref() {
            None => (),
            Some(perip) => {
                let gpio = &perip.GPIOA;
                if gpio.odr.read().odr11().is_low() {
                    gpio.bsrr.write(|w| w.bs11().set());
                } else {
                    gpio.bsrr.write(|w| w.br11().reset());
                }
            }
        });
    }
}
impl Led1 {
    pub fn new() -> Self {
        Self {}
    }
    pub fn init(&self) {
        free(|cs| {
            match G_PERIPHERAL.borrow(cs).borrow().as_ref() {
                None => (),
                Some(perip) => {
                    let gpio = &perip.GPIOA;
                    // GPIOポートの電源投入(クロックの有効化)
                    perip.RCC.ahb2enr.modify(|_, w| w.gpioaen().set_bit());

                    // gpioモード変更
                    let gpio = &perip.GPIOA;
                    gpio.moder.modify(|_, w| w.moder11().output());
                }
            }
        });
    }
}

pub struct Led2 {}
impl<'a> Indicator for Led2 {
    fn on(&self) {
        free(|cs| match G_PERIPHERAL.borrow(cs).borrow().as_ref() {
            None => (),
            Some(perip) => {
                let gpio = &perip.GPIOA;
                gpio.bsrr.write(|w| w.bs10().set());
            }
        });
    }
    fn off(&self) {
        free(|cs| match G_PERIPHERAL.borrow(cs).borrow().as_ref() {
            None => (),
            Some(perip) => {
                let gpio = &perip.GPIOA;
                gpio.bsrr.write(|w| w.br10().reset());
            }
        });
    }
    fn toggle(&self) {
        free(|cs| match G_PERIPHERAL.borrow(cs).borrow().as_ref() {
            None => (),
            Some(perip) => {
                let gpio = &perip.GPIOA;
                if gpio.odr.read().odr10().is_low() {
                    gpio.bsrr.write(|w| w.bs10().set());
                } else {
                    gpio.bsrr.write(|w| w.br10().reset());
                }
            }
        });
    }
}
impl Led2 {
    pub fn new() -> Self {
        Self {}
    }
    pub fn init(&self) {
        free(|cs| {
            match G_PERIPHERAL.borrow(cs).borrow().as_ref() {
                None => (),
                Some(perip) => {
                    let gpio = &perip.GPIOA;
                    // GPIOポートの電源投入(クロックの有効化)
                    perip.RCC.ahb2enr.modify(|_, w| w.gpioaen().set_bit());

                    // gpioモード変更
                    let gpio = &perip.GPIOA;
                    gpio.moder.modify(|_, w| w.moder10().output());
                }
            }
        });
    }
}
