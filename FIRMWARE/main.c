/* main.c
 *      Update:         25.04.2012
 *      Author: ........hanns-jochen heckert
 *      Title:          Power Manager
 *      IDE:............Ubuntu 10.04 + CodeBlocks + AVRDude
 *      Programmer: ....TinyUSB (LadyADA)
 *      Target:.........ATtiny25
 *      fuses setup:    factory default
 *      code size       862 bytes
 * features
 *      hysteretic charge turns on / off charger
*       auto-shutdown disconnects amp from battery after 15 min music pause (timeout)
 *      LED flash warning after 15 Min/2 music pause
 *      to re-activate turn-off / turn-on power pushbutton.
 *      no auto-shutdown in case charger is externally powered.

 *      LED signalling      - indicating
 *      -static on          - either amp is switched on, no warnings, i.e.
 *                              - either powered by ext DC
 *                              - or powered by battery withbattery voltage ok and no timeout warning active
 *                          - or the battery is being charged for a longer period permanently by ext DC supply
 *      -static off         - either pushbutton turned off
 *                          - or bushbutton turned on but
 *                              - either autoshutdown caused after music pause timeout elapsed
 *                              - or autoshutdown caused by battery undervoltage / empty
 *      -regular flash      - warning announcing  autoshutdown soon coming
 *                              - either after half music pause timeout
 *                              - battery voltage has dropped below sane level
 *      -irregular flash   - the battery charger is disrupted each time battery voltage reached eof charge point
 *                           waiting for battery voltage dropping significantly

 *      standby current < 15uA.
 *
 *  pin usage of ATtiny25
 *  pin1    PB5    Reset     not used
 *  pin2    PB3    ADC3      battery voltage monitor
 *  pin3    PB4    ADC2      charge control / DC input voltage monitor
 *  pin4    GND    common    ground
 *  pin5    PB0    AIN0      ACOMP audio monitor input
 *  pin6    PB1    AIN1      ACOMP audio monitor ref / swi open-close detect
 *  pin7    PB2    PMOS      amp switch output
 *  pin8    VCC	   3..5V     powered either by battery or DC input, drives series LED
 */

#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/delay_basic.h>

// constants
#define TimeOut 1800                                        // audio pause timeout: 2count/sec 08:03/16:08 @1800
#define VChargeMax 240                                      // charge turn-off      14V3 @240
#define VChargeMin 220                                      // charge turn-on!!! 13V88 @230 13V28 @220
#define VBatteryLo 190                                      // warning threshold,   11V25 @190
#define VBatteryEmpty 175                                   // shutdown threshold;  10V38 @175
#define VExtHi 200                                          // Ext DC in threshold
#define EmptyBattCountDown 5                                // filter

// i/o mapping
#define pin1 (1<<PB5)
#define pin2 (1<<PB3)
#define pin3 (1<<PB4)
#define pin5 (1<<PB0)
#define pin6 (1<<PB1)
#define pin7 (1<<PB2)

#define BatteryIn pin2                                      // connected to battery voltage divider 10M0/2M00
#define ChargeOut pin3                                      // connected to inv input of NCP3063
#define AudioIn pin5                                        // audio pause detect input
#define PushButtonIn pin6                                   // connected to on-off-switch
#define AmpOut pin7                                         // connected to base of npn driving PMOS pos supply voltage breaker
#define PRRdefault ((1<<PRTIM1)|(1<<PRTIM0)|(1<<PRUSI))     // (P)ower (R)eduction: dis timer-, serial clock
#define ADMUXdefault ((1<<REFS2)|(1<<REFS1)|(1<<ADLAR))     // Ref2V56, result left adj
#define AnaCompOn (1<<ACIS1)|(1<<ACIS0)                     // ACSR: turn-on comp, irq disabled, rising edge irq, ACIE=lo
#define AnaCompOff (1<<ACD) | (1<<ACI)                      // ACSR: shutdown comp, clear IrqFlag, ACIE=lo

// global vars
uint16_t HalfTimeOut, WDTcount;
uint8_t VBattery,ChargeOn,ExtDC,AmpOn,LedOn,LedFlash,PushButtonClosed,LoBatt,EmptyBatt,EmptyBattCounter;
volatile uint8_t WDTirq,ANA_COMPirq;

void Defaults(void) {
    MCUSR               = 0;                                // clear reset flags
    DDRB                = AmpOut;                           //
    DIDR0               = (pin2 | pin3 | pin5);             // dis digital inputs
    PRR                 = PRRdefault;
    ADCSRB              = 0;                                // no comp MUX
    HalfTimeOut         = (TimeOut>>1);                     // warning @ 50% timeout
//!!!    ExtDC               = 0;
//!!!    AmpOn               = 0;
    EmptyBatt           = 0;
    EmptyBattCounter    = EmptyBattCountDown;
//!!!    PushButtonClosed    = 0;
}
ISR(WDT_vect){                                              // watchdog interrupt service routine
    sleep_disable();                                        // wakeup!
    WDTirq = 1;                                             // increment watchdog timeout counter
    WDTCR |= (1<<WDIE);                                     // wdt irq re-enable = avoid reset
}
ISR(ANA_COMP_vect){                                         // analog comp interrupt service routine: Do not wake up!
    ACSR = AnaCompOff ;                                     // done: no more comp irq until next WDT-tick
    ANA_COMPirq = 1;                                        // restart watchdog timeout counter
}
void WDTon(void) {
    WDTCR |= (1<<WDCE);                                     // disable WDT System Reset Mode
    WDTCR = 0;
    WDTCR = (1<<WDIE) | (1<<WDE) | (1<<WDP2) | (1<<WDP0);   // Watchdog timer 0.5sec ticker
}
void WDToff(void) {
    MCUSR = 0;                                              // clear WDRF
    WDTCR |= (1<<WDCE) | (1<<WDE);
    WDTCR = 0;                                              // clear WDE
}
void PreCharge (void) {                                     // setup these first for max settle times:
    ADMUX = (ADMUXdefault | (1<<MUX1));                     // select ADCin2 = pin3 = charge out
    ADCSRA = ((1<<ADEN)|(1<<ADPS0));                        // turn on adc, ref=2V56, fsample=fcpu/2
    ACSR = AnaCompOff ;                                     // default & avoid false ACompIrq during CheckPushButton
    DDRB = (PushButtonIn | AmpOut);                         // charge pushbutton input cap
    PORTB |= (PushButtonIn | ChargeOut);                    // @0us: active hi/pullup hi
    DDRB = AmpOut;                                          // eof charge pulse, open inputs
}
void CheckExtDC (void){
    ADCSRA |= (1<<ADSC);                                    // start ad-conversion
    ExtDC = 1;                                              // default
    while (0 == (ADCSRA & (1<<ADIF)));                      // first conversation takes 25cyc
    ADCSRA |= (1<<ADIF);                                    // clear irq flag
    if (ADCH < VExtHi ) {                                   // external DC-input? polling comp out, comp in = INV!
        ExtDC = 0;                                          // N
        PORTB &= ~ChargeOut;                                // deglitch input+pullup->active lo: turnoff pullup
        DDRB |= ChargeOut;
        PORTB &= ~ChargeOut;                                // @94us: keep NCP3063 inv input active low while not powered
    }
}
void CheckBattery (void) {
    ADMUX = ADMUXdefault |(1<<MUX1)|(1<<MUX0);              // select ADCin3 = pin2 = battery monitor in
    ADCSRA = ((1<<ADEN)|(1<<ADSC)|(1<<ADPS0));              // start ad-conversion fsample=clk/2
    while (0 == (ADCSRA & (1<<ADIF)));                      // first conversation takes 25cyc
    ADCSRA |= (1<<ADIF);                                    // clear irq flag
    VBattery = ADCH;                                        // bit9..2 , 8bit result
    if (VBattery>VChargeMax) {
        ChargeOn = 0;                                       // 2-point hysteretic battery charge
        EmptyBatt=0;                                        // force 1 x full charge cycle after empty batt auto shutdown
    }
    if (VBattery<VChargeMin) ChargeOn = 1;                  // charging
    LoBatt=0;                                               // default
    if (VBattery<VBatteryLo) LoBatt=1;                      // LED warning
    if (VBattery<VBatteryEmpty) {                           // filter
        EmptyBattCounter--;
        if (EmptyBattCounter==0) EmptyBatt=1;               // auto shutdown
    }
    else EmptyBattCounter = EmptyBattCountDown;             // clear filter

}
void ChargeBattery (void) {
    if (!ExtDC) return;
    if (!ChargeOn) {                                        //
        PORTB &= ~ChargeOut;                                // deglitch
        DDRB |= ChargeOut;
        PORTB &= ~ChargeOut;                                // active low
    }
    else {
        DDRB &= ~ChargeOut;                                 // input
        PORTB |= ChargeOut;                                 // +pullup
    }
}
void CheckPushButton(void) {                                // turn on: 34us discharge time

    if (PINB&PushButtonIn) {                                // swi closed? - open/closed=hi/lo
        PushButtonClosed = 0;                               // N: amp ofF
    }
    else {
        if (!PushButtonClosed)  {                           // Y: transition open -> closed?
        WDTcount = 0;                                       // Y: restart watchdog timeout
        }
        PushButtonClosed = 1;
    }
    PORTB &= ~PushButtonIn;                                 // Y: de-glitch
    DDRB |= PushButtonIn;                                   // -> active lo
    PORTB &= ~PushButtonIn;                                 // mark end of PushButton check
    DDRB &= ~PushButtonIn;                                  // enable 30mV ACompRef
}
void SwitchAmp (void) {
    AmpOn = 0;                                              // default switch open
    if (PushButtonClosed) {                                 //
        if (ExtDC) {                                        // Y: dc input powered?
            AmpOn = 1;                                      // Y: override immediately any autoshutdown
            WDTcount = 0;                                   // restart full timeout
        }
        else {                                              // N: battery powered
            if (!EmptyBatt) {
                if (WDTcount<TimeOut) {                     // timeout?
                    AmpOn = 1;                              // N
                    ACSR = AnaCompOn;                       // turn-on comp, ACIE=lo
                    ACSR = AnaCompOn | (1<<ACIE);           // ena irq
                } //timeout
            } //battery ok
        } //battery powered
    } //pushbutton closed                                   // no default for glitch free switch:
    if (AmpOn) PORTB |= AmpOut;                             // amp on?  Y: close PMOS switch
    else  PORTB &= ~AmpOut;                                 // N: open PMOS switch
}
void SwitchLed (void) {                                     // LED controlled by mcu power consumption
    LedOn = 0;                                              // default
    if (AmpOn) {                                            // amp active?
        LedOn = 1;                                          // Y: default
        if ((LoBatt)&((WDTcount&1))) LedOn = 0;             // lo battery LED flash warning
        if ((WDTcount>HalfTimeOut)&((WDTcount&1))) LedOn = 0; // half timeout LED flash warning
    }
    else  if ((ExtDC)&(ChargeOn)) LedOn = 1;                // N: charger
}
void GotoSleep (void) {
        PORTB |= PushButtonIn;                              // @235us pgm end mark
        PORTB &= ~PushButtonIn;                             //
    if (!LedOn) {
        cli();
        ADCSRA &= ~(1<<ADEN);                               // ADC=off saves power during sleep 20120325
        set_sleep_mode(SLEEP_MODE_PWR_DOWN);
        sleep_enable();
        sei();                                              // do sei() to set wdie during wdt-irq to avoid wdt-reset
        sleep_cpu();
    }
}
//++++++++ Main Program

int main(void) {
    cli();                                                  // cboot entry - after connecting battery
    WDToff();
    Defaults();
    WDTon();
    sei();
    for (;;) {
        if (ANA_COMPirq){                                   // new analog comparator irq?
            ANA_COMPirq = 0;                                // Y: done
            WDTcount = 0;                                   // restart watchdog timeout
        }
        if (WDTirq) {                                       // active phase? 2x/1sec
            WDTirq = 0;                                     // Y: done
            if (WDTcount<TimeOut)  WDTcount++;              // execute 2x/sec - ca200usec
                PreCharge();
                CheckExtDC();
                CheckPushButton();
                CheckBattery();
                ChargeBattery();
                SwitchAmp();
                SwitchLed();
                GotoSleep();
        } //WDTirq
        } //for
    } //main
