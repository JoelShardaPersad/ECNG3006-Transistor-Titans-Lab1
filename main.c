/**
 * @file    main.c
 * @author  Joel Sharda Persad
 * @id      816024199
 *
 * Strategy: if-else method (Williams Ch. 6).
 *
 * The FSM runs only on a system tick (SysTick, every TICK_MS = 10 ms, a
 * factor of 500 ms). A tick counter measures the time since the last
 * received character: it counts up and holds once it passes 500 ms
 * (50 ticks). Every received character resets the counter.
 *
 * A received character is VALID if
 *      it differs from the previous character, OR
 *      it is the same character and the interval is greater than 500 ms.
 * A VALID character toggles the state (ON <-> OFF).
 * An invalid character (same char, interval <= 500 ms) is ignored.
 * The LED output follows the state: ON = LED on, OFF = LED off.
 *
 * Serial: SERCOM USART, 9600 baud, 8N1. Receive is polled (RXC flag).
 * Status messages are queued in a small buffer and sent from the main
 * loop, so slow transmission never delays the system tick.
 */

#include <stdint.h>
#include <stdbool.h>
#include "sam.h"                 /* CMSIS register definitions only */

/* ------------------------------------------------------------------ */
/* Board selection - enable exactly ONE                                */
/* ------------------------------------------------------------------ */

#define BOARD_ZERO_EDBG          /* LED PA17, RX PB23 / TX PB22 via programming USB port */
/* #define BOARD_MKR_ZERO */     /* LED PB08, RX PB23 / TX PB22 (needs USB-serial adapter) */
/* #define BOARD_ZERO_STYLE */   /* LED PA17, RX PA11 / TX PA10 (needs USB-serial adapter) */

#if defined(BOARD_ZERO_EDBG)
    #define LED_PORT_GROUP       (0U)               /* PORT A          */
    #define LED_PIN_NUM          (17U)              /* PA17            */
    #define UART_SERCOM          (SERCOM5)
    #define UART_APBC_MASK       (PM_APBCMASK_SERCOM5)
    #define UART_GCLK_ID         (SERCOM5_GCLK_ID_CORE)
    #define UART_RX_PORT_GROUP   (1U)               /* PORT B          */
    #define UART_RX_PIN_NUM      (23U)              /* PB23 (RX)       */
    #define UART_RX_MUX          (3U)               /* function D      */
    #define UART_RX_PAD          (3U)               /* SERCOM PAD[3]   */
    #define UART_TX_PORT_GROUP   (1U)               /* PORT B          */
    #define UART_TX_PIN_NUM      (22U)              /* PB22 (TX)       */
    #define UART_TX_MUX          (3U)               /* function D      */
    #define UART_TX_PAD          (1U)               /* TXPO 1 = PAD[2] */
#elif defined(BOARD_MKR_ZERO)
    #define LED_PORT_GROUP       (1U)               /* PORT B          */
    #define LED_PIN_NUM          (8U)               /* PB08            */
    #define UART_SERCOM          (SERCOM5)
    #define UART_APBC_MASK       (PM_APBCMASK_SERCOM5)
    #define UART_GCLK_ID         (SERCOM5_GCLK_ID_CORE)
    #define UART_RX_PORT_GROUP   (1U)               /* PORT B          */
    #define UART_RX_PIN_NUM      (23U)              /* PB23 (RX)       */
    #define UART_RX_MUX          (3U)               /* function D      */
    #define UART_RX_PAD          (3U)               /* SERCOM PAD[3]   */
    #define UART_TX_PORT_GROUP   (1U)               /* PORT B          */
    #define UART_TX_PIN_NUM      (22U)              /* PB22 (TX)       */
    #define UART_TX_MUX          (3U)               /* function D      */
    #define UART_TX_PAD          (1U)               /* TXPO 1 = PAD[2] */
#elif defined(BOARD_ZERO_STYLE)
    #define LED_PORT_GROUP       (0U)               /* PORT A          */
    #define LED_PIN_NUM          (17U)              /* PA17            */
    #define UART_SERCOM          (SERCOM0)
    #define UART_APBC_MASK       (PM_APBCMASK_SERCOM0)
    #define UART_GCLK_ID         (SERCOM0_GCLK_ID_CORE)
    #define UART_RX_PORT_GROUP   (0U)               /* PORT A          */
    #define UART_RX_PIN_NUM      (11U)              /* PA11 (RX)       */
    #define UART_RX_MUX          (2U)               /* function C      */
    #define UART_RX_PAD          (3U)               /* SERCOM PAD[3]   */
    #define UART_TX_PORT_GROUP   (0U)               /* PORT A          */
    #define UART_TX_PIN_NUM      (10U)              /* PA10 (TX)       */
    #define UART_TX_MUX          (2U)               /* function C      */
    #define UART_TX_PAD          (1U)               /* TXPO 1 = PAD[2] */
#else
    #error "Select a board: BOARD_ZERO_EDBG, BOARD_MKR_ZERO or BOARD_ZERO_STYLE"
#endif

/* ------------------------------------------------------------------ */
/* Configuration                                                       */
/* ------------------------------------------------------------------ */

#define SYSCLK_HZ            (8000000UL)            /* OSC8M, prescaler 1 */
#define UART_BAUD_RATE       (9600ULL)              /* match monitor_speed */

/** USART arithmetic baud value: 65536 * (1 - 16 * baud / f_ref). */
#define UART_BAUD_VALUE      ((uint16_t)(65536ULL - \
                              ((65536ULL * 16ULL * UART_BAUD_RATE) / SYSCLK_HZ)))

#define TICK_MS              (10U)                  /* system tick        */
#define DEBOUNCE_MS          (500U)                 /* debounce time      */
#define DEBOUNCE_TICKS       (DEBOUNCE_MS / TICK_MS)/* 50 ticks           */
#define TICK_COUNT_MAX       (DEBOUNCE_TICKS + 1U)  /* holds at >500 ms   */

#define SYSTICK_RELOAD       ((SYSCLK_HZ / (1000UL / TICK_MS)) - 1UL)

#define LED_PIN_MASK         (1UL << LED_PIN_NUM)

#define TX_BUF_SIZE          (256U)                 /* transmit queue     */

/* ------------------------------------------------------------------ */
/* Types                                                               */
/* ------------------------------------------------------------------ */

typedef enum
{
    FSM_STATE_OFF = 0,   /* LED off */
    FSM_STATE_ON         /* LED on  */
} fsm_state_t;

/* ------------------------------------------------------------------ */
/* Private data                                                        */
/* ------------------------------------------------------------------ */

static fsm_state_t s_state      = FSM_STATE_OFF;
static uint8_t     s_last_char  = 0U;
static uint16_t    s_tick_count = TICK_COUNT_MAX;   /* 1st char is valid */

static uint8_t     s_tx_buf[TX_BUF_SIZE];
static uint16_t    s_tx_head    = 0U;
static uint16_t    s_tx_tail    = 0U;

/* ------------------------------------------------------------------ */
/* Hardware layer - prototypes                                         */
/* ------------------------------------------------------------------ */

static void    hw_clock_init(void);
static void    hw_led_init(void);
static void    hw_uart_init(void);
static void    hw_systick_init(void);
static bool    hw_systick_expired(void);
static bool    hw_serial_rx_available(void);
static uint8_t hw_serial_read(void);
static void    hw_print_char(uint8_t c);
static void    hw_print(const char *text);
static void    hw_tx_service(void);
static void    hw_led_write(bool on);
static void    hw_pin_set_mux(uint8_t group, uint8_t pin, uint8_t mux);

/* ------------------------------------------------------------------ */
/* FSM                                                                 */
/* ------------------------------------------------------------------ */

/** Initialise the FSM: state OFF, LED off. */
static void fsm_init(void)
{
    s_state      = FSM_STATE_OFF;
    s_last_char  = 0U;
    s_tick_count = TICK_COUNT_MAX;

    hw_led_write(false);

    hw_print("Hardware configured. FSM is starting.\r\n");
    hw_print("Initial state: OFF. LED is off. Type in terminal to begin.\r\n\r\n");
}

/** Run the FSM once. Called exactly once every TICK_MS. */
static void fsm_tick(void)
{
    bool    valid = false;
    uint8_t rx_char;

    /* Count time since the last received character, then hold. */
    if (s_tick_count < TICK_COUNT_MAX)
    {
        s_tick_count++;
    }

    /* Check whether a serial character has arrived. */
    if (hw_serial_rx_available())
    {
        rx_char = hw_serial_read();

        hw_print("Received character: '");
        hw_print_char(rx_char);
        hw_print("'\r\n");

        valid = (rx_char != s_last_char) ||
                (s_tick_count > DEBOUNCE_TICKS);

        s_last_char  = rx_char;
        s_tick_count = 0U;           /* reset on every received character */

        if (valid)
        {
            hw_print("Character is VALID. Toggling state.\r\n");
        }
        else
        {
            hw_print("Character IGNORED due to debounce.\r\n\r\n");
        }
    }

    /* State transitions (if-else method). */
    if (s_state == FSM_STATE_OFF)
    {
        if (valid)
        {
            s_state = FSM_STATE_ON;
            hw_print("New state: ON (LED is now on)\r\n\r\n");
        }
    }
    else if (s_state == FSM_STATE_ON)
    {
        if (valid)
        {
            s_state = FSM_STATE_OFF;
            hw_print("New state: OFF (LED is now off)\r\n\r\n");
        }
    }
    else
    {
        s_state = FSM_STATE_OFF;     /* recover from an invalid state */
    }

    /* Output: LED follows the state. */
    hw_led_write(s_state == FSM_STATE_ON);
}

/* ------------------------------------------------------------------ */
/* Main                                                                */
/* ------------------------------------------------------------------ */

int main(void)
{
    hw_clock_init();
    hw_led_init();
    hw_uart_init();
    hw_systick_init();
    fsm_init();

    for (;;)
    {
        /* Send any queued status text, one character at a time. */
        hw_tx_service();

        /* System tick: run the FSM once every TICK_MS. */
        if (hw_systick_expired())
        {
            fsm_tick();
        }
    }
}

/* ------------------------------------------------------------------ */
/* Hardware layer - implementation                                     */
/* ------------------------------------------------------------------ */

/** CPU and peripheral clock: 8 MHz internal oscillator on GCLK0. */
static void hw_clock_init(void)
{
    SYSCTRL->OSC8M.bit.PRESC    = 0U;        /* divide by 1 -> 8 MHz */
    SYSCTRL->OSC8M.bit.ONDEMAND = 0U;
    SYSCTRL->OSC8M.bit.ENABLE   = 1U;
    while (SYSCTRL->PCLKSR.bit.OSC8MRDY == 0U)
    {
        /* wait for oscillator */
    }

    GCLK->GENDIV.reg = GCLK_GENDIV_ID(0U) | GCLK_GENDIV_DIV(1U);
    while (GCLK->STATUS.bit.SYNCBUSY)
    {
        /* wait for sync */
    }

    GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(0U) |
                        GCLK_GENCTRL_SRC_OSC8M |
                        GCLK_GENCTRL_GENEN;
    while (GCLK->STATUS.bit.SYNCBUSY)
    {
        /* wait for sync */
    }
}

/** Route a pin to a peripheral function (A=0, B=1, C=2, D=3, ...). */
static void hw_pin_set_mux(uint8_t group, uint8_t pin, uint8_t mux)
{
    uint8_t index = (uint8_t)(pin >> 1U);
    uint8_t value = PORT->Group[group].PMUX[index].reg;

    if ((pin & 1U) == 0U)
    {
        value = (uint8_t)((value & 0xF0U) | mux);           /* even pin */
    }
    else
    {
        value = (uint8_t)((value & 0x0FU) | (uint8_t)(mux << 4U)); /* odd */
    }

    PORT->Group[group].PMUX[index].reg    = value;
    PORT->Group[group].PINCFG[pin].reg   |= PORT_PINCFG_PMUXEN;
}

static void hw_led_init(void)
{
    PORT->Group[LED_PORT_GROUP].DIRSET.reg = LED_PIN_MASK;  /* output */
    PORT->Group[LED_PORT_GROUP].OUTCLR.reg = LED_PIN_MASK;  /* LED off */
}

static void hw_led_write(bool on)
{
    if (on)
    {
        PORT->Group[LED_PORT_GROUP].OUTSET.reg = LED_PIN_MASK;
    }
    else
    {
        PORT->Group[LED_PORT_GROUP].OUTCLR.reg = LED_PIN_MASK;
    }
}

/** SERCOM USART: 9600 baud, 8 data bits, no parity, 1 stop bit. */
static void hw_uart_init(void)
{
    PM->APBCMASK.reg |= UART_APBC_MASK;                     /* bus clock */

    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(UART_GCLK_ID) |     /* core clock */
                        GCLK_CLKCTRL_GEN_GCLK0 |
                        GCLK_CLKCTRL_CLKEN;
    while (GCLK->STATUS.bit.SYNCBUSY)
    {
        /* wait for sync */
    }

    hw_pin_set_mux(UART_RX_PORT_GROUP, UART_RX_PIN_NUM, UART_RX_MUX);
    hw_pin_set_mux(UART_TX_PORT_GROUP, UART_TX_PIN_NUM, UART_TX_MUX);

    UART_SERCOM->USART.CTRLA.reg = SERCOM_USART_CTRLA_SWRST;
    while (UART_SERCOM->USART.SYNCBUSY.bit.SWRST)
    {
        /* wait for reset */
    }

    UART_SERCOM->USART.CTRLA.reg = SERCOM_USART_CTRLA_DORD |  /* LSB first */
                                   SERCOM_USART_CTRLA_MODE_USART_INT_CLK |
                                   SERCOM_USART_CTRLA_RXPO(UART_RX_PAD) |
                                   SERCOM_USART_CTRLA_TXPO(UART_TX_PAD);

    UART_SERCOM->USART.CTRLB.reg = SERCOM_USART_CTRLB_CHSIZE(0U) | /* 8 bit */
                                   SERCOM_USART_CTRLB_RXEN |
                                   SERCOM_USART_CTRLB_TXEN;
    while (UART_SERCOM->USART.SYNCBUSY.bit.CTRLB)
    {
        /* wait for sync */
    }

    UART_SERCOM->USART.BAUD.reg = UART_BAUD_VALUE;

    UART_SERCOM->USART.CTRLA.reg |= SERCOM_USART_CTRLA_ENABLE;
    while (UART_SERCOM->USART.SYNCBUSY.bit.ENABLE)
    {
        /* wait for enable */
    }
}

static bool hw_serial_rx_available(void)
{
    return (UART_SERCOM->USART.INTFLAG.bit.RXC != 0U);
}

static uint8_t hw_serial_read(void)
{
    return (uint8_t)UART_SERCOM->USART.DATA.reg;   /* clears RXC */
}

/** Queue one character for transmission. Dropped if the queue is full. */
static void hw_print_char(uint8_t c)
{
    uint16_t next = (uint16_t)((s_tx_head + 1U) % TX_BUF_SIZE);

    if (next != s_tx_tail)
    {
        s_tx_buf[s_tx_head] = c;
        s_tx_head           = next;
    }
}

/** Queue a text string for transmission. */
static void hw_print(const char *text)
{
    while (*text != '\0')
    {
        hw_print_char((uint8_t)*text);
        text++;
    }
}

/** Send one queued character if the transmitter is ready. Non-blocking. */
static void hw_tx_service(void)
{
    if ((s_tx_tail != s_tx_head) &&
        (UART_SERCOM->USART.INTFLAG.bit.DRE != 0U))
    {
        UART_SERCOM->USART.DATA.reg = s_tx_buf[s_tx_tail];
        s_tx_tail = (uint16_t)((s_tx_tail + 1U) % TX_BUF_SIZE);
    }
}

/** System tick: SysTick counts down from SYSTICK_RELOAD, polled, no ISR. */
static void hw_systick_init(void)
{
    SysTick->LOAD = SYSTICK_RELOAD;
    SysTick->VAL  = 0U;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |   /* CPU clock, no IRQ */
                    SysTick_CTRL_ENABLE_Msk;
}

/** True once per tick. Reading CTRL clears COUNTFLAG. */
static bool hw_systick_expired(void)
{
    return ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) != 0U);
}