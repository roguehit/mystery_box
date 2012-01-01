/*
 ** ----------------------------------------------------------------------------
 ** "THE BEER-WARE LICENSE" (Revision 42):
 ** <roguehit@gmail.com> wrote this file. As long as you retain this notice you
 ** can do whatever you want with this stuff. If we meet some day, and you think
 ** this stuff is worth it, you can buy me a beer in return Rohit N.
 ** ----------------------------------------------------------------------------
 ** LCD stuff comes from avr-libc/docs/examples/twistdio/
 **/

#include "defines.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <avr/io.h>
#include <util/delay.h>
#include <math.h>
#include <inttypes.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <stdlib.h>
#include <util/delay.h>
#include <util/twi.h>

#include "common.h"

#define EARTH_RADIUS_METERS  6372795.0f
#define PI                   3.1469f
#define KILO_PER_METER       0.001f
#define  NEXT_GPS_CHAR(a) \
    do{ \
        loop_until_bit_is_set(UCSR0A, RXC0); \
        a = UDR0; \
    }while(0)

#define  COMMAND_LEN            5
#define  BUFF_LEN               15
#define  NUM_BUFF               3
#define  CLEAR_SCREEN()         hd44780_outcmd(HD44780_CLR)

#define GLUE(a, b)              a##b
#define HD44780_BUSYFLAG        0x80


/* single-bit macros, used for control bits */
#define SET_(what, p, m) GLUE(what, p) |= (1 << (m))
#define CLR_(what, p, m) GLUE(what, p) &= ~(1 << (m))
#define GET_(/* PIN, */ p, m) GLUE(PIN, p) & (1 << (m))
#define SET(what, x) SET_(what, x)
#define CLR(what, x) CLR_(what, x)
#define GET(/* PIN, */ x) GET_(x)

/* nibble macros, used for data path */
#define ASSIGN_(what, p, m, v) GLUE(what, p) = (GLUE(what, p) & \
        ~((1 << (m)) | (1 << ((m) + 1)) | \
            (1 << ((m) + 2)) | (1 << ((m) + 3)))) | \
((v) << (m))
#define READ_(what, p, m) (GLUE(what, p) & ((1 << (m)) | (1 << ((m) + 1)) | \
            (1 << ((m) + 2)) | (1 << ((m) + 3)))) >> (m)
#define ASSIGN(what, x, v) ASSIGN_(what, x, v)
#define READ(what, x) READ_(what, x)

struct nav{
        int degree;
        int seconds;
        int minutes;
};

struct clue{
        char* clue;
        float latitude;
        float longitude;
};

struct clue clues[]={
        {"Clue 1 string here",19.018508,72.843995} //Dadar Station
        //Add more clues here
};

/*Route stderr to lcd_putchar*/
FILE lcd_str = FDEV_SETUP_STREAM(lcd_putchar, NULL, _FDEV_SETUP_WRITE);

/*EEPROM entry for clue counter*/
uint8_t EEMEM eeprom_clue_count;

void uart_4800(void)
{
    /*Set baud rate */ 
    UBRR0H = (unsigned char)(MYUBRR>>8); 
    UBRR0L = (unsigned char) MYUBRR; 
    /* Enable receiver and transmitter   */
    UCSR0B = (1<<RXEN0)|(1<<TXEN0); 
    /* Frame format: 8data, No parity, 1stop bit */ 
    UCSR0C = (3<<UCSZ00);  
}

/*
 * Send one pulse to the E signal (enable).  Mind the timing
 * constraints.  If readback is set to true, read the HD44780 data
 * pins right before the falling edge of E, and return that value.
 */
static inline uint8_t
hd44780_pulse_e(bool readback) __attribute__((always_inline));

    static inline uint8_t
hd44780_pulse_e(bool readback)
{
    uint8_t x;

    SET(PORT, HD44780_E);
    /*
     * Guarantee at least 500 ns of pulse width.  For high CPU
     * frequencies, a delay loop is used.  For lower frequencies, NOPs
     * are used, and at or below 1 MHz, the native pulse width will
     * already be 1 us or more so no additional delays are needed.
     */
#if F_CPU > 4000000UL
    _delay_us(0.5);
#else
    /*
     * When reading back, we need one additional NOP, as the value read
     * back from the input pin is sampled close to the beginning of a
     * CPU clock cycle, while the previous edge on the output pin is
     * generated towards the end of a CPU clock cycle.
     */
    if (readback)
        __asm__ volatile("nop");
#  if F_CPU > 1000000UL
    __asm__ volatile("nop");
#    if F_CPU > 2000000UL
    __asm__ volatile("nop");
    __asm__ volatile("nop");
#    endif /* F_CPU > 2000000UL */
#  endif /* F_CPU > 1000000UL */
#endif
    if (readback)
        x = READ(PIN, HD44780_D4);
    else
        x = 0;
    CLR(PORT, HD44780_E);

    return x;
}

/*
 * Send one nibble out to the LCD controller.
 */
    static void
hd44780_outnibble(uint8_t n, uint8_t rs)
{
    CLR(PORT, HD44780_RW);
    if (rs)
        SET(PORT, HD44780_RS);
    else
        CLR(PORT, HD44780_RS);
    ASSIGN(PORT, HD44780_D4, n);
    (void)hd44780_pulse_e(false);
}

/*
 * Send one byte to the LCD controller.  As we are in 4-bit mode, we
 * have to send two nibbles.
 */
    void
hd44780_outbyte(uint8_t b, uint8_t rs)
{
    hd44780_outnibble(b >> 4, rs);
    hd44780_outnibble(b & 0xf, rs);
}

/*
 * Read one nibble from the LCD controller.
 */
    static uint8_t
hd44780_innibble(uint8_t rs)
{
    uint8_t x;

    SET(PORT, HD44780_RW);
    ASSIGN(DDR, HD44780_D4, 0x00);
    if (rs)
        SET(PORT, HD44780_RS);
    else
        CLR(PORT, HD44780_RS);
    x = hd44780_pulse_e(true);
    ASSIGN(DDR, HD44780_D4, 0x0F);
    CLR(PORT, HD44780_RW);

    return x;
}

/*
 * Read one byte (i.e. two nibbles) from the LCD controller.
 */
    uint8_t
hd44780_inbyte(uint8_t rs)
{
    uint8_t x;

    x = hd44780_innibble(rs) << 4;
    x |= hd44780_innibble(rs);

    return x;
}

/*
 * Wait until the busy flag is cleared.
 */
    void
hd44780_wait_ready(bool longwait)
{
#if USE_BUSY_BIT
    while (hd44780_incmd() & HD44780_BUSYFLAG) ;
#else
    if (longwait)
        _delay_ms(1.52);
    else
        _delay_us(37);
#endif
}

/*
 * Initialize the LCD controller.
 *
 * The initialization sequence has a mandatory timing so the
 * controller can safely recognize the type of interface desired.
 * This is the only area where timed waits are really needed as
 * the busy flag cannot be probed initially.
 */
    void
hd44780_init(void)
{
    SET(DDR, HD44780_RS);
    SET(DDR, HD44780_RW);
    SET(DDR, HD44780_E);
    ASSIGN(DDR, HD44780_D4, 0x0F);

    _delay_ms(15);		/* 40 ms needed for Vcc = 2.7 V */
    hd44780_outnibble(HD44780_FNSET(1, 0, 0) >> 4, 0);
    _delay_ms(4.1);
    hd44780_outnibble(HD44780_FNSET(1, 0, 0) >> 4, 0);
    _delay_ms(0.1);
    hd44780_outnibble(HD44780_FNSET(1, 0, 0) >> 4, 0);
    _delay_us(37);

    hd44780_outnibble(HD44780_FNSET(0, 1, 0) >> 4, 0);
    hd44780_wait_ready(false);
    hd44780_outcmd(HD44780_FNSET(0, 1, 0));
    hd44780_wait_ready(false);
    hd44780_outcmd(HD44780_DISPCTL(0, 0, 0));
    hd44780_wait_ready(false);
}

/*
 * Prepare the LCD controller pins for powerdown.
 */
    void
hd44780_powerdown(void)
{
    ASSIGN(PORT, HD44780_D4, 0);
    CLR(PORT, HD44780_RS);
    CLR(PORT, HD44780_RW);
    CLR(PORT, HD44780_E);
}

/*
 * Setup the LCD controller.  First, call the hardware initialization
 * function, then adjust the display attributes we want.
 */
    void
lcd_init(void)
{

    hd44780_init();

    /*
     * Clear the display.
     */
    hd44780_outcmd(HD44780_CLR);
    hd44780_wait_ready(0);

    /*
     * Entry mode: auto-increment address counter, no display shift in
     * effect.
     */
    hd44780_outcmd(HD44780_ENTMODE(1, 0));
    hd44780_wait_ready(0);

    /*
     * Enable display, activate non-blinking cursor.
     */
    hd44780_outcmd(HD44780_DISPCTL(1, 1, 0));
    hd44780_wait_ready(0);
}

/*
 * Send character c to the LCD display.  After a '\n' has been seen,
 * the next character will first clear the display.
 */
    int
lcd_putchar(char c, FILE *unused)
{
    static bool nl_seen;

    if (nl_seen && c != '\n')
    {
        /*
         * First character after newline, clear display and home cursor.
         */
        hd44780_wait_ready(0);
        hd44780_outcmd(HD44780_CLR);
        hd44780_wait_ready(0);
        hd44780_outcmd(HD44780_HOME);
        hd44780_wait_ready(0);
        hd44780_outcmd(HD44780_DDADDR(0));

        nl_seen = false;
    }
    if (c == '\n')
    {
        nl_seen = true;
    }
    else
    {
        hd44780_wait_ready(0);
        hd44780_outdata(c);
    }

    return 0;
}

void pwm_init()
{
    //Initialize Timer0
    TCCR0A = _BV(WGM00) | _BV(COM0A1) ;
    TCCR0B = (1 << CS00);

    //Enable output
    DDRD |= _BV(PD6);
}
void open_lock()
{
    OCR0A = 40; 
    _delay_ms(100);
    OCR0A = 0; 
    DDRD &= ~(_BV(PD6));
}

float radians(float x)
{
        return x*PI/180.0;
}
float distance_between_points( float Lat1, float Lon1, float Lat2, float Lon2, float unit_conversion )
{
  float dLat = radians( Lat2 - Lat1 );
  float dLon = radians( Lon2 - Lon1 );
  
  float a = sin( dLat / 2.0f ) * sin( dLat / 2.0f ) +
            cos( radians( Lat1 ) ) * cos( radians( Lat2 ) ) *
            sin( dLon / 2.0f ) * sin( dLon / 2.0f );
            
  float d = 2.0f * atan2( sqrt( a ), sqrt( 1.0f - a ) );

  return d * EARTH_RADIUS_METERS * unit_conversion;
}

void waitfor_valid_gps_lock()
{
    int i=0;
    char data;
    char command[COMMAND_LEN + 1];

    while(1)
    {
        NEXT_GPS_CHAR(data);
        if('$' == data)
        { 
            for(i=0;i<COMMAND_LEN;i++)
            {
                NEXT_GPS_CHAR(command[i]); 
            }
            command[i] = '\0';

            if(0 == strcmp(command,"GPGGA"))
            {
                break;
            }
        }
    }
}

void get_gps_data(float* longitude, float* latitude)
{	
    int i=0;
    char nema_sentence[60];
    int num_comma=0;
    struct nav lon,lat;
    char temp[BUFF_LEN];

    memset(&lon,0,sizeof(struct nav));
    memset(&lat,0,sizeof(struct nav));

    waitfor_valid_gps_lock();

    for(i=0;i<60;i++)
    {
	NEXT_GPS_CHAR(nema_sentence[i]);
    }
    
    for(i=0;i<60;i++)
    {
	if(',' == nema_sentence[i])
        {
            num_comma++;
            continue;
        }
        if(2 == num_comma)
        {
            strncpy(temp,nema_sentence+i,2);
            temp[2]='\0';
            lat.degree=atoi(temp);
            i+=2;

            strncpy(temp,nema_sentence+i,2);
            temp[2]='\0';
            lat.minutes=atoi(temp);
            i+=3;

            strncpy(temp,nema_sentence+i,4);
            temp[4]='\0';
            lat.seconds=atoi(temp);
            i+=3;
        }
        if(4 == num_comma)
        {
            strncpy(temp,nema_sentence+i,3);
            temp[3]='\0';
            lon.degree=atoi(temp+1);
            i+=3;

            strncpy(temp,nema_sentence+i,2);
            temp[2]='\0';
            lon.minutes=atoi(temp);
            i+=3;

            strncpy(temp,nema_sentence+i,4);
            temp[4]='\0';
            lon.seconds=atoi(temp);
            i+=4;
	    break;
         }

    }

    /*Convert everything to degree*/    
    *longitude = (lon.seconds/10000.0)/60.0 + (lon.minutes/60.0) + (float)lon.degree;
    *latitude = (lat.seconds/10000.0)/60.0 + (lat.minutes/60.0) + (float)lat.degree;
}

void main()
{
    int i=0;
    int timeout=64;
    float longitude=0,latitude=0,distance=0;
    uint8_t clue_count;
    int num_clues = sizeof(clues)/(sizeof(typeof(clues[0])));

    stderr = &lcd_str;
    lcd_init();
    pwm_init();
    uart_4800();
   
    clue_count = eeprom_read_byte(&eeprom_clue_count);
    eeprom_busy_wait();

    /*Default eeprom value is 0xff*/
    if(clue_count == 0xff)
    {
        clue_count = 0;
    }
        
    if(clue_count == (num_clues-1))
    {
        goto jump_to_open_door;        
    }

    else
    {
        CLEAR_SCREEN();
        fprintf(stderr,"Acquiring Signal...\n");
        while(1)
        {
            if(timeout < 0)
            {
                goto power_collapse;            
            }

            while(1)
            {
                get_gps_data(&longitude,&latitude);
                if(longitude > 0 || latitude > 0)
                    break;
            }

            CLEAR_SCREEN();
            fprintf(stderr,"%s",clues[clue_count].clue);
            distance = distance_between_points(latitude,
                    longitude,
                    clues[clue_count].latitude,
                    clues[clue_count].longitude,
                    KILO_PER_METER); 

            fprintf(stderr,"Distance - %fKm (%d/%d)",distance,clue_count+1,num_clues);
            if(distance < 0.2)
            {
		clue_count++;
                eeprom_write_byte(&eeprom_clue_count,clue_count);
                if(clue_count == (num_clues-1))
                {
                    goto jump_to_open_door;
                }
            }

            for(i=0;i<20;i++,timeout--){
                /*Scroll the Display to right*/ 
                hd44780_outcmd(HD44780_SHIFT(1,0));
                _delay_ms(800);
            }
        }
    }

jump_to_open_door:        
    CLEAR_SCREEN();
    fprintf(stderr,"This is it, here is the key....");
    _delay_ms(10000);

power_collapse:
    CLEAR_SCREEN();
    fprintf(stderr,"Powering Down....\n");
    _delay_ms(5000);
    /*Power collapse*/
    DDRB |= _BV(PB4);
    PORTB|= _BV(PB4);

    return;
}
