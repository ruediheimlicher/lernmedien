#define CPU_16MHz       0x00
#define CPU_8MHz        0x01

#define OHNE_INTERFACE 0

#define USB_PACKETSIZE 64

#define TEST 0
//Oszi
#define OSZIPORT           PORTD
#define OSZIPORTDDR        DDRD
#define OSZIPORTPIN        PIND
#define OSZI_PULS_A        0
#define PULSA              4
//#define PULSB              5
//#define OSZI_PULS_B        5

#define TEST_PIN           5


#define OSZIA_LO OSZIPORT &= ~(1<<PULSA)
#define OSZIA_HI OSZIPORT |= (1<<PULSA)
#define OSZIA_TOGG OSZIPORT ^= (1<<PULSA)

/*
#define OSZI_B_LO OSZIPORT &= ~(1<<OSZI_PULS_B)
#define OSZI_B_HI OSZIPORT |= (1<<OSZI_PULS_B)
#define OSZI_B_TOGG OSZIPORT ^= (1<<OSZI_PULS_B)
*/



#define LOOPLEDDDR          DDRD    
#define LOOPLEDPORT         PORTD   
#define LOOPLED             6       // fix verdrahtet

#if(MCU ==atmega32u4)//        # Teensy 2.0
#define SPI_PORT           PORTB
#define SPI_DDR            DDRB
#define SPI_MISO           PB3
#define SPI_MOSI           PB2
#define SPI_CLK            PB1
#define SPI_SS             PB0
#endif


// bits von usbstatus

#define WRITEAUTO          0

#define WRITETEMPERATUR    2

#define READAUTO           1


// bits von hoststatus
#define TEENSYPRESENT      7
#define MESSUNG_OK      6
#define DOWNLOAD_OK     5


// bits von spistatus

#define WRITE_SPANNUNG        1
#define WRITE_STROM           2
#define SPI_RUN_BIT           7

#define SOFT_SPI_PORT           PORTB
#define SOFT_SPI_DDR            DDRB
#define SOFT_SPI_MISO           PB3
#define SOFT_SPI_MOSI           PB2
#define SOFT_SPI_SCLK           PB1
#define SOFT_SPI_SS_0            PB0
#define SOFT_SPI_SS_1            PB0


// USB buffer

// Bits in usbstatus1
#define SAVE_SD_BIT        0
#define SAVE_SD_RUN_BIT        1
#define SAVE_SD_STOP_BIT        2

// Bytes fuer Sicherungsort der Daten auf SD
#define ABSCHNITT_BYTE     2
#define BLOCKOFFSETLO_BYTE      3
#define BLOCKOFFSETHI_BYTE      4

#define MESSUNG_START      0xC0
#define MESSUNG_STOP       0xC1

#define STARTSEKUNDELO_BYTE      5
#define STARTSEKU£NDEHI_BYTE      6

#define DATENBREITE_BYTE         7

#define PACKETCOUNT_BYTE         8

#define TAKT_LO_BYTE 14
#define TAKT_HI_BYTE 15

#define DATA_START_BYTE   15    // erstes byte fuer Data

#define DATACOUNT_LO       12 // Nummer der Messung, fortlaufend
#define DATACOUNT_HI       13

// USB Eingang
// Temperatur
#define DSLO               12
#define DSHI               13

// ADC
#define  ADCLO             10
#define  ADCHI             11

// USB Ausgang
#define SERVOALO           10
#define SERVOAHI           11

// write SD
#define WRITE_MMC_TEST     0xF1


// default
#define DEFAULT            0xFE

// logger
#define LOGGER_START       0xA0
#define LOGGER_CONT        0xA1

#define LOGGER_STOP        0xAF

#define LOGGER_SETTING     0xB0 // Setzen der Settings fuer die Messungen
#define MESSUNG_DATA        0xB1


#define USB_STOP           0xAA

// EEPROM Speicherorte

#pragma mark               Transfer

#define TRANSFERBLOCK      0xA0  //Array fuer Transfer an Interface. Byte 1: Abschnitt Byte 2,3: Blockoffset Byte 4,5: Anzahl Blocks
#define TASK_OFFSET        0x2000 // Ort fuer Einstellungen

#define PACKET_SIZE     0x30 // 48 bytes fuer USB-Transfer
//#define PACKET_SIZE     0x18 // 24 bytes fuer USB-Transfer


#define HEADER_SIZE  8 // Header zu beginn der Loggerdaten

/*
// Teensy2 int ref/TL431
#define TASTE1		15
#define TASTE2		23
#define TASTE3		34
#define TASTE4		51
#define TASTE5		72
#define TASTE6		94
#define TASTE7		120
#define TASTE8		141
#define TASTE9		155
#define TASTE_L	168
#define TASTE0		178
#define TASTE_R	194
*/
/*
// Teensy2 int ref/TL431
#define TASTE1		15
#define TASTE2		23
#define TASTE3		34
#define TASTE4		51
#define TASTE5		72
#define TASTE6		94
#define TASTE7		120
#define TASTE8		141
#define TASTE9		155
#define TASTE_L	168
#define TASTE0		178
#define TASTE_R	194
*/
// Teensy2 int Vcc Tastatur1
/*
#define TASTE1		16
#define TASTE2		26
#define TASTE3		40
#define TASTE4		62
#define TASTE5		88
#define TASTE6		114
#define TASTE7		146
#define TASTE8		177
#define TASTE9		222


#define TASTE_L	168
#define TASTE0		178
#define TASTE_R	194
*/




// SPI



#define SPI_BUFSIZE 8

#define STARTDELAYBIT       0
#define HICOUNTBIT          1

#define WDTBIT              7

#define LOOPLEDLONG           0xDFFF
#define LOOPLEDSHORT          0x4FFF




#define USB_ATTACH            1 // USB_Spannung detektiert

#define SD_DATA_SIZE 512
/* commands available in SPI mode */

/* CMD0: response R1 */
#define CMD_GO_IDLE_STATE 0x00
/* CMD1: response R1 */
#define CMD_SEND_OP_COND 0x01
/* CMD8: response R7 */
#define CMD_SEND_IF_COND 0x08
/* CMD9: response R1 */
#define CMD_SEND_CSD 0x09
/* CMD10: response R1 */
#define CMD_SEND_CID 0x0a
/* CMD12: response R1 */
#define CMD_STOP_TRANSMISSION 0x0c
/* CMD13: response R2 */
#define CMD_SEND_STATUS 0x0d
/* CMD16: arg0[31:0]: block length, response R1 */
#define CMD_SET_BLOCKLEN 0x10
/* CMD17: arg0[31:0]: data address, response R1 */
#define CMD_READ_SINGLE_BLOCK 0x11
/* CMD18: arg0[31:0]: data address, response R1 */
#define CMD_READ_MULTIPLE_BLOCK 0x12
/* CMD24: arg0[31:0]: data address, response R1 */
#define CMD_WRITE_SINGLE_BLOCK 0x18
/* CMD25: arg0[31:0]: data address, response R1 */
#define CMD_WRITE_MULTIPLE_BLOCK 0x19
/* CMD27: response R1 */
#define CMD_PROGRAM_CSD 0x1b
/* CMD28: arg0[31:0]: data address, response R1b */
#define CMD_SET_WRITE_PROT 0x1c
/* CMD29: arg0[31:0]: data address, response R1b */
#define CMD_CLR_WRITE_PROT 0x1d
/* CMD30: arg0[31:0]: write protect data address, response R1 */
#define CMD_SEND_WRITE_PROT 0x1e
/* CMD32: arg0[31:0]: data address, response R1 */
#define CMD_TAG_SECTOR_START 0x20
/* CMD33: arg0[31:0]: data address, response R1 */
#define CMD_TAG_SECTOR_END 0x21
/* CMD34: arg0[31:0]: data address, response R1 */
#define CMD_UNTAG_SECTOR 0x22
/* CMD35: arg0[31:0]: data address, response R1 */
#define CMD_TAG_ERASE_GROUP_START 0x23
/* CMD36: arg0[31:0]: data address, response R1 */
#define CMD_TAG_ERASE_GROUP_END 0x24
/* CMD37: arg0[31:0]: data address, response R1 */
#define CMD_UNTAG_ERASE_GROUP 0x25
/* CMD38: arg0[31:0]: stuff bits, response R1b */
#define CMD_ERASE 0x26
/* ACMD41: arg0[31:0]: OCR contents, response R1 */
#define CMD_SD_SEND_OP_COND 0x29
/* CMD42: arg0[31:0]: stuff bits, response R1b */
#define CMD_LOCK_UNLOCK 0x2a
/* CMD55: arg0[31:0]: stuff bits, response R1 */
#define CMD_APP 0x37
/* CMD58: arg0[31:0]: stuff bits, response R3 */
#define CMD_READ_OCR 0x3a
/* CMD59: arg0[31:1]: stuff bits, arg0[0:0]: crc option, response R1 */
#define CMD_CRC_ON_OFF 0x3b

/* command responses */
/* R1: size 1 byte */
#define R1_IDLE_STATE 0
#define R1_ERASE_RESET 1
#define R1_ILL_COMMAND 2
#define R1_COM_CRC_ERR 3
#define R1_ERASE_SEQ_ERR 4
#define R1_ADDR_ERR 5
#define R1_PARAM_ERR 6
/* R1b: equals R1, additional busy bytes */
/* R2: size 2 bytes */
#define R2_CARD_LOCKED 0
#define R2_WP_ERASE_SKIP 1
#define R2_ERR 2
#define R2_CARD_ERR 3
#define R2_CARD_ECC_FAIL 4
#define R2_WP_VIOLATION 5
#define R2_INVAL_ERASE 6
#define R2_OUT_OF_RANGE 7
#define R2_CSD_OVERWRITE 7
#define R2_IDLE_STATE (R1_IDLE_STATE + 8)
#define R2_ERASE_RESET (R1_ERASE_RESET + 8)
#define R2_ILL_COMMAND (R1_ILL_COMMAND + 8)
#define R2_COM_CRC_ERR (R1_COM_CRC_ERR + 8)
#define R2_ERASE_SEQ_ERR (R1_ERASE_SEQ_ERR + 8)
#define R2_ADDR_ERR (R1_ADDR_ERR + 8)
#define R2_PARAM_ERR (R1_PARAM_ERR + 8)
/* R3: size 5 bytes */
#define R3_OCR_MASK (0xffffffffUL)
#define R3_IDLE_STATE (R1_IDLE_STATE + 32)
#define R3_ERASE_RESET (R1_ERASE_RESET + 32)
#define R3_ILL_COMMAND (R1_ILL_COMMAND + 32)
#define R3_COM_CRC_ERR (R1_COM_CRC_ERR + 32)
#define R3_ERASE_SEQ_ERR (R1_ERASE_SEQ_ERR + 32)
#define R3_ADDR_ERR (R1_ADDR_ERR + 32)
#define R3_PARAM_ERR (R1_PARAM_ERR + 32)
/* Data Response: size 1 byte */
#define DR_STATUS_MASK 0x0e
#define DR_STATUS_ACCEPTED 0x05
#define DR_STATUS_CRC_ERR 0x0a
#define DR_STATUS_WRITE_ERR 0x0c



#define ANZ_POT               6 // Anzahl zu lesender Potis

#define POT_FAKTOR            1.20 // Korrekturfaktor fuer Potentiometerstellung


#define MASTER_PORT            PORTD   //    
#define MASTER_DDR             DDRD    //    
#define MASTER_PIN             PIND    //
// PIN's
#define SUB_BUSY_PIN             5 // Ausgang fuer busy-Meldung des Sub an Master

#define INTERRUPT_PORT            PORTB   //
#define INTERRUPT_DDR             DDRB    //
#define INTERRUPT_PIN             PINB    //
// PIN's
#define MASTER_EN_PIN            7 // Eingang fur PinChange-Interrupt vom Master

#define SUB_EN_PORT              PORTE // Gate-Zugang zu EE und RAM fuer Memory-Zugriffe  des Sub
#define SUB_EN_DDR               DDRE  // mit RAM_CS_HI, EE_CS_HI
// PIN's
#define SUB_EN_PIN               0

#define USB_PORT            PORTD   //
#define USB_DDR             DDRD    //
#define USB_PIN             PIND    //
// PIN's
#define USB_DETECT_PIN      3

// Teensy2 int VCC Tastatur2

#define WERT1    19    // 1 oben  Taste 2
#define WERT3    49    // 2 links  Taste 4
#define WERT4    68    // 3 unten  Taste 8
#define WERT6    110   // 4 rechts  Taste 6
#define WERT9    215   // 5 Mitte  Taste 5
#define WERT2     30    //  A links oben Taste  1
#define WERT5    88       //    B links unten Taste 7
#define WERT7    139      //   C rechts oben Taste 3
#define WERT8    168      // D rechts unten Taste 9
#define WERT9    225      // Mitte rechts unten Taste 9




