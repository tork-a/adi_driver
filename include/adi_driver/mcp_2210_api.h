/*
© [2018] Microchip Technology Inc. and its subsidiaries.

Subject to your compliance with these terms, you may use Microchip software and 
any derivatives exclusively with Microchip products. It is your responsibility 
to comply with third party license terms applicable to your use of third party 
software (including open source software) that may accompany Microchip software.

THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER EXPRESS, 
IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES 
OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A PARTICULAR PURPOSE.

IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER 
RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF 
THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY 
LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO THIS 
SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY 
TO MICROCHIP FOR THIS SOFTWARE. 
*/
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <asm/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/hiddev.h>
#include <linux/input.h>
#include <time.h>
#include <ctype.h>
#include <getopt.h>
#include <string.h>
#include <errno.h>
#include <stdint.h>
#include <endian.h>

/* Important Macros */
#define cDEV_NOGPIOS				9

/* Typedefs */
typedef struct stDevChipSettings_S
{
    uint8_t	ucPinOption[cDEV_NOGPIOS]; /*every GPIO has 2 bits for options*/
    uint16_t	uiGpioOutputDefaultValue;
    uint16_t	uiGpioDirection;
    uint8_t	ucSpiFlags;
    uint8_t	ucNVRAMFlags;
} stDevChipSettings_T;

typedef struct stChipStatus_S
{
    uint8_t	ucSpiState;
    uint8_t     ucDataUsbTransmitted;
    uint8_t     ucDataUsbToXfer;
    uint16_t    uiDataTransferred;
    uint8_t     ucUsbCmdOngoing;
} stChipStatus_T;

typedef struct stSpiXferSettings_S
{
    uint32_t ulBaudRate;
    uint16_t uiIdleChipSelects;
    uint16_t uiActiveChipSelects;
    uint16_t uiCsToDataDly;
    uint16_t uiDataToCsDly;
    uint16_t uiDataToDataDly;
    uint16_t uiDataToXfer;
    uint8_t ucSpiModeFlags;
} stSpiXferSettings_T;

/* Library API */
int open_device(const char *device);

int spi_data_xfer(int filedesc, unsigned char *txdata,
                    unsigned char *rxdata, int xferlength,
                    int spimode, int speed,
                    int actcsval, int idlecsval, int gpcsmask,
                    int cs2datadly, int data2datadly, int data2csdly);

int read_eeprom(int filedesc, int address, unsigned char *readdata);
int write_eeprom(int filedesc, int address, unsigned char writedata);

int gpio_write(int filedesc, int gpioval, int gpiomask);
int gpio_read(int filedesc, int *pgpioval, int gpiomask);
int gpio_direction(int filedesc, int gpiodir, int gpiomask);
					
int close_device(int filedesc);

/*==========================================================*/
/* Other internal functions */
int get_chip_status(int filedesc, stChipStatus_T *pstchipstatus);
int get_spi_xfer_params(int filedesc, 
			stSpiXferSettings_T *pstspixfersettings);
int set_spi_xfer_params(int filedesc, 
			stSpiXferSettings_T *pstspixfersettings);
int get_crt_settings(int filedesc, stDevChipSettings_T *pstsettings);
int set_crt_settings(int filedesc, stDevChipSettings_T *pstsettings);
int xfer_spi_data(int filedesc, unsigned char *txdata, 
                    unsigned char *rxdata, uint8_t *pucdatalentx, 
                    uint8_t *pucdatalenrx, uint8_t *presultcode);
int gpio_setval(int filedesc, uint16_t gpvalue);
int gpio_getval(int filedesc, uint16_t *pgpvalue);
int gpio_setdir(int filedesc, uint16_t gpdir);
int gpio_getdir(int filedesc, uint16_t *pgpdir);
/*==========================================================*/
/* Helper functions */
void print_report_buffer(unsigned char *bufdata, int len, int rowlen);
/*==========================================================*/
/* Macros */
#define MCP2210_HID_REPORT_LEN					64
#define MCP2210_MAX_SPI_DATA_LEN				60
#define MCP2210_EEPROM_LEN                                      256
#define MCP2210_XFER_RETRIES                                    200

#define cSPI_D_IDLE                                             0x00
#define cSPI_D_GETSTATUS                                        0x10
#define cSPI_D_CANCELCMD                                        0x11
#define cSPI_D_READSETTINGS                                     0x20
#define cSPI_D_WRITESETTINGS                                    0x21
#define cSPI_D_SETGPIO                                          0x30
#define cSPI_D_GETGPIO                                          0x31
#define cSPI_D_SETGPIODIR                                       0x32
#define cSPI_D_GETGPIODIR                                       0x33
#define cSPI_D_SETSPITRANSACTION                                0x40
#define cSPI_D_GETSPITRANSACTION                                0x41
#define	cSPI_D_XFERSPIDATA                                      0x42
#define cSPI_D_READEEPROM                                       0x50
#define cSPI_D_WRITEEEPROM                                      0x51
#define cSPI_D_SETNVRAMPARAM                                    0x60
#define cSPI_D_GETNVRAMPARAM                                    0x61

/* MCP2210 return codes */
#define cCHIP_NOERR                                             0x00

#define cCHIP_SPI_XFER_RXDATAEND                                0x10
#define cCHIP_SPI_XFER_NORXDATA                                 0x20
#define cCHIP_SPI_XFER_RXDATA                                   0x30
#define cCHIP_SPI_XFERONGOING                                   0xF8
#define cCHIP_SPI_BUSHIZ                                        0xF7
#define cCHIP_BLOCKEDACCESS                                     0xFB

/* API return codes */
#define ERR_NOERR                                               0
#define ERR_WRERR                                               10
#define ERR_RDERR                                               20
#define ERR_HWERR                                               30
#define ERR_GETCHIPSTATUS                                       100
#define ERR_GETSETTINGS                                         110
#define ERR_SETSETTINGS                                         120
#define ERR_GETSPISETTINGS                                      130
#define ERR_SETSPISETTINGS                                      140
#define ERR_ADDROUTOFRANGE                                      150
#define ERR_BLOCKEDACCESS                                       160
#define ERR_WRITEGPIO                                           170
#define ERR_READGPIO                                            180
#define ERR_SETGPIODIR                                          190

/* MCP2210 pin designation */
#define cDEV_PIN_GPIO                 			0x00
#define cDEV_PIN_ALTFUNC_1            			0x01 /* chip select */
#define cDEV_PIN_ALTFUNC_2            			0x02 /* leds, other */
