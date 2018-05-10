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
#include <mcp2210_api.h>

int open_device(const char *device)
{
    int fd;
    /*use udev or other similar mechanisms to get the system path "/dev/hidraw1" */
    fd = open(device, O_RDWR);

    if (fd < 0) {
        perror("\n\tUnable to open device");
    }
    return fd;
}

int close_device(int filedesc)
{
    return close(filedesc);
}

int spi_data_xfer(int filedesc, unsigned char *txdata,
					unsigned char *rxdata, int xferlength,
					int spimode, int speed,
					int actcsval, int idlecsval, int gpcsmask,
					int cs2datadly, int data2datadly, int data2csdly)
{
    stDevChipSettings_T stsettings;
    stChipStatus_T stchipstatus;
    stSpiXferSettings_T stspixfersettings;
    int idx, ires, inumxfers, idataxferrem, ixfertimeout, retries, itmp;
    int idatatxidx, idatarxidx;
    uint8_t spixferaddrescode, forcedexit;
    uint8_t xferpartlentx, xferpartlenrx;

    /* get the current status of the chip */
    ires = get_chip_status(filedesc, &stchipstatus);
    if (ires != ERR_NOERR)
    {
        return -ERR_GETCHIPSTATUS;
    }

    /* get the current chip settings */
    ires = get_crt_settings(filedesc, &stsettings);
    if (ires != ERR_NOERR)
    {
        return -ERR_GETSETTINGS;
    }

    /* assign the needed GPs for CS operation */
    for(idx = 0; idx < cDEV_NOGPIOS; idx++)
    {
        if (gpcsmask & (1 << idx))
        {/* the current pin will be assigned for CS operation*/
            stsettings.ucPinOption[idx] = cDEV_PIN_ALTFUNC_1;
        }
    }

    /* set the current chip settings */
    ires = set_crt_settings(filedesc, &stsettings);
    if (ires != ERR_NOERR)
    {
        return -ERR_SETSETTINGS;
    }

    /* fill-in the required SPI xfer parameters */
    stspixfersettings.ulBaudRate                = (uint32_t)speed;
    stspixfersettings.ucSpiModeFlags            = spimode;
    stspixfersettings.uiDataToXfer              = (uint16_t)xferlength;
    stspixfersettings.uiIdleChipSelects         = (uint16_t)idlecsval;
    stspixfersettings.uiActiveChipSelects       = (uint16_t)actcsval;
    stspixfersettings.uiCsToDataDly             = (uint16_t)cs2datadly;
    stspixfersettings.uiDataToDataDly           = (uint16_t)data2datadly;
    stspixfersettings.uiDataToCsDly             = (uint16_t)data2csdly;

    /* set the required SPI xfer parameters */
    ires = set_spi_xfer_params(filedesc, &stspixfersettings);
    if (ires != ERR_NOERR)
    {
        /* get the current status of the chip */
        itmp = get_chip_status(filedesc, &stchipstatus);
        if (itmp != ERR_NOERR)
        {
            return -ERR_GETCHIPSTATUS;
        }
        return -ERR_SETSPISETTINGS;
    }

    /* !!!OPTIONAL - get back the actual chip settings */
    ires = get_spi_xfer_params(filedesc, &stspixfersettings);
    if (ires != ERR_NOERR)
    {
        return -ERR_GETSPISETTINGS;
    }

    /* transfer the actual SPI data */
    /* establish the key parameters - timeout */
    ixfertimeout    = (100000/stspixfersettings.ulBaudRate) + 1;
    idataxferrem    = xferlength;
    if ((xferlength % MCP2210_MAX_SPI_DATA_LEN) == 0)
    {/* exact multiple of MCP2210_MAX_SPI_DATA_LEN (60 bytes) to xfer */
        inumxfers       = (xferlength / MCP2210_MAX_SPI_DATA_LEN);
    }
    else
    {/* we need to add an extra transfer if the data length is not a */
        /* multiple of MCP2210_MAX_SPI_DATA_LEN or 60 bytes */
        inumxfers       = (xferlength / MCP2210_MAX_SPI_DATA_LEN) + 1;
    }
    idatatxidx      = 0;
    idatarxidx      = 0;
    forcedexit      = 0;
    retries         = 0;
    for(idx = 0; idx < inumxfers; idx++)
    {
        /* send the data to the SPI engine */
        while((retries < MCP2210_XFER_RETRIES) && (forcedexit != 1))
        {
            if (retries == 0)
            {
                /* prepare the data chunk to be sent out */
                if (idataxferrem <= MCP2210_MAX_SPI_DATA_LEN)
                {/* less than 60 bytes to transfer */
                    xferpartlentx = (uint8_t)idataxferrem;
                    idataxferrem = 0;
                }
                else
                {
                    xferpartlentx = MCP2210_MAX_SPI_DATA_LEN;
                    idataxferrem -= MCP2210_MAX_SPI_DATA_LEN;
                }
            }
            
            ires = xfer_spi_data(filedesc, &txdata[idatatxidx], &rxdata[idatarxidx], 
                                &xferpartlentx, &xferpartlenrx, &spixferaddrescode);
            if (ires < 0) /* in case of an error */
            {
                printf("\n\n\n\nSerious HW Error!!!\n\n\n\n");
                printf("\nspixferaddrescode \t= 0x%02X", spixferaddrescode);
                printf("\nxferpartlentx \t\t= %d", xferpartlentx);
                printf("\nxferpartlenrx \t\t= %d", xferpartlenrx);
                printf("\nidatatxidx \t\t= %d", idatatxidx);
                printf("\nidatarxidx \t\t= %d", idatarxidx);
                printf("\nretries \t\t= %d", retries);
                printf("\ninumxfers \t\t= %d", inumxfers);
                printf("\nidataxferrem \t\t= %d", idataxferrem);
                printf("\n\n\n\n");
                exit(1);
                /* get the current status of the chip */
                itmp = get_chip_status(filedesc, &stchipstatus);
                if (itmp != ERR_NOERR)
                {
                    return -ERR_GETCHIPSTATUS;
                }
                return ires;
            }
            
            switch(spixferaddrescode)
            {
                case cCHIP_SPI_XFER_NORXDATA:
                {/* the data has been transmitted but no rx data yet available */
                    if (retries == 0)
                    {
                        idatatxidx += xferpartlentx;
                        if (idataxferrem <= MCP2210_MAX_SPI_DATA_LEN)
                        {/* less than 60 bytes to transfer */
                            xferpartlentx = (uint8_t)idataxferrem;
                            idataxferrem = 0;
                        }
                        else
                        {
                            xferpartlentx = MCP2210_MAX_SPI_DATA_LEN;
                            idataxferrem -= MCP2210_MAX_SPI_DATA_LEN;
                        }
                    }
                    /* allow the SPI to transmit its data */
                    usleep(ixfertimeout * 1000);
                }
                break;
                
                case cCHIP_SPI_XFER_RXDATA:
                case cCHIP_SPI_XFER_RXDATAEND:
                {/* rx data is available */
                    idatarxidx += xferpartlenrx;
                    if (spixferaddrescode == cCHIP_SPI_XFER_RXDATA)
                    {
                        idatatxidx += xferpartlentx;
                        if (idataxferrem <= MCP2210_MAX_SPI_DATA_LEN)
                        {/* less than 60 bytes to transfer */
                            xferpartlentx = (uint8_t)idataxferrem;
                            idataxferrem = 0;
                        }
                        else
                        {
                            xferpartlentx = MCP2210_MAX_SPI_DATA_LEN;
                            idataxferrem -= MCP2210_MAX_SPI_DATA_LEN;
                        }
                        /* increase the number of transfers because we're */
                        /* jumping to the next one */
                        idx++;
                    }
                    else
                    {
                        forcedexit = 1;
                    }
                }
                break;
                
                case cCHIP_SPI_XFERONGOING:
                {/* do nothing and loop again - chip is still busy */
                }
                break;
            }
            retries++;
        }
        retries     = 0;
        forcedexit  = 0;
    }
    
    return ERR_NOERR;
}
/*==========================================================*/
int xfer_spi_data(int filedesc, unsigned char *txdata, 
                    unsigned char *rxdata, uint8_t *pucdatalentx, 
                    uint8_t *pucdatalenrx, uint8_t *presultcode)
{
    unsigned char buftx[MCP2210_HID_REPORT_LEN];
    unsigned char bufrx[MCP2210_HID_REPORT_LEN];
    int ires;
    
    /* copy the user given data */
    memcpy(&buftx[4], txdata, *pucdatalentx);
    if (*pucdatalentx > MCP2210_MAX_SPI_DATA_LEN)
    {
        buftx[1] = MCP2210_MAX_SPI_DATA_LEN;
    }
    else
    {
        buftx[1] = *pucdatalentx;
    }
    
    buftx[0] = cSPI_D_XFERSPIDATA;
    /* Send a Report to the Device */
    ires = write(filedesc, buftx, MCP2210_HID_REPORT_LEN);
    if (ires < 0) {
#ifdef DEBUG_INFO
        printf("\nError: %d", errno);
        perror("\n\txfer_spi_data() -> write");
#endif
        return -ERR_WRERR;
    } else {
#ifdef DEBUG_INFO
        printf("\n\txfer_spi_data()->write() wrote %d bytes", ires);
#endif
    }

#ifdef DEBUG_INFO
    print_report_buffer(buftx, MCP2210_HID_REPORT_LEN, 10);
#endif
    /*usleep(100 * 1000);*/

    /* Get a report from the device */
    ires = read(filedesc, bufrx, MCP2210_HID_REPORT_LEN);
    if (ires < 0) {
#ifdef DEBUG_INFO
        printf("\nError: %d", errno);
        perror("\n\txfer_spi_data() -> read");
#endif
        return -ERR_RDERR;
    } else {
#ifdef DEBUG_INFO
        printf("\n\txfer_spi_data() -> read() read %d bytes:", ires);
#endif
    }
    
#ifdef DEBUG_INFO
    print_report_buffer(bufrx, MCP2210_HID_REPORT_LEN, 10);
#endif
    if (bufrx[1] != cCHIP_NOERR)
    {
        if (bufrx[1] == cCHIP_SPI_XFERONGOING)
        {/* SPI engine is busy - try again later */
            *presultcode    = cCHIP_SPI_XFERONGOING;
            *pucdatalenrx   = 0;
            return ERR_NOERR;
        }
#ifdef DEBUG_INFO
        printf("\nxfer_spi_data() - CMD status ERROR");
#endif
        return -ERR_HWERR;
    }
    /* copy the additional result code and the user data */
    *presultcode    = bufrx[3];
    *pucdatalenrx   = bufrx[2];
    if ((bufrx[3] == cCHIP_SPI_XFER_RXDATAEND) || 
            (bufrx[3] == cCHIP_SPI_XFER_RXDATA))
    {/* bufrx[2] tells how many bytes are available to read */
        memcpy(rxdata, &bufrx[4], bufrx[2]);
    }

    return ERR_NOERR;
}
/*==========================================================*/
int get_chip_status(int filedesc, stChipStatus_T *pstchipstatus)
{
    unsigned char buftx[MCP2210_HID_REPORT_LEN];
    unsigned char bufrx[MCP2210_HID_REPORT_LEN];
    int ires;
    uint16_t *ptmpvar;
    
    buftx[0] = cSPI_D_GETSTATUS;
    /* Send a Report to the Device */
    ires = write(filedesc, buftx, MCP2210_HID_REPORT_LEN);
    if (ires < 0) {
#ifdef DEBUG_INFO
        printf("\nError: %d", errno);
        perror("\n\tget_chip_status() -> write");
#endif
        return -ERR_WRERR;
    } else {
#ifdef DEBUG_INFO
        printf("\n\tget_chip_status()->write() wrote %d bytes", ires);
#endif
    }

#ifdef DEBUG_INFO
    print_report_buffer(buftx, MCP2210_HID_REPORT_LEN, 10);
#endif
    /*usleep(100 * 1000);*/

    /* Get a report from the device */
    ires = read(filedesc, bufrx, MCP2210_HID_REPORT_LEN);
    if (ires < 0) {
#ifdef DEBUG_INFO
        printf("\nError: %d", errno);
        perror("\n\tget_chip_status() -> read");
#endif
        return -ERR_RDERR;
    } else {
#ifdef DEBUG_INFO
        printf("\n\tget_chip_status() -> read() read %d bytes:", ires);
#endif
    }
    
#ifdef DEBUG_INFO
    print_report_buffer(bufrx, MCP2210_HID_REPORT_LEN, 10);
#endif
    if (bufrx[1] != cCHIP_NOERR)
    {
#ifdef DEBUG_INFO
        printf("\nget_chip_status() - CMD status ERROR");
#endif
        return -ERR_HWERR;
    }
    
    /* now, copy the fields we're interested in */
    pstchipstatus->ucDataUsbTransmitted             = bufrx[6];
    pstchipstatus->ucDataUsbToXfer                  = bufrx[7];
    ptmpvar                                         = (uint16_t *)&bufrx[8];
    pstchipstatus->uiDataTransferred                = le16toh(*ptmpvar);
    pstchipstatus->ucUsbCmdOngoing                  = bufrx[10];
    pstchipstatus->ucSpiState                       = bufrx[11];
    
#ifdef DEBUG_INFO
    puts("\n");
    printf("\nget_chip_status(): Results");
    printf("\n\tSPI Data to Xfer(total): %d bytes", pstchipstatus->ucDataUsbToXfer);
    printf("\n\tSPI Data Xferred(buf): %d bytes", pstchipstatus->ucDataUsbTransmitted);
    printf("\n\tSPI Data to Xfer(buf): 0%d bytes", pstchipstatus->ucDataUsbToXfer);
    printf("\n\tUSB-SPI Ongoing Xfer: 0x%02X", pstchipstatus->ucUsbCmdOngoing);
    printf("\n\tSPI Engine Status: 0x%02X", pstchipstatus->ucSpiState);
    puts("\n");
#endif
    return ERR_NOERR;
}
/*==========================================================*/
int get_spi_xfer_params(int filedesc, stSpiXferSettings_T *pstspixfersettings)
{
    unsigned char buftx[MCP2210_HID_REPORT_LEN];
    unsigned char bufrx[MCP2210_HID_REPORT_LEN];
    int ires;
    uint16_t *puintvar;
    uint32_t *pulongvar;
    
    buftx[0] = cSPI_D_GETSPITRANSACTION;
    /* Send a Report to the Device */
    ires = write(filedesc, buftx, MCP2210_HID_REPORT_LEN);
    if (ires < 0) {
#ifdef DEBUG_INFO
        printf("\nError: %d", errno);
        perror("\n\tget_spi_xfer_params() -> write");
#endif
        return -ERR_WRERR;
    } else {
#ifdef DEBUG_INFO
        printf("\n\tget_spi_xfer_params()->write() wrote %d bytes", ires);
#endif
    }

#ifdef DEBUG_INFO
    print_report_buffer(buftx, MCP2210_HID_REPORT_LEN, 10);
#endif
    /*usleep(100 * 1000);*/

    /* Get a report from the device */
    ires = read(filedesc, bufrx, MCP2210_HID_REPORT_LEN);
    if (ires < 0) {
#ifdef DEBUG_INFO
        printf("\nError: %d", errno);
        perror("\n\tget_spi_xfer_params() -> read");
#endif
        return -ERR_RDERR;
    } else {
#ifdef DEBUG_INFO
        printf("\n\tget_spi_xfer_params() -> read() read %d bytes:", ires);
#endif
    }
    
#ifdef DEBUG_INFO
    print_report_buffer(bufrx, MCP2210_HID_REPORT_LEN, 10);
#endif
    if (bufrx[1] != cCHIP_NOERR)
    {
#ifdef DEBUG_INFO
        printf("\nget_spi_xfer_params() - CMD status ERROR");
#endif
        return -ERR_HWERR;
    }
    
    /* get the SPI parameters into the SPI params structure */
    pulongvar                                   = (uint32_t *)&bufrx[4];
    pstspixfersettings->ulBaudRate              = le32toh(*pulongvar);
    puintvar                                    = (uint16_t *)&bufrx[8];
    pstspixfersettings->uiIdleChipSelects       = le16toh(*puintvar);
    puintvar                                    = (uint16_t *)&bufrx[10];
    pstspixfersettings->uiActiveChipSelects     = le16toh(*puintvar);
    puintvar                                    = (uint16_t *)&bufrx[12];
    pstspixfersettings->uiCsToDataDly           = le16toh(*puintvar);
    puintvar                                    = (uint16_t *)&bufrx[14];
    pstspixfersettings->uiDataToCsDly           = le16toh(*puintvar);
    puintvar                                    = (uint16_t *)&bufrx[16];
    pstspixfersettings->uiDataToDataDly         = le16toh(*puintvar);
    puintvar                                    = (uint16_t *)&bufrx[18];
    pstspixfersettings->uiDataToXfer            = le16toh(*puintvar);
    pstspixfersettings->ucSpiModeFlags          = bufrx[20];
    
#ifdef DEBUG_INFO
    puts("\n");
    printf("\nget_spi_xfer_params(): Results");
    printf("\n\tSPI Mode: %d", pstspixfersettings->ucSpiModeFlags);
    printf("\n\tSPI Data to Xfer: %d bytes", pstspixfersettings->uiDataToXfer);
    printf("\n\tSPI Speed: %d bps", pstspixfersettings->ulBaudRate);
    printf("\n\tSPI Idle CS(val): 0x%04X", pstspixfersettings->uiIdleChipSelects);
    printf("\n\tSPI Active CS(val): 0x%04X", pstspixfersettings->uiActiveChipSelects);
    printf("\n\tSPI CS-to-Data delay: %d", pstspixfersettings->uiCsToDataDly);
    printf("\n\tSPI Data-to-Data delay: %d", pstspixfersettings->uiDataToDataDly);
    printf("\n\tSPI Data-to-CS delay: %d", pstspixfersettings->uiDataToCsDly);
    puts("\n");
#endif
    return ERR_NOERR;
}
/*==========================================================*/
int set_spi_xfer_params(int filedesc, stSpiXferSettings_T *pstspixfersettings)
{
    unsigned char buftx[MCP2210_HID_REPORT_LEN];
    unsigned char bufrx[MCP2210_HID_REPORT_LEN];
    int ires;
    
    /* Put the needed settings into the right locations */
    buftx[4] = (uint8_t)(pstspixfersettings->ulBaudRate & 0xFF);
    buftx[5] = (uint8_t)((pstspixfersettings->ulBaudRate & 0xFF00) >> 8);
    buftx[6] = (uint8_t)((pstspixfersettings->ulBaudRate & 0xFF0000) >> 16);
    buftx[7] = (uint8_t)((pstspixfersettings->ulBaudRate & 0xFF000000) >> 24);
    
    buftx[8] = (uint8_t)(pstspixfersettings->uiIdleChipSelects & 0xFF);
    buftx[9] = (uint8_t)((pstspixfersettings->uiIdleChipSelects & 0xFF00) >> 8);
    
    buftx[10] = (uint8_t)(pstspixfersettings->uiActiveChipSelects & 0xFF);
    buftx[11] = (uint8_t)((pstspixfersettings->uiActiveChipSelects & 0xFF00) >> 8);
    
    buftx[12] = (uint8_t)(pstspixfersettings->uiCsToDataDly & 0xFF);
    buftx[13] = (uint8_t)((pstspixfersettings->uiCsToDataDly & 0xFF00) >> 8);
    
    buftx[14] = (uint8_t)(pstspixfersettings->uiDataToCsDly & 0xFF);
    buftx[15] = (uint8_t)((pstspixfersettings->uiDataToCsDly & 0xFF00) >> 8);
    
    buftx[16] = (uint8_t)(pstspixfersettings->uiDataToDataDly & 0xFF);
    buftx[17] = (uint8_t)((pstspixfersettings->uiDataToDataDly & 0xFF00) >> 8);
    
    buftx[18] = (uint8_t)(pstspixfersettings->uiDataToXfer & 0xFF);
    buftx[19] = (uint8_t)((pstspixfersettings->uiDataToXfer & 0xFF00) >> 8);
    
    buftx[20] = pstspixfersettings->ucSpiModeFlags;
    
    buftx[0] = cSPI_D_SETSPITRANSACTION;
    buftx[1] = buftx[2] = buftx[3] = 0;
    /* Send a Report to the Device */
    ires = write(filedesc, buftx, MCP2210_HID_REPORT_LEN);
    if (ires < 0) {
#ifdef DEBUG_INFO
        printf("\nError: %d", errno);
        perror("\n\tset_spi_xfer_params() -> write");
#endif
        return -ERR_WRERR;
    } else {
#ifdef DEBUG_INFO
        printf("\n\tset_spi_xfer_params() -> write() wrote %d bytes", ires);
#endif
    }

#ifdef DEBUG_INFO
    print_report_buffer(buftx, MCP2210_HID_REPORT_LEN, 10);
#endif
    /*usleep(100 * 1000);*/

    /* Get a report from the device */
    ires = read(filedesc, bufrx, MCP2210_HID_REPORT_LEN);
    if (ires < 0) {
#ifdef DEBUG_INFO
        printf("\nError: %d", errno);
        perror("\n\tset_spi_xfer_params() -> read");
#endif
        return -ERR_RDERR;
    } else {
#ifdef DEBUG_INFO
        printf("\n\tset_spi_xfer_params() -> read() read %d bytes:", ires);
#endif
    }
#ifdef DEBUG_INFO
    print_report_buffer(bufrx, MCP2210_HID_REPORT_LEN, 10);
#endif
    if (bufrx[1] != cCHIP_NOERR)
    {
#ifdef DEBUG_INFO
        printf("\nset_spi_xfer_params() - CMD status ERROR");
#endif
        return -ERR_HWERR;
    }
    
    return ERR_NOERR;
}
/*==========================================================*/
int set_crt_settings(int filedesc, stDevChipSettings_T *pstsettings)
{
    unsigned char buftx[MCP2210_HID_REPORT_LEN];
    unsigned char bufrx[MCP2210_HID_REPORT_LEN];
    int ires, idx;
    
    /* Put the needed settings into the right locations */
    for(idx = 0; idx < cDEV_NOGPIOS; idx++)
    {/* copy the new GP designations */
        buftx[4 + idx] = pstsettings->ucPinOption[idx];
    }
    buftx[13] = (uint8_t)(pstsettings->uiGpioOutputDefaultValue & 0xFF);
    buftx[14] = (uint8_t)((pstsettings->uiGpioOutputDefaultValue & 0xFF00) >> 8);
    buftx[15] = (uint8_t)(pstsettings->uiGpioDirection & 0xFF);
    buftx[16] = (uint8_t)((pstsettings->uiGpioDirection & 0xFF00) >> 8);
    buftx[17] = pstsettings->ucSpiFlags;
    
    buftx[0] = cSPI_D_WRITESETTINGS;
    buftx[1] = buftx[2] = buftx[3] = 0;
    /* Send a Report to the Device */
    ires = write(filedesc, buftx, MCP2210_HID_REPORT_LEN);
    if (ires < 0) {
#ifdef DEBUG_INFO
        printf("\nError: %d", errno);
        perror("\n\tset_crt_settings() -> write");
#endif
        return -ERR_WRERR;
    } else {
#ifdef DEBUG_INFO
        printf("\n\tset_crt_settings() -> write() wrote %d bytes", ires);
#endif
    }

#ifdef DEBUG_INFO
    print_report_buffer(buftx, MCP2210_HID_REPORT_LEN, 10);
#endif
    /*usleep(100 * 1000);*/

    /* Get a report from the device */
    ires = read(filedesc, bufrx, MCP2210_HID_REPORT_LEN);
    if (ires < 0) {
#ifdef DEBUG_INFO
        printf("\nError: %d", errno);
        perror("\n\tset_crt_settings() -> read");
#endif
        return -ERR_RDERR;
    } else {
#ifdef DEBUG_INFO
        printf("\n\tset_crt_settings() -> read() read %d bytes:", ires);
#endif
    }
#ifdef DEBUG_INFO
    print_report_buffer(bufrx, MCP2210_HID_REPORT_LEN, 10);
#endif
    if (bufrx[1] != cCHIP_NOERR)
    {
#ifdef DEBUG_INFO
        printf("\nset_crt_settings() - CMD status ERROR");
#endif
        return -ERR_HWERR;
    }
    
    return ERR_NOERR;
}
/*==========================================================*/
int get_crt_settings(int filedesc, stDevChipSettings_T *pstsettings)
{
    unsigned char buftx[MCP2210_HID_REPORT_LEN];
    unsigned char bufrx[MCP2210_HID_REPORT_LEN];
    int ires;
    uint16_t *ptmpvar;

    buftx[0] = cSPI_D_READSETTINGS;
    /* Send a Report to the Device */
    ires = write(filedesc, buftx, MCP2210_HID_REPORT_LEN);
    if (ires < 0) {
#ifdef DEBUG_INFO
        printf("\nError: %d", errno);
        perror("\n\tget_crt_settings() -> write");
#endif
        return -ERR_WRERR;
    } else {
#ifdef DEBUG_INFO
        printf("\n\tget_crt_settings() -> write() wrote %d bytes", ires);
#endif
    }

#ifdef DEBUG_INFO
    print_report_buffer(buftx, MCP2210_HID_REPORT_LEN, 10);
#endif
    /*usleep(100 * 1000);*/

    /* Get a report from the device */
    ires = read(filedesc, bufrx, MCP2210_HID_REPORT_LEN);
    if (ires < 0) {
#ifdef DEBUG_INFO
        printf("\nError: %d", errno);
        perror("\n\tget_crt_settings() -> read");
#endif
        return -ERR_RDERR;
    } else {
#ifdef DEBUG_INFO
        printf("\n\tget_crt_settings() -> read() read %d bytes:", ires);
#endif
    }
#ifdef DEBUG_INFO
    print_report_buffer(bufrx, MCP2210_HID_REPORT_LEN, 10);
#endif
    if (bufrx[1] != cCHIP_NOERR)
    {
#ifdef DEBUG_INFO
        printf("\nget_crt_settings() - CMD status ERROR");
#endif
        return -ERR_HWERR;
    }
    /* copy the chip settings into the structure */
    /* MCP2210 uses little endian */
    /* should work on big endian systems */
    memcpy((void *)&pstsettings->ucPinOption, &bufrx[4], cDEV_NOGPIOS);
    ptmpvar                     = (uint16_t *)&bufrx[4 + cDEV_NOGPIOS + 0];
    pstsettings->uiGpioOutputDefaultValue
                                = le16toh(*ptmpvar);
    ptmpvar                     = (uint16_t *)&bufrx[4 + cDEV_NOGPIOS + 2];					
    pstsettings->uiGpioDirection
                                = le16toh(*ptmpvar);
    pstsettings->ucSpiFlags     = bufrx[4 + cDEV_NOGPIOS + 4];
    pstsettings->ucNVRAMFlags	= bufrx[4 + cDEV_NOGPIOS + 5];
#ifdef DEBUG_INFO
    puts("\n");
    printf("\nget_crt_settings(): Results");
    printf("\nGPIO Designation:");
    print_report_buffer((unsigned char *)&pstsettings->ucPinOption, 
                                                    cDEV_NOGPIOS, cDEV_NOGPIOS);
    printf("\nGPIO Output Default Value: 0x%04X", 
                    pstsettings->uiGpioOutputDefaultValue);
    printf("\nGPIO Output Direction: 0x%04X", 
                    pstsettings->uiGpioDirection);
    printf("\nSPI Flags: 0x%02X", pstsettings->ucSpiFlags);
    printf("\nNVRAM Flags: 0x%02X", pstsettings->ucNVRAMFlags);
    puts("\n");
#endif
    return ERR_NOERR;
}
/*==========================================================*/
int read_eeprom(int filedesc, int address, unsigned char *readdata)
{
    unsigned char buftx[MCP2210_HID_REPORT_LEN];
    unsigned char bufrx[MCP2210_HID_REPORT_LEN];
    int ires;
    
    if ((address >= MCP2210_EEPROM_LEN) || (address < 0))
    {
        return -ERR_ADDROUTOFRANGE;
    }

    buftx[0] = cSPI_D_READEEPROM;
    buftx[1] = (uint8_t)address;
    
    /* Send a Report to the Device */
    ires = write(filedesc, buftx, MCP2210_HID_REPORT_LEN);
    if (ires < 0) {
#ifdef DEBUG_INFO
        printf("\nError: %d", errno);
        perror("\n\tread_eeprom() -> write");
#endif
        return -ERR_WRERR;
    } else {
#ifdef DEBUG_INFO
        printf("\n\tread_eeprom() -> write() wrote %d bytes", ires);
#endif
    }

#ifdef DEBUG_INFO
    print_report_buffer(buftx, MCP2210_HID_REPORT_LEN, 10);
#endif
    /*usleep(100 * 1000);*/

    /* Get a report from the device */
    ires = read(filedesc, bufrx, MCP2210_HID_REPORT_LEN);
    if (ires < 0) {
#ifdef DEBUG_INFO
        printf("\nError: %d", errno);
        perror("\n\tread_eeprom() -> read");
#endif
        return -ERR_RDERR;
    } else {
#ifdef DEBUG_INFO
        printf("\n\tread_eeprom() -> read() read %d bytes:", ires);
#endif
    }
#ifdef DEBUG_INFO
    print_report_buffer(bufrx, MCP2210_HID_REPORT_LEN, 10);
#endif
    if (bufrx[1] != cCHIP_NOERR)
    {
#ifdef DEBUG_INFO
        printf("\nread_eeprom() - CMD status ERROR");
#endif
        return -ERR_HWERR;
    }
    /* copy the read result */
    *readdata = bufrx[3];
    return ERR_NOERR;
}
/*==========================================================*/
int write_eeprom(int filedesc, int address, unsigned char writedata)
{
    unsigned char buftx[MCP2210_HID_REPORT_LEN];
    unsigned char bufrx[MCP2210_HID_REPORT_LEN];
    int ires;
    
    if ((address >= MCP2210_EEPROM_LEN) || (address < 0))
    {
        return -ERR_ADDROUTOFRANGE;
    }

    buftx[0] = cSPI_D_WRITEEEPROM;
    buftx[1] = (uint8_t)address;
    buftx[2] = (uint8_t)writedata;
    
    /* Send a Report to the Device */
    ires = write(filedesc, buftx, MCP2210_HID_REPORT_LEN);
    if (ires < 0) {
#ifdef DEBUG_INFO
        printf("\nError: %d", errno);
        perror("\n\twrite_eeprom() -> write");
#endif
        return -ERR_WRERR;
    } else {
#ifdef DEBUG_INFO
        printf("\n\twrite_eeprom() -> write() wrote %d bytes", ires);
#endif
    }

#ifdef DEBUG_INFO
    print_report_buffer(buftx, MCP2210_HID_REPORT_LEN, 10);
#endif
    /*usleep(100 * 1000);*/

    /* Get a report from the device */
    ires = read(filedesc, bufrx, MCP2210_HID_REPORT_LEN);
    if (ires < 0) {
#ifdef DEBUG_INFO
        printf("\nError: %d", errno);
        perror("\n\twrite_eeprom() -> read");
#endif
        return -ERR_RDERR;
    } else {
#ifdef DEBUG_INFO
        printf("\n\twrite_eeprom() -> read() read %d bytes:", ires);
#endif
    }
#ifdef DEBUG_INFO
    print_report_buffer(bufrx, MCP2210_HID_REPORT_LEN, 10);
#endif
    if (bufrx[1] != cCHIP_NOERR)
    {
        if (bufrx[1] == cCHIP_BLOCKEDACCESS)
        {
#ifdef DEBUG_INFO
            printf("\nwrite_eeprom() - CMD status ERROR");
#endif
            return -ERR_BLOCKEDACCESS;
        }
#ifdef DEBUG_INFO
        printf("\nwrite_eeprom() - CMD status ERROR");
#endif
        return -ERR_HWERR;
    }
    
    return ERR_NOERR;
}
/*==========================================================*/
int gpio_setval(int filedesc, uint16_t gpvalue)
{
    unsigned char buftx[MCP2210_HID_REPORT_LEN];
    unsigned char bufrx[MCP2210_HID_REPORT_LEN];
    int ires;

    buftx[0] = cSPI_D_SETGPIO;
    buftx[4] = (uint8_t)(gpvalue & 0xFF);
    buftx[5] = (uint8_t)((gpvalue & 0xFF00) >> 8);
    
    /* Send a Report to the Device */
    ires = write(filedesc, buftx, MCP2210_HID_REPORT_LEN);
    if (ires < 0) {
#ifdef DEBUG_INFO
        printf("\nError: %d", errno);
        perror("\n\tgpio_write() -> write");
#endif
        return -ERR_WRERR;
    } else {
#ifdef DEBUG_INFO
        printf("\n\tgpio_write() -> write() wrote %d bytes", ires);
#endif
    }

#ifdef DEBUG_INFO
    print_report_buffer(buftx, MCP2210_HID_REPORT_LEN, 10);
#endif
    /*usleep(100 * 1000);*/

    /* Get a report from the device */
    ires = read(filedesc, bufrx, MCP2210_HID_REPORT_LEN);
    if (ires < 0) {
#ifdef DEBUG_INFO
        printf("\nError: %d", errno);
        perror("\n\tgpio_write() -> read");
#endif
        return -ERR_RDERR;
    } else {
#ifdef DEBUG_INFO
        printf("\n\tgpio_write() -> read() read %d bytes:", ires);
#endif
    }
#ifdef DEBUG_INFO
    print_report_buffer(bufrx, MCP2210_HID_REPORT_LEN, 10);
#endif
    if (bufrx[1] != cCHIP_NOERR)
    {
        if (bufrx[1] == cCHIP_BLOCKEDACCESS)
        {
#ifdef DEBUG_INFO
            printf("\ngpio_write() - CMD status ERROR");
#endif
            return -ERR_BLOCKEDACCESS;
        }
#ifdef DEBUG_INFO
        printf("\ngpio_write() - CMD status ERROR");
#endif
        return -ERR_HWERR;
    }
    
    return ERR_NOERR;
}
/*==========================================================*/
int gpio_getval(int filedesc, uint16_t *pgpvalue)
{
    unsigned char buftx[MCP2210_HID_REPORT_LEN];
    unsigned char bufrx[MCP2210_HID_REPORT_LEN];
    int ires;
    uint16_t *ptmpvar;

    buftx[0] = cSPI_D_GETGPIO;
    
    /* Send a Report to the Device */
    ires = write(filedesc, buftx, MCP2210_HID_REPORT_LEN);
    if (ires < 0) {
#ifdef DEBUG_INFO
        printf("\nError: %d", errno);
        perror("\n\twrite_eeprom() -> write");
#endif
        return -ERR_WRERR;
    } else {
#ifdef DEBUG_INFO
        printf("\n\twrite_eeprom() -> write() wrote %d bytes", ires);
#endif
    }

#ifdef DEBUG_INFO
    print_report_buffer(buftx, MCP2210_HID_REPORT_LEN, 10);
#endif
    /*usleep(100 * 1000);*/

    /* Get a report from the device */
    ires = read(filedesc, bufrx, MCP2210_HID_REPORT_LEN);
    if (ires < 0) {
#ifdef DEBUG_INFO
        printf("\nError: %d", errno);
        perror("\n\twrite_eeprom() -> read");
#endif
        return -ERR_RDERR;
    } else {
#ifdef DEBUG_INFO
        printf("\n\twrite_eeprom() -> read() read %d bytes:", ires);
#endif
    }
#ifdef DEBUG_INFO
    print_report_buffer(bufrx, MCP2210_HID_REPORT_LEN, 10);
#endif
    if (bufrx[1] != cCHIP_NOERR)
    {
        if (bufrx[1] == cCHIP_BLOCKEDACCESS)
        {
#ifdef DEBUG_INFO
            printf("\nwrite_eeprom() - CMD status ERROR");
#endif
            return -ERR_BLOCKEDACCESS;
        }
#ifdef DEBUG_INFO
        printf("\nwrite_eeprom() - CMD status ERROR");
#endif
        return -ERR_HWERR;
    }
    
    /* copy back the GPIO value */
    ptmpvar = (uint16_t *)&bufrx[4];
    *pgpvalue = le16toh(*ptmpvar);
    
    return ERR_NOERR;
}
/*==========================================================*/
int gpio_setdir(int filedesc, uint16_t gpdir)
{
    unsigned char buftx[MCP2210_HID_REPORT_LEN];
    unsigned char bufrx[MCP2210_HID_REPORT_LEN];
    int ires;

    buftx[0] = cSPI_D_SETGPIODIR;
    buftx[4] = (uint8_t)(gpdir & 0xFF);
    buftx[5] = (uint8_t)((gpdir & 0xFF00) >> 8);
    
    /* Send a Report to the Device */
    ires = write(filedesc, buftx, MCP2210_HID_REPORT_LEN);
    if (ires < 0) {
#ifdef DEBUG_INFO
        printf("\nError: %d", errno);
        perror("\n\twrite_eeprom() -> write");
#endif
        return -ERR_WRERR;
    } else {
#ifdef DEBUG_INFO
        printf("\n\twrite_eeprom() -> write() wrote %d bytes", ires);
#endif
    }

#ifdef DEBUG_INFO
    print_report_buffer(buftx, MCP2210_HID_REPORT_LEN, 10);
#endif
    /*usleep(100 * 1000);*/

    /* Get a report from the device */
    ires = read(filedesc, bufrx, MCP2210_HID_REPORT_LEN);
    if (ires < 0) {
#ifdef DEBUG_INFO
        printf("\nError: %d", errno);
        perror("\n\twrite_eeprom() -> read");
#endif
        return -ERR_RDERR;
    } else {
#ifdef DEBUG_INFO
        printf("\n\twrite_eeprom() -> read() read %d bytes:", ires);
#endif
    }
#ifdef DEBUG_INFO
    print_report_buffer(bufrx, MCP2210_HID_REPORT_LEN, 10);
#endif
    if (bufrx[1] != cCHIP_NOERR)
    {
        if (bufrx[1] == cCHIP_BLOCKEDACCESS)
        {
#ifdef DEBUG_INFO
            printf("\nwrite_eeprom() - CMD status ERROR");
#endif
            return -ERR_BLOCKEDACCESS;
        }
#ifdef DEBUG_INFO
        printf("\nwrite_eeprom() - CMD status ERROR");
#endif
        return -ERR_HWERR;
    }
    
    return ERR_NOERR;
}
/*==========================================================*/
int gpio_getdir(int filedesc, uint16_t *pgpdir)
{
    unsigned char buftx[MCP2210_HID_REPORT_LEN];
    unsigned char bufrx[MCP2210_HID_REPORT_LEN];
    int ires;
    uint16_t *ptmpvar;

    buftx[0] = cSPI_D_GETGPIODIR;
    
    /* Send a Report to the Device */
    ires = write(filedesc, buftx, MCP2210_HID_REPORT_LEN);
    if (ires < 0) {
#ifdef DEBUG_INFO
        printf("\nError: %d", errno);
        perror("\n\twrite_eeprom() -> write");
#endif
        return -ERR_WRERR;
    } else {
#ifdef DEBUG_INFO
        printf("\n\twrite_eeprom() -> write() wrote %d bytes", ires);
#endif
    }

#ifdef DEBUG_INFO
    print_report_buffer(buftx, MCP2210_HID_REPORT_LEN, 10);
#endif
    /*usleep(100 * 1000);*/

    /* Get a report from the device */
    ires = read(filedesc, bufrx, MCP2210_HID_REPORT_LEN);
    if (ires < 0) {
#ifdef DEBUG_INFO
        printf("\nError: %d", errno);
        perror("\n\twrite_eeprom() -> read");
#endif
        return -ERR_RDERR;
    } else {
#ifdef DEBUG_INFO
        printf("\n\twrite_eeprom() -> read() read %d bytes:", ires);
#endif
    }
#ifdef DEBUG_INFO
    print_report_buffer(bufrx, MCP2210_HID_REPORT_LEN, 10);
#endif
    if (bufrx[1] != cCHIP_NOERR)
    {
        if (bufrx[1] == cCHIP_BLOCKEDACCESS)
        {
#ifdef DEBUG_INFO
            printf("\nwrite_eeprom() - CMD status ERROR");
#endif
            return -ERR_BLOCKEDACCESS;
        }
#ifdef DEBUG_INFO
        printf("\nwrite_eeprom() - CMD status ERROR");
#endif
        return -ERR_HWERR;
    }
    
    /* copy back the GPIO value */
    ptmpvar = (uint16_t *)&bufrx[4];
    *pgpdir = le16toh(*ptmpvar);
    
    return ERR_NOERR;
}
/*==========================================================*/
int gpio_write(int filedesc, int gpioval, int gpiomask)
{
    stDevChipSettings_T stsettings;
    int idx, ires;
    uint16_t tmpval, tmpdir;
    
    /* get the current chip settings */
    ires = get_crt_settings(filedesc, &stsettings);
    if (ires != ERR_NOERR)
    {
        return -ERR_GETSETTINGS;
    }

    /* assign the needed GPs for CS operation */
    for(idx = 0; idx < cDEV_NOGPIOS; idx++)
    {
        tmpval = 1 << idx;
        tmpdir = 1 << idx;
        if (gpiomask & (1 << idx))
        {/* the current pin will be assigned for GPIO operation*/
            stsettings.ucPinOption[idx] = cDEV_PIN_GPIO;
            /* if the GP is selected, clear its bit out from direction */
            stsettings.uiGpioDirection &= (uint16_t)(~(tmpdir));
            stsettings.uiGpioOutputDefaultValue &= (uint16_t)(~(tmpval));
            stsettings.uiGpioOutputDefaultValue |= (uint16_t)(tmpval & gpioval);
        }
    }

    /* set the current chip settings */
    ires = set_crt_settings(filedesc, &stsettings);
    if (ires != ERR_NOERR)
    {
        return -ERR_SETSETTINGS;
    }
    
    /* set the given GPIO value */
    ires = gpio_setval(filedesc, (uint16_t)gpioval);
    if (ires != ERR_NOERR)
    {
        return -ERR_WRITEGPIO;
    }
    
    return ERR_NOERR;
}
/*==========================================================*/
int gpio_read(int filedesc, int *pgpioval, int gpiomask)
{
    stDevChipSettings_T stsettings;
    int idx, ires;
    uint16_t tmpval, tmpdir;
    
    /* get the current chip settings */
    ires = get_crt_settings(filedesc, &stsettings);
    if (ires != ERR_NOERR)
    {
        return -ERR_GETSETTINGS;
    }

    /* assign the needed GPs for CS operation */
    for(idx = 0; idx < cDEV_NOGPIOS; idx++)
    {
        tmpval = 1 << idx;
        tmpdir = 1 << idx;
        if (gpiomask & (1 << idx))
        {/* the current pin will be assigned for GPIO operation*/
            stsettings.ucPinOption[idx] = cDEV_PIN_GPIO;
            /* if the GP is selected, set its bit out from direction */
            stsettings.uiGpioDirection |= (uint16_t)(tmpdir);
        }
    }

    /* set the current chip settings */
    ires = set_crt_settings(filedesc, &stsettings);
    if (ires != ERR_NOERR)
    {
        return -ERR_SETSETTINGS;
    }
    
    /* get the given GPIO value */
    ires = gpio_getval(filedesc, &tmpval);
    if (ires != ERR_NOERR)
    {
        return -ERR_READGPIO;
    }
    
    /* copy back the value */
    *pgpioval = (int)tmpval;
    
    return ERR_NOERR;
}
/*==========================================================*/
int gpio_direction(int filedesc, int gpiodir, int gpiomask)
{
    stDevChipSettings_T stsettings;
    int idx, ires;
    uint16_t tmpdir;
    
    /* get the current chip settings */
    ires = get_crt_settings(filedesc, &stsettings);
    if (ires != ERR_NOERR)
    {
        return -ERR_GETSETTINGS;
    }

    /* assign the needed GPs for CS operation */
    for(idx = 0; idx < cDEV_NOGPIOS; idx++)
    {
        tmpdir = 1 << idx;
        if (gpiomask & (1 << idx))
        {/* the current pin will be assigned for GPIO operation*/
            stsettings.ucPinOption[idx] = cDEV_PIN_GPIO;
            /* if the GP is selected, clear/set its bit out from direction */
            stsettings.uiGpioDirection &= (uint16_t)(~(tmpdir));
            stsettings.uiGpioDirection |= (uint16_t)(tmpdir & gpiodir);
        }
    }

    /* set the current chip settings */
    ires = set_crt_settings(filedesc, &stsettings);
    if (ires != ERR_NOERR)
    {
        return -ERR_SETSETTINGS;
    }
    
    /* set the given GPIO direction */
    ires = gpio_setdir(filedesc, gpiodir);
    if (ires != ERR_NOERR)
    {
        return -ERR_SETGPIODIR;
    }
    
    return ERR_NOERR;
}
/*==========================================================*/
void print_report_buffer(unsigned char *bufdata, int len, int rowlen)
{
    int ictr = 0;
    int irowidx = 0;

    while(ictr < len)
    {
        if ((ictr%rowlen) == 0)
        {/* new row needed */
            printf("\n%04d: ", ictr);
        }
        for(irowidx = 0; ((irowidx < rowlen) && (ictr < len));)
        {
            printf("  %02X", bufdata[ictr]);
            irowidx++;
            ictr++;
        }
    }
}
/*==========================================================*/
