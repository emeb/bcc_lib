/* bcc_lib.c - Blank Canvas Cape FPGA board access library                   */
/* Copyright 2012 Eric Brombaugh                                             */
/*   This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
    MA 02110-1301 USA.
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <errno.h>
#include <math.h>
#include "bcc_lib.h"

const unsigned char idprom[] =
{
	0x00, 0x06, 0x00, 0x01, 0x00, 0x00
};

/* .bit file header */
const char bit_hdr[] =
{
	0x00, 0x09, 0x0F, 0xF0, 0x0F, 0xF0, 0x0F, 0xF0, 0x0F, 0xF0, 0x00, 0x00, 0x01
};

const char *bit_hdr_strings[] =
{
	"filename",
	"device",
	"date",
	"time"
};

const char *deviceid = "3s200avq100";

/* M25P20 Flash ID */
const char flash_id[] = 
{
	0x20, 0x20, 0x12
};

/* wrapper for fprintf(stderr, ...) to support verbose control */
void qprintf(bfpga *s, char *fmt, ...)
{
	va_list args;
	
	if(s->verbose)
	{
		va_start(args, fmt);
		vfprintf(stderr, fmt, args);
		va_end(args);
	}
}

/* set I2C slave address */
int i2c_set_slave_addr(int file, int address)
{
	if(ioctl(file, I2C_SLAVE, address) < 0)
		return -errno;
	else
		return 0;
}

/* low-level interface to I2C read/write */
static inline __s32 i2c_smbus_access(int file, char read_write, __u8 command, 
                                     int size, union i2c_smbus_data *data)
{
	struct i2c_smbus_ioctl_data args;

	args.read_write = read_write;
	args.command = command;
	args.size = size;
	args.data = data;
	return ioctl(file,I2C_SMBUS,&args);
}

/* get a single byte from an I2C EEPROM */
int bcc_i2c_get_prom(bfpga *s, int saddr, __u8 loc)
{
	union i2c_smbus_data data;
	
	/* set address */
	if(i2c_set_slave_addr(s->i2c_file, saddr))
		return -1;

	/* get data */
	if (i2c_smbus_access(s->i2c_file,I2C_SMBUS_READ, loc,
	                     I2C_SMBUS_BYTE_DATA,&data))
		return -1;
	else
		return 0x0FF & data.byte;
}

/* send two bytes to I2C slave */
int bcc_i2c_set_prom(bfpga *s, int saddr, __u8 loc, __u8 dat)
{
	union i2c_smbus_data data;
	
	data.byte = dat;
	
	/* set address */
	if(i2c_set_slave_addr(s->i2c_file, saddr))
		return -1;

	/* send data */
	return i2c_smbus_access(s->i2c_file,I2C_SMBUS_WRITE, loc,
	                     I2C_SMBUS_BYTE_DATA,&data);
}

/* get a single byte from an I2C slave */
int bcc_i2c_get(bfpga *s, int saddr)
{
	union i2c_smbus_data data;
	
	/* set address */
	if(i2c_set_slave_addr(s->i2c_file, saddr))
		return -1;

	/* get data */
	if (i2c_smbus_access(s->i2c_file,I2C_SMBUS_READ, 0,
	                     I2C_SMBUS_BYTE,&data))
		return -1;
	else
		return 0x0FF & data.byte;
}

/* set a single byte to an I2C slave */
int bcc_i2c_set(bfpga *s, int saddr, __u8 dat)
{
	/* set address */
	if(i2c_set_slave_addr(s->i2c_file, saddr))
		return -1;

	/* send data */
	return i2c_smbus_access(s->i2c_file,I2C_SMBUS_WRITE,dat,
	                        I2C_SMBUS_BYTE,NULL);
}

/* get PCF GPIO bit */
int bcc_i2c_pcf_rd(bfpga *s, int bit)
{
	return (bcc_i2c_get(s, 0x38) >> bit) & 1;
}

/* get PCF GPIO bit */
void bcc_i2c_pcf_wr(bfpga *s, int bit, int val)
{
	int oldval = bcc_i2c_get(s, 0x38) | 0xC0;		// don't clear uppers
	int hmask = 1<<bit;
	int lmask = ~hmask & 0xFF;
	int newval = (oldval & lmask) | ((val & 1) << bit);
	bcc_i2c_set(s, 0x38, newval);
}

/* SPI Transmit/Receive */
int bcc_spi_txrx(bfpga *s, uint8_t *tx, uint8_t *rx, __u32 len)
{
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = len,
		.delay_usecs = 0,
		.speed_hz = 2000000,
		.bits_per_word = 8,
	};
	
	return ioctl(s->spi_file, SPI_IOC_MESSAGE(1), &tr);
}

/* initialize our FPGA interface */
bfpga *bcc_init(int i2c_bus, int spi_bus, int spi_add, int verbose)
{
	bfpga *s;
	char filename[20];
	uint32_t speed = 10000000;
	uint8_t mode = 0;
	//uint8_t mode = SPI_CPHA;
	//uint8_t mode = SPI_CPOL;
	int i;
	
	/* allocate the object */
	if((s = calloc(1, sizeof(bfpga))) == NULL)
	{
		qprintf(s, "bcc_init: Couldn't allocate bfpga object\n");
		goto fail0;
	}
	
	/* set verbose level */
	s->verbose = verbose;
	
	/* open I2C bus */
	sprintf(filename, "/dev/i2c-%d", i2c_bus);
	s->i2c_file = open(filename, O_RDWR);

	if(s->i2c_file < 0)
	{
		qprintf(s, "bcc_init: Couldn't open I2C device %s\n", filename);
		goto fail1;
	}
	else
		qprintf(s, "bcc_init: opened I2C device %s\n", filename);
		
	
#if 0
	/* Check for the Beagle FPGA ID PROM */
	for(i=0;i<6;i++)
	{
		int value = bcc_i2c_get_prom(s, 0x50, i);
		qprintf(s, "bcc_init: ID[%1d] = 0x%02X\n", i, value);
		
		if(value != idprom[i])                                          
		{
			qprintf(s, "bcc_init: IDPROM mismatch - giving up\n");
			goto fail2;
		}
	}
	qprintf(s, "bcc_init: found IDPROM\n");
#endif
	
	/* diagnostid - check status of port expander */
	qprintf(s, "bcc_init: PCF reads 0x%02X\n", bcc_i2c_get(s, 0x38));

	/* Open the SPI port */
	sprintf(filename, "/dev/spidev%d.%d", spi_bus, spi_add);
	s->spi_file = open(filename, O_RDWR);
	
	if(s->spi_file < 0)
	{
		qprintf(s, "bcc_init: Couldn't open spi device %s\n", filename);
		goto fail2;
	}
	else
		qprintf(s, "bcc_init: opened spi device %s\n", filename);

	if(ioctl(s->spi_file, SPI_IOC_WR_MAX_SPEED_HZ, &speed) == -1)
	{
		qprintf(s, "bcc_init: Couldn't set SPI clock to %d Hz\n", speed);
		goto fail2;
	}
	else
		qprintf(s, "bcc_init: Set SPI clock to %d Hz\n", speed);
	
	if(ioctl(s->spi_file, SPI_IOC_WR_MODE, &mode) == -1)
	{
		qprintf(s, "bcc_init: Couldn't set SPI mode\n");
		goto fail2;
	}
	else
		qprintf(s, "bcc_init: Set SPI mode\n");
	
	/* Check if FPGA already configured */
	if(bcc_i2c_pcf_rd(s,PCF_FPGA_DONE)==0)
	{
		qprintf(s, "bcc_init: FPGA not already configured - DONE not high\n\r");
	}	
	
	/* Allow OMAP to drive SPI bus */
	bcc_i2c_pcf_wr(s,PCF_FPGA_SPI_MODE,0);		// drive nOE line low
	qprintf(s, "bcc_init: OMAP Drives SPI bus\n");
		
	/* success */
	return s;

	/* failure modes */
//fail3:
//	close(s->spi_file);		/* close the SPI device */
fail2:
	close(s->i2c_file);		/* close the I2C device */
fail1:
	free(s);				/* free the structure */
fail0:
	return NULL;
}

/* open a bitfile and check the header */
FILE *bfpga_open_bitfile(bfpga *s, char *bitfile, long *n)
{
	FILE *fd;
	char readbuf[READBUFSIZE], *cp;
	int read, j, d;
	
	/* open file or return error*/
	if(!(fd = fopen(bitfile, "r")))
	{
		qprintf(s, "bfpga_open_bitfile: open file %s failed\n\r", bitfile);
		return 0;
	}
	else
	{
		qprintf(s, "bfpga_open_bitfile: found bitstream file %s\n\r", bitfile);
	}

	/* Read file & send bitstream via SPI1 */
	qprintf(s, "bfpga_open_bitfile: parsing header\n\r");
	if( (read=fread(readbuf, sizeof(char), 13, fd)) == 13 )
	{
		/* init pointer to keep track */
		cp = readbuf;
		
		/* check / skip .bit header */
		for(j=0;j<13;j++)
		{
			if(bit_hdr[j] != *cp++)
			{
				qprintf(s, "bfpga_open_bitfile: .bit header mismatch\n\r");
				fclose(fd);
				return 0;
			}
		}
		qprintf(s, "bfpga_open_bitfile: found header\n\r");
	}
	else
	{
		qprintf(s, "bfpga_open_bitfile: .bit header truncated\n\r");
		fclose(fd);
		return 0;
	}
		
	/* Skip File header chunks */
	for(j=0;j<4;j++)
	{
		if( (read=fread(readbuf, sizeof(char), 3, fd)) == 3 )
		{
			/* init pointer to keep track */
			cp = readbuf;
		
			/* get 1 byte chunk desginator (a,b,c,d) */
			d = *cp++;
			
			/* compute chunksize */
			*n = *cp++;
			*n <<= 8;
			*n += *cp;
			
			/* read chunk */
			if( (read=fread(readbuf, sizeof(char), *n, fd)) == *n )
			{
				/* print chunk */
				qprintf(s, "bfpga_open_bitfile: chunk %c length %ld %s %s\n\r", d, *n, bit_hdr_strings[j], readbuf);
			}
			else
			{
				qprintf(s, "bfpga_open_bitfile: chunk data truncated\n\r");
				fclose(fd);
				return 0;
			}
			
			/* Check device type */
			if(j==1)
			{
				if(strcmp(readbuf, deviceid))
					qprintf(s, "bfpga_open_bitfile: Device != %s\n\r", deviceid);
				else
					qprintf(s, "bfpga_open_bitfile: Device == %s\n\r", deviceid);
			}
		}
		else
		{
			qprintf(s, "bfpga_open_bitfile: chunk header truncated\n\r");
			fclose(fd);
			return 0;
		}
	}
	
	if( (read=fread(readbuf, sizeof(char), 5, fd)) == 5 )
	{
		/* init pointer to keep track */
		cp = readbuf;
		
		/* Skip final chunk designator */
		cp++;
	
		/* compute config data size - modified for 16-bit int & char */
		*n = *cp++;
		*n <<= 8;
		*n += *cp++;
		*n <<= 8;
		*n += *cp++;
		*n <<= 8;
		*n += *cp++;
		qprintf(s, "bfpga_open_bitfile: config size = %ld\n\r", *n);
	}
	else
	{
		qprintf(s, "bfpga_open_bitfile: final chunk truncated\n\r");
		fclose(fd);
		return 0;
	}
	
	/* success */
	return fd;
}

/* Send a bitstream to the FPGA */
int bcc_cfg(bfpga *s, char *bitfile)
{
	FILE *fd;
    int read;
	long ct, n;
	unsigned char byte, rxbyte, dummybuf[READBUFSIZE];
	char readbuf[READBUFSIZE];

	/* open file or return error*/
	if(!(fd = bfpga_open_bitfile(s, bitfile, &n)))
	{
		qprintf(s, "bcc_cfg: open bitfile %s failed\n\r", bitfile);
		return 1;
	}
	else
	{
		qprintf(s, "bcc_cfg: found bitfile %s\n\r", bitfile);
	}

	/* Set FLASH_DRV */
	qprintf(s, "bcc_cfg: Setting OMAP -> FPGA cfg\n\r");
	bcc_i2c_pcf_wr(s,PCF_OMAP_FLASH_DRV,0);		// drive low
	
	/* pulse PROG_B low min 500 ns */
	bcc_i2c_pcf_wr(s,PCF_FPGA_PROG,0);			// drive low
	usleep(1);			// wait a bit
	
	/* Wait for INIT low */
	qprintf(s, "bcc_cfg: PROG low, Waiting for INIT low\n\r");
	while(bcc_i2c_pcf_rd(s,PCF_FPGA_INIT)==1)
	{
		asm volatile ("nop");	//"nop" means no-operation.  We don't want to do anything during the delay
	}
	
	/* Release PROG */
	bcc_i2c_pcf_wr(s,PCF_FPGA_PROG,1);			// set as hi
	
	/* Wait for INIT high */
	qprintf(s, "bcc_cfg: PROG high, Waiting for INIT high\n\r");
	while(bcc_i2c_pcf_rd(s,PCF_FPGA_INIT)==0)
	{
		asm volatile ("nop");	//"nop" means no-operation.  We don't want to do anything during the delay
	}

	/* wait 5us */
	usleep(5);
	qprintf(s, "bcc_cfg: Sending bitstream\n\r");
	
	/* Read file & send bitstream to FPGA via SPI */
	ct = 0;
	while( (read=fread(readbuf, sizeof(char), READBUFSIZE, fd)) > 0 )
	{
		/* Send bitstream */
		bcc_spi_txrx(s, (unsigned char *)readbuf, dummybuf, read);
		ct += read;
		
		/* diagnostic to track buffers */
		qprintf(s, ".");
		if(s->verbose)
			fflush(stdout);
		
		/* Check INIT - if low then fail */
		if(bcc_i2c_pcf_rd(s,PCF_FPGA_INIT)==0)
		{
			qprintf(s, "\n\rbcc_cfg: INIT low during bitstream send\n\r");
			fclose(fd);
			return 1;
		}
	}
	
	/* close file */
	qprintf(s, "\n\rbcc_cfg: sent %ld of %ld bytes\n\r", ct, n);
	qprintf(s, "bcc_cfg: bitstream sent, closing file\n\r");
	fclose(fd);
	
	/* send dummy data while waiting for DONE or !INIT */
 	qprintf(s, "bcc_cfg: sending dummy clocks, waiting for DONE or fail\n\r");
	byte = 0xFF;
	ct = 0;
	while((bcc_i2c_pcf_rd(s,PCF_FPGA_DONE)==0) && (bcc_i2c_pcf_rd(s,PCF_FPGA_INIT)==1))
	{
		/* Dummy - all ones */
		bcc_spi_txrx(s, &byte, &rxbyte, 1);
		ct++;
	}
 	qprintf(s, "bcc_cfg: %d dummy clocks sent\n\r", ct*8);
		
	/* Clear FLASH_DRV */
	qprintf(s, "bcc_cfg: Setting FLASH -> FPGA cfg\n\r");
	bcc_i2c_pcf_wr(s,PCF_OMAP_FLASH_DRV,1);		// drive high
			
	/* return status */
	if(bcc_i2c_pcf_rd(s,PCF_FPGA_DONE)==0)
	{
		qprintf(s, "bcc_cfg: cfg failed - DONE not high\n\r");
		return 1;	// Done = 0 - error
	}
	else	
	{
		qprintf(s, "bcc_cfg: success\n\r");
		return 0;	// Done = 1 - OK
	}
}

/* Send a bitstream to the SPI Configuration Flash */
int bcc_pgm(bfpga *s, char *bitfile)
{
	int i, ct, read;
	long n;
	unsigned char txbuf[READBUFSIZE], rxbuf[READBUFSIZE], *cp;
	char *fcfg_fname = "flash_prog.bit";
	FILE *fd;
	
#if 0	
	/* send special flash cfg bitstream */
	qprintf(s, "bcc_cfg: sending flash cfg bitstream %s\n\r", fcfg_fname);
	if(bcc_cfg(s, fcfg_fname))
	{
		qprintf(s, "bcc_pgm: Error sending bitstream to FPGA\n");
		return 1;
	}
#endif

	/* Check flash memory ID */
	qprintf(s, "bcc_pgm: Read Flash ID:\n");
	txbuf[0] = FLASH_RDID;
	bcc_spi_txrx(s, txbuf, rxbuf, 21);
	qprintf(s, "MFG ID: 0x%02X\n", rxbuf[1]);
	qprintf(s, "DEV ID: 0x%02X 0x%02X\n", rxbuf[2], rxbuf[3]);
	cp = &rxbuf[1];
	for(i=0;i<3;i++)
	{
		if(flash_id[i] != *cp++)
		{
			qprintf(s, "bcc_pgm: Flash ID mismatch\n\r");
			return 1;
		}
	}
	qprintf(s, "bcc_pgm: found Flash ID\n\r");
	
	/* open the bitfile */
	if(!(fd = bfpga_open_bitfile(s, bitfile, &n)))
	{
		qprintf(s, "bcc_pgm: open bitfile %s failed\n\r", bitfile);
		return 1;
	}
	else
	{
		qprintf(s, "bcc_pgm: found bitfile %s\n\r", bitfile);
	}

	/* Erase the Flash */
	qprintf(s, "bcc_pgm: Sending WREN\n");
	txbuf[0] = FLASH_WREN;
	bcc_spi_txrx(s, txbuf, rxbuf, 1);
	
	qprintf(s, "bcc_pgm: Sending Bulk Erase\n");
	txbuf[0] = FLASH_BE;
	bcc_spi_txrx(s, txbuf, rxbuf, 1);
	txbuf[0] = FLASH_RDSR;
	txbuf[1] = 0x00;
	i = 0;
	do
	{
		bcc_spi_txrx(s, txbuf, rxbuf, 2);
		i++;
		usleep(5);
	}
	while(((rxbuf[1]&0x01) == 0x01) && (i < 30000));
	if(i == 30000)
	{
		qprintf(s, "bcc_pgm: Timed out waiting for Bulk Erase\n");
		fclose(fd);
		return 1;
	}
	qprintf(s, "bcc_pgm: Bulk Erase complete (%d checks)\n", i);
	
	/* Read file & send bitstream to flash via SPI */
	qprintf(s, "bcc_pgm: Sending PP\n");
	ct = 0;
	while( (read=fread(&txbuf[4], sizeof(char), 256, fd)) > 0 )
	{
		/* Send WREN */
		txbuf[0] = FLASH_WREN;
		bcc_spi_txrx(s, txbuf, rxbuf, 1);
	
		/* Tack on header */
		txbuf[0] = FLASH_PP;
		txbuf[1] = (ct>>16)&0xff;
		txbuf[2] = (ct>>8)&0xff;
		txbuf[3] = ct&0xff;
	
		/* Send PP & bitstream */
		bcc_spi_txrx(s, (unsigned char *)txbuf, rxbuf, read+4);
		ct += read;
		
		/* Wait for PP to complete */
		txbuf[0] = FLASH_RDSR;
		txbuf[1] = 0x00;
		i = 0;
		do
		{
			bcc_spi_txrx(s, txbuf, rxbuf, 2);
			i++;
			usleep(5);
		}
		while(((rxbuf[1]&0x01) == 0x01) && (i < 30000));
		if(i == 30000)
		{
			qprintf(s, "bcc_pgm: Timed out waiting for Page Program\n");
			fclose(fd);
			return 1;
		}
		
		/* diagnostic to track buffers */
		qprintf(s, ".");
		if(s->verbose)
			fflush(stdout);
	}
	qprintf(s, "bcc_pgm: Programmed %d bytes \n", ct);
	
	/* success */
	fclose(fd);
	return 0;
}

/* Clean shutdown of our FPGA interface */
void bcc_delete(bfpga *s)
{
	/* Release SPI bus */
	bcc_i2c_pcf_wr(s,PCF_FPGA_SPI_MODE,1);			// drive nOE line high
	qprintf(s, "bcc_init: OMAP off SPI bus\n");
		
	close(s->spi_file);		/* close the SPI device */
	close(s->i2c_file);		/* close the I2C device */
	free(s);				/* free the structure */
}
