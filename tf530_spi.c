/*
 * This file is part of the flashrom project.
 *
 * Copyright (C) 2011 Sven Schnelle <svens@stackframe.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
 */

#if CONFIG_TF530_SPI == 1

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <ctype.h>
#include "flash.h"
#include "chipdrivers.h"
#include "programmer.h"
#include "spi.h"


#include <exec/resident.h>
#include <exec/errors.h>
#include <exec/memory.h>
#include <exec/lists.h>
#include <exec/alerts.h>
#include <exec/tasks.h>
#include <exec/io.h>
#include <exec/execbase.h>

#include <libraries/expansion.h>

#include <devices/trackdisk.h>
#include <devices/timer.h>
#include <devices/scsidisk.h>

#include <dos/filehandler.h>

#include <proto/exec.h>
#include <proto/disk.h>
#include <proto/expansion.h>


struct TF530SDRegs {
    volatile uint8_t ctrl; // Control register
    volatile uint8_t unused1; // padding
    volatile uint8_t unused2; // padding
    volatile uint8_t unused3 ; // padding
    volatile uint8_t data; // data
};


static struct ConfigDev* cd = NULL;
static int tf530_spi_shutdown(void *data);
static int tf530_spi_send_command(struct flashctx *flash, unsigned int writecnt,
                                  unsigned int readcnt,
                                  const unsigned char *txbuf,
                                  unsigned char *rxbuf);

#define TF530_CTRL_CS0 1
#define TF530_CTRL_CS1 2
#define TF530_CTRL_BUSY 4
#define TF530_CTRL_CS2 8

void spi_send_byte(struct TF530SDRegs* port, uint8_t value)
{
    uint8_t busy = 0;

    while (busy  == 0)
    {
        busy = port->ctrl & TF530_CTRL_BUSY;
    }

    port->data = value;
}

uint8_t inline spi_recv_byte(struct TF530SDRegs* port)
{
    uint8_t busy = 0;

    while (busy  == 0)
    {
        busy = port->ctrl & TF530_CTRL_BUSY;
    }

    return port->data;
}

void spi_cs_unassert(struct TF530SDRegs* port)
{
    uint8_t current = port->ctrl;
    port->ctrl = current | TF530_CTRL_CS0 | TF530_CTRL_CS1 | TF530_CTRL_CS2 | TF530_CTRL_BUSY;
}

void spi_cs_assert(struct TF530SDRegs* port)
{
    uint8_t current = port->ctrl;
    port->ctrl = current & (~TF530_CTRL_CS2);
}

static const struct spi_master spi_master_tf530 = {
    .type		= SPI_CONTROLLER_TF530,
    .features	= SPI_MASTER_4BA,
    .max_data_read	= 256,
    .max_data_write	= 256,
    .command	= tf530_spi_send_command,
    .multicommand	= default_spi_send_multicommand,
    .read		= default_spi_read,
    .write_256	= default_spi_write_256,
    .write_aai	= default_spi_write_aai,
};

int tf530_spi_init(void)
{
    struct Library* ExpansionBase;

    if ((ExpansionBase = (struct Library*)OpenLibrary("expansion.library",0L))==NULL) {
        return 0;
    }

    if (cd = (struct ConfigDev*)FindConfigDev(cd,0x13D8,0x81))
    {
        msg_cinfo("\nTF530 SPI found in autoconfig chain.\n");
        CloseLibrary(ExpansionBase);
    }
    else
    {
        msg_perr("\nTF530 SPI not found in autoconfig chain.\n");
        CloseLibrary(ExpansionBase);
        return -1;
    }

    if (register_shutdown(tf530_spi_shutdown, NULL))
        return 1;

    register_spi_master(&spi_master_tf530);

    return 0;
}

static int tf530_spi_shutdown(void *data)
{
    // TODO: free config dev?
    return 0;
}

static int tf530_spi_send_command(struct flashctx *flash, unsigned int writecnt,
                                  unsigned int readcnt,
                                  const unsigned char *txbuf,
                                  unsigned char *rxbuf)
{
    struct TF530SDRegs *port = cd->cd_BoardAddr;

    spi_cs_assert(port);

    for (int i = 0; i < writecnt; i++)
    {
        spi_send_byte(port, txbuf[i]);
    }

    // read the dummy byte and make sure
    // the compiler doesnt optimize it away
    uint8_t volatile dummy = spi_recv_byte(port);


    for (int i = 0; i < readcnt; i++)
    {
        rxbuf[i] = spi_recv_byte(port);
    }

    spi_cs_unassert(port);

    return 0;
}

#endif // CONFIG_TF530_SPI == 1
