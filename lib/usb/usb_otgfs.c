/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2011 Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <string.h>
#include <libopencm3/cm3/common.h>
#include <libopencm3/stm32/tools.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/dwc/otg_fs.h>
#include <libopencm3/usb/dwc/otg_common.h>
#include "usb_private.h"
#include <stdio.h>

/* Receive FIFO size in 32-bit words. */
#define RX_FIFO_SIZE 128
#define dev_base_address (_usbd_dev.driver->base_address)
#define REBASE(x)        MMIO32((x) + (dev_base_address))

static usbd_device *otgfs_usb_init(void);
static void otgfs_usb_set_address(usbd_device *, uint8_t);
static void otgfs_usb_ep_setup(usbd_device *, uint8_t, uint8_t, uint16_t, void (*) (usbd_device *, uint8_t) );
static void otgfs_usb_reset(usbd_device *);
static void otgfs_usb_ep_stall_set(usbd_device *, uint8_t, uint8_t);
static uint8_t otgfs_usb_ep_stall_get(usbd_device *, uint8_t);
static void otgfs_usb_ep_nak_set(usbd_device *, uint8_t, uint8_t);
static uint16_t otgfs_usb_ep_write_packet(usbd_device *, uint8_t, const void *, uint16_t);
static uint16_t otgfs_usb_ep_read_packet(usbd_device *, uint8_t, void *, uint16_t);
static void otgfs_usb_poll(usbd_device *);
static void otgfs_usb_disconnect(usbd_device *, bool);

static struct _usbd_device _usbd_dev;

const struct _usbd_driver otgfs_usb_driver = {
	.init = otgfs_usb_init,
	.set_address = otgfs_usb_set_address,
	.ep_setup = otgfs_usb_ep_setup,
	.ep_reset = otgfs_usb_reset,
	.ep_stall_set = otgfs_usb_ep_stall_set,
	.ep_stall_get = otgfs_usb_ep_stall_get,
	.ep_nak_set = otgfs_usb_ep_nak_set,
	.ep_write_packet = otgfs_usb_ep_write_packet,
	.ep_read_packet = otgfs_usb_ep_read_packet,
	.poll = otgfs_usb_poll,
	.disconnect = otgfs_usb_disconnect,
	.base_address = USB_OTG_FS_BASE,
	.set_address_before_status = 1,
	.rx_fifo_size = RX_FIFO_SIZE,
};

// otgfs_usb_init()

static void otgfs_usb_core_init(void) {
	// 22.17.1 Core initialization
	//
	// The application must perform the core initialization sequence. If the
	// cable is connected during power-up, the current mode of operation bit in
	// the OTG_FS_GINTSTS (CMOD bit in OTG_FS_GINTSTS) reflects the mode. The
	// OTG_FS controller enters host mode when an “A” plug is connected or
	// device mode when a “B” plug is connected.
	//
	// This section explains the initialization of the OTG_FS controller after
	// power-on. The application must follow the initialization sequence
	// irrespective of host or device mode operation. All core global registers
	// are initialized according to the core’s configuration:
	//
	// 1. Program the following fields in the OTG_FS_GAHBCFG register:
	//   – Global interrupt mask bit GINTMSK = 1
	OTG_FS_GAHBCFG |= OTG_GAHBCFG_GINT;
	//   – RxFIFO non-empty (RXFLVL bit in OTG_FS_GINTSTS)
	OTG_FS_GINTSTS |= OTG_GINTSTS_RXFLVL;
	//   – Periodic TxFIFO empty level
	OTG_FS_GINTSTS |= OTG_GINTSTS_PTXFE;
	// 2. Program the following fields in the OTG_FS_GUSBCFG register:
	//   – HNP capable bit
	OTG_FS_GUSBCFG |= OTG_GUSBCFG_HNPCAP;
	//   – SRP capable bit
	OTG_FS_GUSBCFG |= OTG_GUSBCFG_SRPCAP;
	//   – FS timeout calibration field
	// SKIP
	//   – USB turnaround time field
	OTG_FS_GUSBCFG |= OTG_GUSBCFG_TRDT_MASK;
	// 3. The software must unmask the following bits in the OTG_FS_GINTMSK
	//   register:
	//   OTG interrupt mask
	OTG_FS_GINTMSK |= OTG_GINTSTS_OTGINT;
	//   Mode mismatch interrupt mask
	OTG_FS_GINTMSK |= OTG_GINTSTS_MMIS;
	// 4. The software can read the CMOD bit in OTG_FS_GINTSTS to determine
	//   whether the OTG_FS controller is operating in host or device mode.
	while(OTG_FS_GINTSTS & OTG_GINTSTS_CMOD);
}

static void otgfs_usb_device_init(void) {
	// 22.17.3
	//
	// The application must perform the following steps to initialize the core
	// as a device on power-up or after a mode change from host to device.
	//
	// 1. Program the following fields in the OTG_FS_DCFG register:
	//   – Device speed
	OTG_FS_DCFG |= OTG_DCFG_DSPD;
	//   – Non-zero-length status OUT handshake
	OTG_FS_DCFG |= OTG_DCFG_NZLSOHSK;
	// 2. Program the OTG_FS_GINTMSK register to unmask the following
	//   interrupts:
	//   – USB reset
	OTG_FS_GINTMSK |= OTG_GINTSTS_USBRST;
	//   – Enumeration done
	OTG_FS_GINTMSK |= OTG_GINTMSK_ENUMDNEM;
	//   – Early suspend
	OTG_FS_GINTMSK |= OTG_GINTSTS_ESUSP;
	//   – USB suspend
	OTG_FS_GINTMSK |= OTG_GINTSTS_USBSUSP;
	//   – SOF
	OTG_FS_GINTMSK |= OTG_GINTSTS_SOF;
	// 3. Program the VBUSBSEN bit in the OTG_FS_GCCFG register to enable V BUS
	// sensing in “B” device mode and supply the 5 volts across the pull-up
	// resistor on the DP line.
	OTG_FS_GCCFG |= OTG_GCCFG_NOVBUSSENS;

	// Force device mode
	OTG_FS_GUSBCFG |= OTG_GUSBCFG_FDMOD;
}

static usbd_device *otgfs_usb_init(void) {
	printf("otgfs_usb_init()\r\n");
	rcc_periph_clock_enable(RCC_OTGFS);

	otgfs_usb_core_init();
	otgfs_usb_device_init();

	OTG_FS_GCCFG |= OTG_GCCFG_PWRDWN;

	// OTG_FS_GUSBCFG |= OTG_GUSBCFG_PHYSEL;

	/* Wait for AHB idle. */
	// while (!(OTG_FS_GRSTCTL & OTG_GRSTCTL_AHBIDL));
	/* Do core soft reset. */
	// OTG_FS_GRSTCTL |= OTG_GRSTCTL_CSRST;
	// while (OTG_FS_GRSTCTL & OTG_GRSTCTL_CSRST);

	/* Enable VBUS sensing in device mode and power up the PHY. */
	// OTG_FS_GCCFG |= OTG_GCCFG_VBUSBSEN | OTG_GCCFG_PWRDWN;

	/* Explicitly enable DP pullup (not all cores do this by default) */
	// OTG_FS_DCTL &= ~OTG_DCTL_SDIS;

	/* Force peripheral only mode. */
	// OTG_FS_GUSBCFG |= OTG_GUSBCFG_FDMOD;

	/* Restart the PHY clock. */
	// OTG_FS_PCGCCTL = 0;

	// IN endpoints
	// OTG_FS_GINTMSK |= OTG_GINTMSK_IEPINT;
	// USB Suspend
	// OTG_FS_GINTMSK |= OTG_GINTMSK_USBSUSPM;
	// Resume/remote wakeup detected interrupt mask
	// OTG_FS_GINTMSK |= OTG_GINTMSK_WUIM;

	// OTG_FS_DAINTMSK = 0xF;
	// OTG_FS_DIEPMSK = OTG_DIEPMSK_XFRCM;

	return &_usbd_dev;
}

// otgfs_usb_set_address()

static void otgfs_usb_set_address(usbd_device *usbd_dev, uint8_t addr) {
	(void)usbd_dev;
	(void)addr;
	printf("otgfs_usb_set_address()\r\n");
	printf("  TODO\r\n");
	while(true);
	return;
	REBASE(OTG_DCFG) = (REBASE(OTG_DCFG) & ~OTG_DCFG_DAD) | (addr << 4);
}

// otgfs_usb_ep_setup()

static void otgfs_usb_ep_setup(
	usbd_device *usbd_dev,
	uint8_t addr,
	uint8_t type,
	uint16_t max_size,
	void (*callback) (usbd_device *usbd_dev, uint8_t ep)
) {
	printf("otgfs_usb_ep_setup()\r\n");
	printf("  TODO\r\n");
	while(true);
	return;
	/*
	 * Configure endpoint address and type. Allocate FIFO memory for
	 * endpoint. Install callback function.
	 */
	uint8_t dir = addr & 0x80;
	addr &= 0x7f;

	if (addr == 0) { /* For the default control endpoint */
		/* Configure IN part. */
		if (max_size >= 64) {
			REBASE(OTG_DIEPCTL0) = OTG_DIEPCTL0_MPSIZ_64;
		} else if (max_size >= 32) {
			REBASE(OTG_DIEPCTL0) = OTG_DIEPCTL0_MPSIZ_32;
		} else if (max_size >= 16) {
			REBASE(OTG_DIEPCTL0) = OTG_DIEPCTL0_MPSIZ_16;
		} else {
			REBASE(OTG_DIEPCTL0) = OTG_DIEPCTL0_MPSIZ_8;
		}

		REBASE(OTG_DIEPTSIZ0) =
			(max_size & OTG_DIEPSIZ0_XFRSIZ_MASK);
		REBASE(OTG_DIEPCTL0) |=
			OTG_DIEPCTL0_EPENA | OTG_DIEPCTL0_SNAK;

		/* Configure OUT part. */
		usbd_dev->doeptsiz[0] = OTG_DIEPSIZ0_STUPCNT_1 |
			OTG_DIEPSIZ0_PKTCNT |
			(max_size & OTG_DIEPSIZ0_XFRSIZ_MASK);
		REBASE(OTG_DOEPTSIZ(0)) = usbd_dev->doeptsiz[0];
		REBASE(OTG_DOEPCTL(0)) |=
		    OTG_DOEPCTL0_EPENA | OTG_DIEPCTL0_SNAK;

		REBASE(OTG_GNPTXFSIZ) = ((max_size / 4) << 16) |
					 usbd_dev->driver->rx_fifo_size;
		usbd_dev->fifo_mem_top += max_size / 4;
		usbd_dev->fifo_mem_top_ep0 = usbd_dev->fifo_mem_top;

		return;
	}

	if (dir) {
		REBASE(OTG_DIEPTXF(addr)) = ((max_size / 4) << 16) |
					     usbd_dev->fifo_mem_top;
		usbd_dev->fifo_mem_top += max_size / 4;

		REBASE(OTG_DIEPTSIZ(addr)) =
		    (max_size & OTG_DIEPSIZ0_XFRSIZ_MASK);
		REBASE(OTG_DIEPCTL(addr)) |=
		    OTG_DIEPCTL0_EPENA | OTG_DIEPCTL0_SNAK | (type << 18)
		    | OTG_DIEPCTL0_USBAEP | OTG_DIEPCTLX_SD0PID
		    | (addr << 22) | max_size;

		if (callback) {
			usbd_dev->user_callback_ctr[addr][USB_TRANSACTION_IN] =
			    (void *)callback;
		}
	}

	if (!dir) {
		usbd_dev->doeptsiz[addr] = OTG_DIEPSIZ0_PKTCNT |
				 (max_size & OTG_DIEPSIZ0_XFRSIZ_MASK);
		REBASE(OTG_DOEPTSIZ(addr)) = usbd_dev->doeptsiz[addr];
		REBASE(OTG_DOEPCTL(addr)) |= OTG_DOEPCTL0_EPENA |
		    OTG_DOEPCTL0_USBAEP | OTG_DIEPCTL0_CNAK |
		    OTG_DOEPCTLX_SD0PID | (type << 18) | max_size;

		if (callback) {
			usbd_dev->user_callback_ctr[addr][USB_TRANSACTION_OUT] =
			    (void *)callback;
		}
	}
}

// otgfs_usb_reset()

static void otgfs_usb_reset(usbd_device *usbd_dev) {
	printf("otgfs_usb_reset()\r\n");
	printf("  TODO\r\n");
	while(true);
	return;
	int i;
	/* The core resets the endpoints automatically on reset. */
	usbd_dev->fifo_mem_top = usbd_dev->fifo_mem_top_ep0;

	/* Disable any currently active endpoints */
	for (i = 1; i < 4; i++) {
		if (REBASE(OTG_DOEPCTL(i)) & OTG_DOEPCTL0_EPENA) {
			REBASE(OTG_DOEPCTL(i)) |= OTG_DOEPCTL0_EPDIS;
		}
		if (REBASE(OTG_DIEPCTL(i)) & OTG_DIEPCTL0_EPENA) {
			REBASE(OTG_DIEPCTL(i)) |= OTG_DIEPCTL0_EPDIS;
		}
	}

	/* Flush all tx/rx fifos */
	REBASE(OTG_GRSTCTL) = OTG_GRSTCTL_TXFFLSH | OTG_GRSTCTL_TXFNUM_ALL
			      | OTG_GRSTCTL_RXFFLSH;
}

// otgfs_usb_ep_stall_set()

static void otgfs_usb_ep_stall_set(usbd_device *usbd_dev, uint8_t addr, uint8_t stall) {
	(void)usbd_dev;
	printf("otgfs_usb_ep_stall_set()\r\n");
	printf("  TODO\r\n");
	while(true);
	return;
	if (addr == 0) {
		if (stall) {
			REBASE(OTG_DIEPCTL(addr)) |= OTG_DIEPCTL0_STALL;
		} else {
			REBASE(OTG_DIEPCTL(addr)) &= ~OTG_DIEPCTL0_STALL;
		}
	}

	if (addr & 0x80) {
		addr &= 0x7F;

		if (stall) {
			REBASE(OTG_DIEPCTL(addr)) |= OTG_DIEPCTL0_STALL;
		} else {
			REBASE(OTG_DIEPCTL(addr)) &= ~OTG_DIEPCTL0_STALL;
			REBASE(OTG_DIEPCTL(addr)) |= OTG_DIEPCTLX_SD0PID;
		}
	} else {
		if (stall) {
			REBASE(OTG_DOEPCTL(addr)) |= OTG_DOEPCTL0_STALL;
		} else {
			REBASE(OTG_DOEPCTL(addr)) &= ~OTG_DOEPCTL0_STALL;
			REBASE(OTG_DOEPCTL(addr)) |= OTG_DOEPCTLX_SD0PID;
		}
	}
}

// otgfs_usb_ep_stall_get()

static uint8_t otgfs_usb_ep_stall_get(usbd_device *usbd_dev, uint8_t addr) {
	(void)usbd_dev;
	printf("otgfs_usb_ep_stall_get()\r\n");
	printf("  TODO\r\n");
	while(true);
	return 0;
	/* Return non-zero if STALL set. */
	if (addr & 0x80) {
		return (REBASE(OTG_DIEPCTL(addr & 0x7f)) &
				OTG_DIEPCTL0_STALL) ? 1 : 0;
	} else {
		return (REBASE(OTG_DOEPCTL(addr)) &
				OTG_DOEPCTL0_STALL) ? 1 : 0;
	}
}

// otgfs_usb_ep_nak_set()

static void otgfs_usb_ep_nak_set(usbd_device *usbd_dev, uint8_t addr, uint8_t nak) {
	printf("otgfs_usb_ep_nak_set()\r\n");
	printf("  TODO\r\n");
	while(true);
	return;
	/* It does not make sense to force NAK on IN endpoints. */
	if (addr & 0x80) {
		return;
	}

	usbd_dev->force_nak[addr] = nak;

	if (nak) {
		REBASE(OTG_DOEPCTL(addr)) |= OTG_DOEPCTL0_SNAK;
	} else {
		REBASE(OTG_DOEPCTL(addr)) |= OTG_DOEPCTL0_CNAK;
	}
}

// otgfs_usb_ep_write_packet()

static uint16_t otgfs_usb_ep_write_packet(
	usbd_device *usbd_dev,
	uint8_t addr,
	const void *buf, uint16_t len
) {
	(void)usbd_dev;
	printf("otgfs_usb_ep_write_packet()\r\n");
	printf("  TODO\r\n");
	while(true);
	return 0;
	const uint32_t *buf32 = buf;
	#if defined(__ARM_ARCH_6M__)
	const uint8_t *buf8 = buf;
	uint32_t word32;
	#endif /* defined(__ARM_ARCH_6M__) */
	int i;

	addr &= 0x7F;

	/* Return if endpoint is already enabled. */
	if (REBASE(OTG_DIEPTSIZ(addr)) & OTG_DIEPSIZ0_PKTCNT) {
		return 0;
	}

	/* Enable endpoint for transmission. */
	REBASE(OTG_DIEPTSIZ(addr)) = OTG_DIEPSIZ0_PKTCNT | len;
	REBASE(OTG_DIEPCTL(addr)) |= OTG_DIEPCTL0_EPENA |
				     OTG_DIEPCTL0_CNAK;

	/* Copy buffer to endpoint FIFO, note - memcpy does not work.
	 * ARMv7M supports non-word-aligned accesses, ARMv6M does not. */
	#if defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__)
	for (i = len; i > 0; i -= 4) {
		REBASE(OTG_FIFO(addr)) = *buf32++;
	}
	#endif /* defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__) */

	#if defined(__ARM_ARCH_6M__)
	/* Take care of word-aligned and non-word-aligned buffers */
	if (((uint32_t)buf8 & 0x3) == 0) {
		for (i = len; i > 0; i -= 4) {
			REBASE(OTG_FIFO(addr)) = *buf32++;
		}
	} else {
		for (i = len; i > 0; i -= 4) {
			memcpy(&word32, buf8, 4);
			REBASE(OTG_FIFO(addr)) = word32;
			buf8 += 4;
		}
	}
	#endif /* defined(__ARM_ARCH_6M__) */

	return len;
}

// otgfs_usb_ep_read_packet()

static uint16_t otgfs_usb_ep_read_packet(
	usbd_device *usbd_dev,
	uint8_t addr,
	void *buf,
	uint16_t len
) {
	printf("otgfs_usb_ep_read_packet()\r\n");
	printf("  TODO\r\n");
	while(true);
	return 0;
	int i;
	uint32_t *buf32 = buf;
	#if defined(__ARM_ARCH_6M__)
	uint8_t *buf8 = buf;
	uint32_t word32;
	#endif /* defined(__ARM_ARCH_6M__) */
	uint32_t extra;

	/* We do not need to know the endpoint address since there is only one
	 * receive FIFO for all endpoints.
	 */
	(void) addr;
	len = MIN(len, usbd_dev->rxbcnt);

	/* ARMv7M supports non-word-aligned accesses, ARMv6M does not. */
	#if defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__)
	for (i = len; i >= 4; i -= 4) {
		*buf32++ = REBASE(OTG_FIFO(0));
		usbd_dev->rxbcnt -= 4;
	}
	#endif /* defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__) */

	#if defined(__ARM_ARCH_6M__)
	/* Take care of word-aligned and non-word-aligned buffers */
	if (((uint32_t)buf8 & 0x3) == 0) {
		for (i = len; i >= 4; i -= 4) {
			*buf32++ = REBASE(OTG_FIFO(0));
			usbd_dev->rxbcnt -= 4;
		}
	} else {
		for (i = len; i >= 4; i -= 4) {
			word32 = REBASE(OTG_FIFO(0));
			memcpy(buf8, &word32, 4);
			usbd_dev->rxbcnt -= 4;
			buf8 += 4;
		}
		/* buf32 needs to be updated as it is used for extra */
		buf32 = (uint32_t *)buf8;
	}
	#endif /* defined(__ARM_ARCH_6M__) */

	if (i) {
		extra = REBASE(OTG_FIFO(0));
		/* we read 4 bytes from the fifo, so update rxbcnt */
		if (usbd_dev->rxbcnt < 4) {
			/* Be careful not to underflow (rxbcnt is unsigned) */
			usbd_dev->rxbcnt = 0;
		} else {
			usbd_dev->rxbcnt -= 4;
		}
		memcpy(buf32, &extra, i);
	}

	return len;
}

// otgfs_usb_poll()

static void otgfs_usb_endpoint_init_on_usb_reset(void) {
	printf("    otgfs_usb_endpoint_init_on_usb_reset()\r\n");
	// 22.17.5 Device programming model
	//
	// Endpoint initialization on USB reset
	//
	// 1. Set the NAK bit for all OUT endpoints
	//   – SNAK = 1 in OTG_FS_DOEPCTLx (for all OUT endpoints)
	for(int i=1 ; i <=3 ; i++)
		OTG_FS_DOEPCTL(i) |= OTG_DIEPCTL0_SNAK;
	// 2. Unmask the following interrupt bits
	//   – INEP0 = 1 in OTG_FS_DAINTMSK (control 0 IN endpoint)
	OTG_FS_DAINTMSK |= (1<<0);
	//   – OUTEP0 = 1 in OTG_FS_DAINTMSK (control 0 OUT endpoint)
	OTG_FS_DAINTMSK |= (1<<16);
	//   – STUP = 1 in DOEPMSK
	OTG_FS_DOEPMSK |= OTG_DOEPMSK_STUPM;
	//   – XFRC = 1 in DOEPMSK
	OTG_FS_DOEPMSK |= OTG_DOEPMSK_XFRCM;
	//   – XFRC = 1 in DIEPMSK
	OTG_FS_DIEPMSK |= OTG_DIEPMSK_XFRCM;
	//   – TOC = 1 in DIEPMSK
	OTG_FS_DIEPMSK |= OTG_DIEPMSK_TOM;
	// 3. Set up the Data FIFO RAM for each of the FIFOs
	//   – Program the OTG_FS_GRXFSIZ register, to be able to receive control
	//     OUT data and setup data. If thresholding is not enabled, at a
	//     minimum, this must be equal to 1 max packet size of control endpoint
	//     0 + 2 words (for the status of the control OUT data packet) + 10
	//     words (for setup packets).
	OTG_FS_GRXFSIZ = otgfs_usb_driver.rx_fifo_size & 0xFF;
	//   – Program the OTG_FS_TX0FSIZ register (depending on the FIFO number
	//     chosen) to be able to transmit control IN data. At a minimum, this
	//     must be equal to 1 max packet size of control endpoint 0.
	// TODO https://github.com/wookey-project/driver-stm32f4xx-usb/blob/master/stm32f4xx_usb_fs.c#L1216
	// 4. Program the following fields in the endpoint-specific registers for
	//   control OUT endpoint 0 to receive a SETUP packet
	//   – STUPCNT = 3 in OTG_FS_DOEPTSIZ0 (to receive up to 3 back-to-back
	//     SETUP packets)
	OTG_FS_DOEPTSIZ0 |= OTG_DIEPSIZ0_STUPCNT_3;
	// At this point, all initialization required to receive SETUP packets is done.

	_usbd_dev.current_address = 0;
	_usbd_dev.current_config = 0;
	// _usbd_dev.fifo_mem_top = otgfs_usb_driver.rx_fifo_size;
}

static void otgfs_usb_endpoint_init_on_enumeration_completion(void) {
	printf("    otgfs_usb_endpoint_init_on_enumeration_completion()\r\n");
	uint16_t max_size;
	// 22.17.5 Device programming model
	//
	// Endpoint initialization on enumeration completion
	//
	// 1. On the Enumeration Done interrupt (ENUMDNE in OTG_FS_GINTSTS), read
	// the OTG_FS_DSTS register to determine the enumeration speed.
	// 2. Program the MPSIZ field in OTG_FS_DIEPCTL0 to set the maximum packet
	// size. This step configures control endpoint 0. The maximum packet size
	// for a control endpoint depends on the enumeration speed.
	max_size = _usbd_dev.desc->bMaxPacketSize0;
	if (max_size >= 64) {
		REBASE(OTG_DIEPCTL0) = OTG_DIEPCTL0_MPSIZ_64;
	} else if (max_size >= 32) {
		REBASE(OTG_DIEPCTL0) = OTG_DIEPCTL0_MPSIZ_32;
	} else if (max_size >= 16) {
		REBASE(OTG_DIEPCTL0) = OTG_DIEPCTL0_MPSIZ_16;
	} else {
		REBASE(OTG_DIEPCTL0) = OTG_DIEPCTL0_MPSIZ_8;
	}
}

static void otgfs_usb_packet_read(void) {
	uint32_t grxstsp;
	uint8_t ep;
	printf("    otgfs_usb_packet_read()\r\n");

	// 22.17.6 Operational model

	// SETUP and OUT data transfers
	// This section describes the internal data flow and application-level
	// operations during data OUT transfers and SETUP transactions.
	// • Packet read
	// This section describes how to read packets (OUT data and SETUP packets)
	// from the receive FIFO.
	// 1. On catching an RXFLVL interrupt (OTG_FS_GINTSTS register), the
	//   application must read the Receive status pop register (OTG_FS_GRXSTSP).
	grxstsp = OTG_FS_GRXSTSP;
	ep = grxstsp & OTG_GRXSTSP_EPNUM_MASK;
	// 2. The application can mask the RXFLVL interrupt (in OTG_FS_GINTSTS) by
	//   writing to RXFLVL = 0 (in OTG_FS_GINTMSK), until it has read the packet
	//   from the receive FIFO.
	OTG_FS_GINTMSK &= ~(OTG_GINTMSK_RXFLVLM);
	// 3. If the received packet’s byte count is not 0, the byte count amount of
	//   data is popped from the receive Data FIFO and stored in memory. If the
	//   received packet byte count is 0, no data is popped from the receive data
	//   FIFO.
	_usbd_dev.rxbcnt = (grxstsp & OTG_GRXSTSP_BCNT_MASK) >> 4;
	// 4. The receive FIFO’s packet status readout indicates one of the
	//   following:
	switch(grxstsp & OTG_GRXSTSP_PKTSTS_MASK) {
		// 	  a) Global OUT NAK pattern:
		//      PKTSTS = Global OUT NAK,
		//      DPID = Don’t Care (0b00).
		//      BCNT = 0x000,
		//      EPNUM = Don’t Care (0x0),
		//      These data indicate that the global OUT NAK bit has taken effect.
		case OTG_GRXSTSP_PKTSTS_GOUTNAK:
			printf("      OTG_GRXSTSP_PKTSTS_GOUTNAK\r\n");
			printf("  TODO\r\n");
			while(true);
			break;
		// 	  b) SETUP packet pattern:
		//      PKTSTS = SETUP,
		//      DPID = D0.
		//      BCNT = 0x008,
		//      EPNUM = Control EP Num,
		//      These data indicate that a SETUP packet for the specified endpoint
		//      is now available for reading from the receive FIFO.
		case OTG_GRXSTSP_PKTSTS_SETUP:
			printf("      OTG_GRXSTSP_PKTSTS_SETUP\r\n");
			otgfs_usb_ep_read_packet(&_usbd_dev, ep, &_usbd_dev.control_state.req, 8);
			break;
		//    c) Setup stage done pattern:
		//      PKTSTS = Setup Stage Done,
		//      DPID = Don’t Care (0b00).
		//      BCNT = 0x0,
		//      EPNUM = Control EP Num,
		//      These data indicate that the Setup stage for the specified endpoint
		//      has completed and the Data stage has started. After this entry is
		//      popped from the receive FIFO, the core asserts a Setup interrupt on
		//      the specified control OUT endpoint.
		case OTG_GRXSTSP_PKTSTS_SETUP_COMP:
			printf("      OTG_GRXSTSP_PKTSTS_SETUP_COMP\r\n");
			printf("  TODO\r\n");
			while(true);
			break;
		//    d) Data OUT packet pattern:
		//      PKTSTS = DataOUT,
		//      DPID = Actual Data PID.
		//      BCNT = size of the received data OUT packet (0 ≤ BCNT ≤ 1 024),
		//      EPNUM = EPNUM on which the packet was received,
		case OTG_GRXSTSP_PKTSTS_OUT:
			printf("      OTG_GRXSTSP_PKTSTS_OUT\r\n");
			printf("  TODO\r\n");
			while(true);
			break;
		//    e) Data transfer completed pattern:
		//      PKTSTS = Data OUT Transfer Done,
		//      DPID = Don’t Care (0b00).
		//      BCNT = 0x0,
		//      EPNUM = OUT EP Num on which the data transfer is complete,
		//      These data indicate that an OUT data transfer for the specified OUT
		//      endpoint has completed. After this entry is popped from the receive
		//      FIFO, the core asserts a Transfer Completed interrupt on the
		//      specified OUT endpoint.
		case OTG_GRXSTSP_PKTSTS_OUT_COMP:
			printf("      OTG_GRXSTSP_PKTSTS_OUT_COMP\r\n");
			printf("  TODO\r\n");
			while(true);
			break;
	}
	// 5. After the data payload is popped from the receive FIFO, the RXFLVL
	//   interrupt (OTG_FS_GINTSTS) must be unmasked.
	OTG_FS_GINTMSK |= OTG_GINTMSK_RXFLVLM;
	// 6. Steps 1–5 are repeated every time the application detects assertion of
	//   the interrupt line due to RXFLVL in OTG_FS_GINTSTS. Reading an empty
	//   receive FIFO can result in undefined core behavior.
}

static void otgfs_usb_flush_txfifo(usbd_device *usbd_dev, int ep) {
	(void)usbd_dev;
	printf("otgfs_usb_flush_txfifo()\r\n");
	printf("  TODO\r\n");
	while(true);
	return;
	uint32_t fifo;
	/* set IN endpoint NAK */
	REBASE(OTG_DIEPCTL(ep)) |= OTG_DIEPCTL0_SNAK;
	/* wait for core to respond */
	while (!(REBASE(OTG_DIEPINT(ep)) & OTG_DIEPINTX_INEPNE)) {
		/* idle */
	}
	/* get fifo for this endpoint */
	fifo = (REBASE(OTG_DIEPCTL(ep)) & OTG_DIEPCTL0_TXFNUM_MASK) >> 22;
	/* wait for core to idle */
	while (!(REBASE(OTG_GRSTCTL) & OTG_GRSTCTL_AHBIDL)) {
		/* idle */
	}
	/* flush tx fifo */
	REBASE(OTG_GRSTCTL) = (fifo << 6) | OTG_GRSTCTL_TXFFLSH;
	/* reset packet counter */
	REBASE(OTG_DIEPTSIZ(ep)) = 0;
	while ((REBASE(OTG_GRSTCTL) & OTG_GRSTCTL_TXFFLSH)) {
		/* idle */
	}
}

static void otgfs_usb_poll(usbd_device *usbd_dev) {
	uint32_t gintsts;
	int i;

	// printf("otgfs_usb_poll()\r\n");
	gintsts = OTG_FS_GINTSTS;


	// 22.17.3
	//
	// 4. Wait for the USBRST interrupt in OTG_FS_GINTSTS. It indicates that a
	// reset has been detected on the USB that lasts for about 10 ms on
	// receiving this interrupt.
	if(gintsts & OTG_GINTSTS_USBRST) {
		printf("  OTG_GINTSTS_USBRST\r\n");
		otgfs_usb_endpoint_init_on_usb_reset();
		OTG_FS_GINTSTS |= OTG_GINTSTS_USBRST;
		return;
	}

	// 22.17.3
	//
	// Wait for the ENUMDNE interrupt in OTG_FS_GINTSTS. This interrupt
	// indicates the end of reset on the USB. On receiving this interrupt, the
	// application must read the OTG_FS_DSTS register to determine the
	// enumeration speed and perform the steps listed in Endpoint initialization
	// on enumeration completion on page 781.
	if (gintsts & OTG_GINTSTS_ENUMDNE) {
		printf("  OTG_GINTSTS_ENUMDNE\r\n");
		otgfs_usb_endpoint_init_on_enumeration_completion();
		// _usbd_dev->driver->set_address(usbd_dev, 0);
		if (usbd_dev->user_callback_reset) {
			usbd_dev->user_callback_reset();
		}
		OTG_FS_GINTSTS |= OTG_GINTSTS_ENUMDNE;
		return;
	}

	// At this point, the device is ready to accept SOF packets and perform
	// control transfers on control endpoint 0.

	// 22.17.6 Operational model

	// SETUP and OUT data transfers
	// This section describes the internal data flow and application-level
	// operations during data OUT transfers and SETUP transactions.
	// • Packet read
	if (gintsts & OTG_GINTSTS_RXFLVL) {
		printf("  OTG_GINTSTS_RXFLVL\r\n");
		otgfs_usb_packet_read();
		return;
	}

	return;
	/////////////////////////////////////

	/*
	 * There is no global interrupt flag for transmit complete.
	 * The XFRC bit must be checked in each OTG_DIEPINT(x).
	 */
	for (i = 0; i < 4; i++) { /* Iterate over endpoints. */
		if (REBASE(OTG_DIEPINT(i)) & OTG_DIEPINTX_XFRC) {
			/* Transfer complete. */
			if (usbd_dev->user_callback_ctr[i]
						       [USB_TRANSACTION_IN]) {
				usbd_dev->user_callback_ctr[i]
					[USB_TRANSACTION_IN](usbd_dev, i);
			}

			REBASE(OTG_DIEPINT(i)) = OTG_DIEPINTX_XFRC;
		}
	}

	/* Note: RX and TX handled differently in this device. */
	if (gintsts & OTG_GINTSTS_RXFLVL) {
		/* Receive FIFO non-empty. */
		uint32_t rxstsp = REBASE(OTG_GRXSTSP);
		uint32_t pktsts = rxstsp & OTG_GRXSTSP_PKTSTS_MASK;
		uint8_t ep = rxstsp & OTG_GRXSTSP_EPNUM_MASK;

		if (pktsts == OTG_GRXSTSP_PKTSTS_SETUP_COMP) {
			usbd_dev->user_callback_ctr[ep][USB_TRANSACTION_SETUP] (usbd_dev, ep);
		}

		if (pktsts == OTG_GRXSTSP_PKTSTS_OUT_COMP
			|| pktsts == OTG_GRXSTSP_PKTSTS_SETUP_COMP)  {
			REBASE(OTG_DOEPTSIZ(ep)) = usbd_dev->doeptsiz[ep];
			REBASE(OTG_DOEPCTL(ep)) |= OTG_DOEPCTL0_EPENA |
				(usbd_dev->force_nak[ep] ?
				 OTG_DOEPCTL0_SNAK : OTG_DOEPCTL0_CNAK);
			return;
		}

		if ((pktsts != OTG_GRXSTSP_PKTSTS_OUT) &&
		    (pktsts != OTG_GRXSTSP_PKTSTS_SETUP)) {
			return;
		}

		uint8_t type;
		if (pktsts == OTG_GRXSTSP_PKTSTS_SETUP) {
			type = USB_TRANSACTION_SETUP;
		} else {
			type = USB_TRANSACTION_OUT;
		}

		if (type == USB_TRANSACTION_SETUP
			&& (REBASE(OTG_DIEPTSIZ(ep)) & OTG_DIEPSIZ0_PKTCNT)) {
			/* SETUP received but there is still something stuck
			 * in the transmit fifo.  Flush it.
			 */
			otgfs_usb_flush_txfifo(usbd_dev, ep);
		}

		/* Save packet size for otgfs_usb_ep_read_packet(). */
		usbd_dev->rxbcnt = (rxstsp & OTG_GRXSTSP_BCNT_MASK) >> 4;

		if (type == USB_TRANSACTION_SETUP) {
			otgfs_usb_ep_read_packet(usbd_dev, ep, &usbd_dev->control_state.req, 8);
		} else if (usbd_dev->user_callback_ctr[ep][type]) {
			usbd_dev->user_callback_ctr[ep][type] (usbd_dev, ep);
		}

		/* Discard unread packet data. */
		for (i = 0; i < usbd_dev->rxbcnt; i += 4) {
			/* There is only one receive FIFO, so use OTG_FIFO(0) */
			(void)REBASE(OTG_FIFO(0));
		}

		usbd_dev->rxbcnt = 0;
	}

	if (gintsts & OTG_GINTSTS_USBSUSP) {
		if (usbd_dev->user_callback_suspend) {
			usbd_dev->user_callback_suspend();
		}
		REBASE(OTG_GINTSTS) = OTG_GINTSTS_USBSUSP;
	}

	if (gintsts & OTG_GINTSTS_WKUPINT) {
		if (usbd_dev->user_callback_resume) {
			usbd_dev->user_callback_resume();
		}
		REBASE(OTG_GINTSTS) = OTG_GINTSTS_WKUPINT;
	}

	if (gintsts & OTG_GINTSTS_SOF) {
		if (usbd_dev->user_callback_sof) {
			usbd_dev->user_callback_sof();
		}
		REBASE(OTG_GINTSTS) = OTG_GINTSTS_SOF;
	}

	if (usbd_dev->user_callback_sof) {
		REBASE(OTG_GINTMSK) |= OTG_GINTMSK_SOFM;
	} else {
		REBASE(OTG_GINTMSK) &= ~OTG_GINTMSK_SOFM;
	}
}

// otgfs_usb_disconnect()

static void otgfs_usb_disconnect(usbd_device *usbd_dev, bool disconnected) {
	(void)usbd_dev;
	(void)disconnected;
	printf("otgfs_usb_disconnect()\r\n");
	printf("  TODO\r\n");
	while(true);
	return;
	if (disconnected) {
		REBASE(OTG_DCTL) |= OTG_DCTL_SDIS;
	} else {
		REBASE(OTG_DCTL) &= ~OTG_DCTL_SDIS;
	}
}