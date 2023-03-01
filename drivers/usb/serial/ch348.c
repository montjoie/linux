// SPDX-License-Identifier: GPL-2.0
/*
 * USB serial driver for USB to Octal UARTs chip ch348.
 *
 * Copyright (C) 2022 Corentin Labbe <clabbe@baylibre.com>
 * With the help of Neil Armstrong <neil.armstrong@linaro.org>
 */

#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/serial.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/usb.h>
#include <linux/usb/serial.h>

#define DEFAULT_BAUD_RATE 9600
#define CH348_CMD_TIMEOUT   2000

#define CH348_CTO_D	0x01
#define CH348_CTO_R	0x02

#define CH348_CTI_C	0x10
#define CH348_CTI_DSR	0x20
#define CH348_CTI_R	0x40
#define CH348_CTI_DCD	0x80

#define CH348_LO	0x02
#define CH348_LP	0x04
#define CH348_LF	0x08
#define CH348_LB	0x10

#define CMD_W_R		0xC0
#define CMD_W_BR	0x80

#define CMD_WB_E	0x90
#define CMD_RB_E	0xC0

#define M_NOR		0x00
#define M_HF		0x03

#define R_MOD		0x97
#define R_IO_D		0x98
#define R_IO_O		0x99
#define R_IO_I		0x9b
#define R_TM_O		0x9c
#define R_INIT		0xa1

#define R_C1		0x01
#define R_C2		0x02
#define R_C4		0x04
#define R_C5		0x06

#define R_II_B1		0x06
#define R_II_B2		0x02
#define R_II_B3		0x00

#define CH348_RX_PORT_CHUNK_LENGTH	32
#define CH348_RX_PORT_MAX_LENGTH	30

struct ch348_rxbuf {
	u8 port;
	u8 length;
	u8 data[CH348_RX_PORT_MAX_LENGTH];
} __packed;

struct ch348_txbuf {
	u8 port;
	__le16 length;
	u8 data[];
} __packed;

#define CH348_TX_HDRSIZE offsetof(struct ch348_txbuf, data)

struct ch348_initbuf {
	u8 cmd;
	u8 reg;
	u8 port;
	__be32 baudrate;
	u8 format;
	u8 paritytype;
	u8 databits;
	u8 rate;
	u8 unknown;
} __packed;

#define CH348_MAXPORT 8

/*
 * The CH348 multiplexes rx & tx into a pair of Bulk USB endpoints for
 * the 8 serial ports, and another pair of Bulk USB endpoints to
 * set port settings and receive port status events.
 *
 * The USB serial cores ties every Bulk endpoints pairs to each ports,
 * but in our case it will set port 0 with the rx/tx endpoints
 * and port 1 with the setup/status endpoints.
 *
 * To still take advantage of the generic code, we (re-)initialize
 * the USB serial port structure with the correct USB endpoint
 * for read and write, and write proper process_read_urb()
 * and prepare_write_buffer() to correctly (de-)multiplex data.
 */

/*
 * struct ch348_port - per-port information
 * @uartmode:		UART port current mode
 */
struct ch348_port {
	u8 uartmode;
};

/*
 * struct ch348 - main container for all this driver information
 * @udev:		pointer to the CH348 USB device
 * @ports:		List of per-port information
 * @serial:		pointer to the serial structure
 * @status_ep:		endpoint number for status operations
 * @cmd_ep:		endpoint number for configure operations
 * @status_urb:		URB for status
 * @status_buffer:	buffer used by status_urb
 */
struct ch348 {
	struct usb_device *udev;
	struct ch348_port ports[CH348_MAXPORT];
	struct usb_serial *serial;

	int status_ep;
	int cmd_ep;

	struct urb *status_urb;
	u8 *status_buffer;
};

struct ch348_magic {
	u8 action;
	u8 reg;
	u8 control;
} __packed;

/*
 * Some values came from vendor tree, and we have no meaning for them, this
 * function simply use them.
 */
static int ch348_do_magic(struct ch348 *ch348, int portnum, u8 action, u8 reg, u8 control)
{
	struct ch348_magic *buffer;
	int ret, len;

	buffer = kzalloc(sizeof(*buffer), GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	if (portnum < 4)
		reg += 0x10 * portnum;
	else
		reg += 0x10 * (portnum - 4) + 0x08;

	buffer->action = action;
	buffer->reg = reg;
	buffer->control = control;

	ret = usb_bulk_msg(ch348->udev, ch348->cmd_ep, buffer, 3, &len,
			   CH348_CMD_TIMEOUT);
	if (ret)
		dev_err(&ch348->udev->dev, "Failed to write magic err=%d\n", ret);

	kfree(buffer);

	return ret;
}

static int ch348_configure(struct ch348 *ch348, int portnum)
{
	int ret;

	ret = ch348_do_magic(ch348, portnum, CMD_W_R, R_C2, 0x87);
	if (ret)
		return ret;

	return ch348_do_magic(ch348, portnum, CMD_W_R, R_C4, 0x08);
}

static void ch348_process_read_urb(struct urb *urb)
{
	struct usb_serial_port *port = urb->context;
	struct ch348 *ch348 = usb_get_serial_data(port->serial);
	unsigned int portnum, usblen;
	struct ch348_rxbuf *rxb;
	u8 *buffer, *end;

	buffer = urb->transfer_buffer;

	if (urb->actual_length < 2) {
		dev_dbg(&port->dev, "Empty rx buffer\n");
		return;
	}

	end = buffer + urb->actual_length;

	for (; buffer < end; buffer += CH348_RX_PORT_CHUNK_LENGTH) {
		rxb = (struct ch348_rxbuf *)buffer;
		portnum = rxb->port;
		if (portnum >= CH348_MAXPORT) {
			dev_dbg(&port->dev, "Invalid port %d\n", portnum);
			break;
		}

		port = ch348->serial->port[portnum];

		usblen = rxb->length;
		if (usblen > CH348_RX_PORT_MAX_LENGTH) {
			dev_dbg(&port->dev, "Invalid length %d for port %d\n",
				usblen, portnum);
			break;
		}

		tty_insert_flip_string(&port->port, rxb->data, usblen);
		tty_flip_buffer_push(&port->port);
		port->icount.rx += usblen;
		usb_serial_debug_data(&port->dev, __func__, usblen, rxb->data);
	}
}

static int ch348_prepare_write_buffer(struct usb_serial_port *port, void *dest, size_t size)
{
	struct ch348_txbuf *rxt = dest;
	int count;

	count = kfifo_out_locked(&port->write_fifo, rxt->data,
				 size - CH348_TX_HDRSIZE, &port->lock);

	rxt->port = port->port_number;
	rxt->length = cpu_to_le16(count);

	return count + CH348_TX_HDRSIZE;
}

static int ch348_set_uartmode(struct ch348 *ch348, int portnum, u8 index, u8 mode)
{
	int ret;

	if (ch348->ports[portnum].uartmode == M_NOR && mode == M_HF) {
		ret = ch348_do_magic(ch348, portnum, CMD_W_BR, R_C4, 0x51);
		if (ret)
			return ret;
		ch348->ports[portnum].uartmode = M_HF;
	}

	if (ch348->ports[portnum].uartmode == M_HF && mode == M_NOR) {
		ret = ch348_do_magic(ch348, portnum, CMD_W_BR, R_C4, 0x50);
		if (ret)
			return ret;
		ch348->ports[portnum].uartmode = M_NOR;
	}
	return 0;
}

static void ch348_set_termios(struct tty_struct *tty, struct usb_serial_port *port,
			      const struct ktermios *termios_old)
{
	struct ch348 *ch348 = usb_get_serial_data(port->serial);
	int portnum = port->port_number;
	struct ktermios *termios = &tty->termios;
	int ret, sent;
	speed_t	baudrate;
	u8 format;
	struct ch348_initbuf *buffer;

	if (termios_old && !tty_termios_hw_change(&tty->termios, termios_old))
		return;

	buffer = kzalloc(sizeof(*buffer), GFP_KERNEL);
	if (!buffer) {
		if (termios_old)
			tty->termios = *termios_old;
		return;
	}

	baudrate = tty_get_baud_rate(tty);
	/* test show no success on low baud and datasheet said it is not supported */
	if (baudrate < 1200)
		baudrate = DEFAULT_BAUD_RATE;
	/* datasheet said it is not supported */
	if (baudrate > 6000000)
		baudrate = 6000000;

	format = termios->c_cflag & CSTOPB ? 2 : 1;

	buffer->paritytype = 0;
	if (termios->c_cflag & PARENB) {
		if (termios->c_cflag & PARODD)
			buffer->paritytype += 1;
		else
			buffer->paritytype += 2;
		if  (termios->c_cflag & CMSPAR)
			buffer->paritytype += 2;
	}

	switch (C_CSIZE(tty)) {
	case CS5:
		buffer->databits = 5;
		break;
	case CS6:
		buffer->databits = 6;
		break;
	case CS7:
		buffer->databits = 7;
		break;
	case CS8:
	default:
		buffer->databits = 8;
		break;
	}
	buffer->cmd = CMD_WB_E | (portnum & 0x0F);
	buffer->reg = R_INIT;
	buffer->port = portnum;
	buffer->baudrate = cpu_to_be32(baudrate);

	if (format == 2)
		buffer->format = 0x02;
	else if (format == 1)
		buffer->format = 0x00;

	buffer->rate = max_t(speed_t, 5, DIV_ROUND_CLOSEST(10000 * 15, baudrate));

	ret = usb_bulk_msg(ch348->udev, ch348->cmd_ep, buffer,
			   sizeof(*buffer), &sent, CH348_CMD_TIMEOUT);
	if (ret < 0) {
		dev_err(&ch348->udev->dev, "Failed to change line settings: err=%d\n",
			ret);
		goto out;
	}

	ret = ch348_do_magic(ch348, portnum, CMD_W_R, R_C1, 0x0F);
	if (ret < 0)
		goto out;

	if (C_CRTSCTS(tty))
		ret = ch348_set_uartmode(ch348, portnum, portnum, M_HF);
	else
		ret = ch348_set_uartmode(ch348, portnum, portnum, M_NOR);

out:
	kfree(buffer);
}

static int ch348_open(struct tty_struct *tty, struct usb_serial_port *port)
{
	struct ch348 *ch348 = usb_get_serial_data(port->serial);
	int ret;

	if (tty)
		ch348_set_termios(tty, port, NULL);

	ret = ch348_configure(ch348, port->port_number);
	if (ret) {
		dev_err(&ch348->udev->dev, "Fail to configure err=%d\n", ret);
		return ret;
	}

	return usb_serial_generic_open(tty, port);
}

static void ch348_disconnect(struct usb_serial *serial)
{
	struct ch348 *ch348 = usb_get_serial_data(serial);

	usb_kill_urb(ch348->status_urb);
}

static int ch348_attach(struct usb_serial *serial)
{
	struct usb_serial_port *port0 = serial->port[1];
	struct usb_device *usb_dev = serial->dev;
	struct usb_endpoint_descriptor *epcmd;
	struct usb_interface *intf;
	struct ch348 *ch348;
	int ret;

	ch348 = kzalloc(sizeof(*ch348), GFP_KERNEL);
	if (!ch348)
		return -ENOMEM;

	usb_set_serial_data(serial, ch348);

	ch348->udev = serial->dev;
	ch348->serial = serial;

	intf = usb_ifnum_to_if(usb_dev, 0);
	epcmd = &intf->cur_altsetting->endpoint[3].desc;
	ret = usb_serial_generic_submit_read_urbs(port0, GFP_KERNEL);
	if (ret)
		return ret;

	ch348->cmd_ep = usb_sndbulkpipe(usb_dev, epcmd->bEndpointAddress);

	return 0;
}

static void ch348_release(struct usb_serial *serial)
{
	struct ch348 *ch348 = usb_get_serial_data(serial);
	struct usb_serial_port *port0 = serial->port[1];

	usb_serial_generic_close(port0);

	kfree(ch348);
}

static int ch348_probe(struct usb_serial *serial, const struct usb_device_id *id)
{
	struct usb_device *usb_dev = serial->dev;
	struct usb_endpoint_descriptor *epcmd;
	struct usb_endpoint_descriptor *epread;
	struct usb_endpoint_descriptor *epwrite;
	struct usb_interface *intf;
	int ret;

	intf = usb_ifnum_to_if(usb_dev, 0);

	ret = usb_find_common_endpoints(intf->cur_altsetting, &epread, &epwrite,
					NULL, NULL);
	if (ret) {
		dev_err(&serial->dev->dev, "Failed to find basic endpoints ret=%d\n", ret);
		return ret;
	}
	epcmd = &intf->cur_altsetting->endpoint[3].desc;

	if (!usb_endpoint_is_bulk_out(epcmd)) {
		dev_err(&serial->dev->dev, "Missing second bulk out\n");
		return -ENODEV;
	}

	return 0;
}

static int ch348_calc_num_ports(struct usb_serial *serial,
				struct usb_serial_endpoints *epds)
{
	int i;

	for (i = 1; i < CH348_MAXPORT; ++i) {
		epds->bulk_out[i] = epds->bulk_out[0];
		epds->bulk_in[i] = epds->bulk_in[0];
	}
	epds->num_bulk_out = CH348_MAXPORT;
	epds->num_bulk_in = CH348_MAXPORT;

	return CH348_MAXPORT;
}

static const struct usb_device_id ch348_ids[] = {
	{ USB_DEVICE(0x1a86, 0x55d9), },
	{ }
};

MODULE_DEVICE_TABLE(usb, ch348_ids);

static struct usb_serial_driver ch348_device = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "ch348",
	},
	.id_table =		ch348_ids,
	.num_ports =		CH348_MAXPORT,
	.num_bulk_in =		1,
	.num_bulk_out =		2,
	.open =			ch348_open,
	.set_termios =		ch348_set_termios,
	.process_read_urb =	ch348_process_read_urb,
	.prepare_write_buffer =	ch348_prepare_write_buffer,
	.probe =		ch348_probe,
	.calc_num_ports =	ch348_calc_num_ports,
	.attach =		ch348_attach,
	.release =		ch348_release,
	.disconnect=		ch348_disconnect,
};

static struct usb_serial_driver * const serial_drivers[] = {
	&ch348_device, NULL
};

module_usb_serial_driver(serial_drivers, ch348_ids);

MODULE_AUTHOR("Corentin Labbe <clabbe@baylibre.com>");
MODULE_DESCRIPTION("USB CH348 Octo port serial converter driver");
MODULE_LICENSE("GPL");
