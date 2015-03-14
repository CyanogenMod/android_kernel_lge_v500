/*
 * u_atcmd.c - AT Command Handler
 *
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <mach/usb_bridge.h>
#include <mach/usb_gadget_xport.h>
#include <linux/string.h>

#define ENABLE_DUMMY_HSIC_TTY

#ifdef ENABLE_DUMMY_HSIC_TTY
static int hsic_setup(struct usb_gadget *g);
void hsic_cleanup(void);
#endif

#define AT_PREFIX "ttyGS"

#define ATD_HANDLE_SETCOM_ATCMD
#define ATD_HANDLE_TSENS_ATCMD
#define ATD_HANDLE_BATT_THEM_ATCMD
#define ATD_HANDLE_MDMLOG

#ifdef CONFIG_MACH_APQ8064_ALTEV
#define ATCMD_BUF_SIZE 20
unsigned char atcmd_buf[ATCMD_BUF_SIZE] = {0,};
int atcmd_enter = 0;
#endif

static const char *at_table[] = {
    /*
     * NOTICE: don't use "%QEM","+CKPD"
     */

    "+MTC","+CCLK","+CSDF","+CSO","+CSS","+CTSA","+VZWAPNE","%ACCEL","%ACS","%ALC","%AVR","%BATL",
    "%BATT","%BCPE","%BCPL","%BOOF","%BTAD","%BTSEARCH","%BTTM","%CALCK","%CALDT","%CAM","%CAMPREQ",
    "%CHARGE","%CHCOMP","%CHIPSET","%CODECRC","%COMPASS","%DBCHK","%DETACH","%DEVICETEST","%DIAGNV",
    "%DRMCERT","%DRMERASE","%DRMINDEX","%DRMTYPE","%ECALL","%EMMREJECT","%EMT","%FBOOT","%FILECRC",
    "%FKPD","%FLIGHT","%FOTAID","%FPRICRC","%FRST","%FRSTSTATUS","%FUELRST","%FUELVAL","%FUSG","%GKPD",
    "%GNSS","%GNSS1","%GYRO","%SURV","%VZWHM","%VZWIOTHM","%HWVER","%IDDE","%IMEI","%3WAYSYNC","%IMPL",
    "%IMSTESTMODE","%INFO","%INISIM","%ISSIM","%KEYLOCK","%LANG","%LCATT","%LCD","%BOFF","%LCDCAPTURE",
    "%LCDINFO","%LCIMSSETCFG","%LGANDROID","%LGATSERVICE","%LGPWD","%LOGSERVICE","%LTEAUTHFAIL",
    "%LTECALL","%LTERFMODE","%MAC","%MACCK","%MAXCT","%MIMOANTCHECK","%MLT","%MMCFACTORYFORMAT",
    "%MMCFORMAT","%MMCISFORMATTED","%MMCTOTALSIZE","%MMCUSEDSIZE","%DATST","%MOT","%MPT","%NCM","%NFC",
    "%NTCODE","%NTSWAP","%OSPPWDINIT","%OSVER","%PMRST","%PNUM","%PNUMBER","%PNUMBER","%PNUM","%PROXIMITY",
    "%PTNCLR","%REATTACH","%RESTART","%SATPC","%LOGSAVE","%SCHK","%WALLPAPER","%SERIALNO","%SIMID","%SIMOFF",
    "%SIMPWDINIT","%SLEN","%SLTYPE","%SPM","%SUFFIX","%SULC","%SWOV","%SWV","%TETHER","%TOTALCRC",
    "%TOUCHFWVER","%ULCV","%ULCW","%USB","%VCOIN","%VLC","%VLST","%VSLT","%WLAN","%WLANR","%WLANT",
#ifdef ATD_HANDLE_SETCOM_ATCMD
    "+CATLIST","+CTACT","+CCLGS","+CDUR","+CDVOL","+CEMAIL","+CWAP","+CDCONT","+CSYNC","+CBLTH","+CALRM",
    "+CTMRV","+CSMCT","+CWLNT","+CKSUM","+CNPAD","+CTASK","+CMSG","+CTBCPS","+CRST","+CRMIND",
#endif
#ifdef ATD_HANDLE_TSENS_ATCMD
    "%TSENS",
#endif
#ifdef ATD_HANDLE_BATT_THEM_ATCMD
    "%BATMP",
#endif
#ifdef ATD_HANDLE_MDMLOG
    "%MDMLOG",
#endif
    NULL
};

#define AT_RESPONSE_OK "AT\r\nOK\r\n"

static int at_open(struct tty_struct *tty, struct file *file)
{
    int port_num = tty->index;
    struct gdata_port *port = gdata_ports[port_num].port;
    int status;

    pr_debug("at_open: ttyGS%d (%p,%p) ...\n", port->port_num, tty, file);

    do {
        mutex_lock(&port->tty_lock);
        if (!port)
            status = -ENODEV;
        else {
            /* already open?  Great. */
            if (port->open_count) {
                status = 0;
                port->open_count++;

                /* currently opening/closing? wait ... */
            } else if (port->openclose) {
                status = -EBUSY;

                /* ... else we do the work */
            } else {
                status = -EAGAIN;
                port->openclose = true;
            }
        }
        mutex_unlock(&port->tty_lock);

        switch (status) {
            default:
                /* fully handled */
                return status;
            case -EAGAIN:
                /* must do the work */
                break;
            case -EBUSY:
                /* wait for EAGAIN task to finish */
                msleep(1);
                /* REVISIT could have a waitchannel here, if
                 * concurrent open performance is important
                 */
                break;
        }
    } while (status != -EAGAIN);

    tty->driver_data = port;
    port->tty = tty;

    port->open_count = 1;
    port->openclose = false;

    pr_debug("at_open: ttyGS%d (%p,%p) done!\n", port->port_num, tty, file);

    return 0;
}

static void at_close(struct tty_struct *tty, struct file *file)
{
    struct gdata_port *port = tty->driver_data;

    pr_debug("at_close: ttyGS%d (%p,%p) ...\n", port->port_num, tty, file);

    if (port->open_count != 1) {
        if (port->open_count == 0)
            WARN_ON(1);
        else
            --port->open_count;

        return;
    }

    port->openclose = false;
    port->open_count = 0;

    tty->driver_data = NULL;
    port->tty = NULL;

    port->openclose = false;

    pr_debug("at_close: ttyGS%d (%p,%p) done!\n",
            port->port_num, tty, file);

}

static int at_write(struct tty_struct *tty, const unsigned char *buf, int count)
{
    struct gdata_port *port = tty->driver_data;
    int avail;
    struct sk_buff *skb;
    int status;

    pr_vdebug("at_write: ttyGS%d (%p) writing %d bytes\n",
            port->port_num, tty, count);

    avail = test_bit(CH_OPENED, &port->bridge_sts);
    /* if no space, we'll have to setup a notification later to wake up the
     * tty framework when space becomes avaliable
     */
    if (!avail) {
        pr_debug("at_write: avail(0)\n");
        return 0;
    }

#ifdef VERBOSE_DEBUG
    print_hex_dump(KERN_DEBUG, "fromatd:", DUMP_PREFIX_OFFSET, 16, 1, buf, count, 1);
#endif

    skb = alloc_skb(count, GFP_ATOMIC);
    if (!skb) {
        pr_debug("at_write: ENOMEM\n");
        return -ENOMEM;
    }
    memcpy(skb->data, buf, count);
    skb->len = count;
    status = ghsic_data_receive(port, skb, count);
    if (status) {
        pr_debug("at_write: status(%d)\n", status);
        return status;
    }

    pr_debug("at_write: return(%d)\n", count);
    return count;
}

static int at_write_room(struct tty_struct *tty)
{
    struct gdata_port *port = tty->driver_data;

    pr_debug("%s\n", __func__);

    return test_bit(CH_OPENED, &port->bridge_sts) ? PAGE_SIZE : 0;
}

#if 0
static int at_chars_in_buffer(struct tty_struct *tty)
{
    struct gdata_port *port = tty->driver_data;

    pr_debug("%s\n", __func__);

    return test_bit(CH_OPENED, &port->bridge_sts);
}
#endif

static void at_unthrottle(struct tty_struct *tty)
{
    struct gdata_port *port = tty->driver_data;

    pr_debug("%s\n", __func__);

    if (!port)
        return;
    pr_vdebug("at_flush_chars: (%d,%p)\n", port->port_num, tty);

    ghsic_data_start_rx(port);
}

static int at_tiocmget(struct tty_struct *tty)
{
    int port_num = tty->index;
    struct gctrl_port *port = gctrl_ports[port_num].port;
    struct gserial *gser;
    unsigned int result = 0;

    spin_lock_irq(&port->port_lock);
    gser = port->port_usb;
    if (!gser) {
        result = -ENODEV;
        goto fail;
    }

    if (gser->get_dtr)
        result |= (gser->get_dtr(gser) ? TIOCM_DTR : 0);

    if (gser->get_rts)
        result |= (gser->get_rts(gser) ? TIOCM_RTS : 0);

    if (gser->serial_state & TIOCM_CD)
        result |= TIOCM_CD;

    if (gser->serial_state & TIOCM_RI)
        result |= TIOCM_RI;
fail:
    spin_unlock_irq(&port->port_lock);
    return result;
}

static int at_tiocmset(struct tty_struct *tty, unsigned int set, unsigned int clear)
{
    return 0;
}

int atcmd_write_toatd(struct gdata_port *port, struct sk_buff *skb)
{
    struct tty_struct *tty;
    unsigned char *ptr;
    int avail;
    char *cmd;
    int i;

    pr_debug("%s\n", __func__);

    tty = port->tty;
    if (!tty)
        return -ENODEV;

    avail = skb->len;
    if (avail == 0)
        return -EINVAL;

    ptr = skb->data + avail - 1;

#ifdef CONFIG_MACH_APQ8064_ALTEV
	if (avail == 1) {
		if (strlen(atcmd_buf) == ATCMD_BUF_SIZE) {
			printk("ATCMD BUF overflow\n");
		    memset(&atcmd_buf, 0, sizeof(char)*ATCMD_BUF_SIZE);
			goto out;
		}

		if (*(skb->data) == 127) {
//			printk("receive del\n");
			if(strlen(atcmd_buf) == 0) {
				goto out;
			} else {
				atcmd_buf[strlen(atcmd_buf) - 1] = 0;
				goto out;
			}
		} else if (*(skb->data) == 13) {
//			printk("receive enter\n");

			atcmd_enter = 1;

			skb->len = strlen(atcmd_buf)+1;
			avail = skb->len;
			memcpy(skb->data,atcmd_buf,strlen(atcmd_buf));
			*(skb->data + avail - 1) = '\r';
			*(skb->data + avail) = '\n';
			ptr = skb->data + avail - 1;
			memset(&atcmd_buf, 0, ATCMD_BUF_SIZE);
		} else {
//			printk("receive else\n");
			*(skb->data + 1) = 0;
			strcat(atcmd_buf,skb->data);
		}
	}
out :
#endif

    if (strncasecmp(skb->data, "AT", 2) ||
        !(*ptr == '\r' || *ptr == '\n' || *ptr == '\0')) {
        return -EINVAL;
    }

    cmd = kstrdup(skb->data + 2, GFP_ATOMIC);
    if (!cmd) {
        pr_debug("%s: ENOMEM\n", __func__);
        return -ENOMEM;
    }
    if ((ptr = strchr(cmd, '=')) ||
        (ptr = strchr(cmd, '?')) ||
        (ptr = strchr(cmd, '\r')) ) {
        *ptr = '\0';
    }

    if (*cmd != '\0') {
        for (i = 0; at_table[i] != NULL; i++) {
            if (!strcasecmp(cmd, at_table[i])) {
                kfree(cmd);

                if (!test_bit(CH_OPENED, &port->bridge_sts)) {
                    /* signal TTY clients using TTY_BREAK */
                    tty_insert_flip_char(tty, 0x00, TTY_BREAK);
                    tty_flip_buffer_push(tty);
                    break;
                } else {
                    avail = tty_prepare_flip_string(tty, &ptr, avail);
                    if (avail <= 0) {
                        return -EBUSY;
                    }

#ifdef VERBOSE_DEBUG
                    print_hex_dump(KERN_DEBUG, "toatd:", DUMP_PREFIX_OFFSET, 16, 1, skb->data, skb->len, 1);
#endif

                    memcpy(ptr, skb->data, avail);
                    dev_kfree_skb_any(skb);

                    tty_flip_buffer_push(tty);
                }

                /* XXX only when writable and necessary */
                tty_wakeup(tty);
#ifdef CONFIG_MACH_APQ8064_ALTEV
                atcmd_enter = 0;
#endif
                return 0;
            }
        }
    }

#ifdef CONFIG_MACH_APQ8064_ALTEV
	if (atcmd_enter == 1) {
		*(skb->data) = 13;
		*(skb->data+1) = '\r';
		skb->len = 1;
		atcmd_enter = 0;
    }
#endif

    kfree(cmd);
    return -ENOENT;
}

static struct tty_operations at_tty_ops = {
    .open = at_open,
    .close = at_close,
    .write = at_write,
    .write_room = at_write_room,
#if 0
    .chars_in_buffer = at_chars_in_buffer,
#endif
    .unthrottle = at_unthrottle,
    .tiocmget = at_tiocmget,
    .tiocmset = at_tiocmset,
};

static struct tty_driver *at_tty_driver;

int atcmd_setup(struct usb_gadget *g)
{
    struct device *tty_dev;
    int status;

    at_tty_driver = alloc_tty_driver(1);
    if (!at_tty_driver)
        return -ENOMEM;

    at_tty_driver->driver_name = "at_serial";
    at_tty_driver->name = AT_PREFIX;
    /* uses dynamically assigned dev_t values */

    at_tty_driver->type = TTY_DRIVER_TYPE_SERIAL;
    at_tty_driver->subtype = SERIAL_TYPE_NORMAL;
    at_tty_driver->flags = TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV
        | TTY_DRIVER_RESET_TERMIOS;
    at_tty_driver->init_termios = tty_std_termios;

    /* 9600-8-N-1 ... matches defaults expected by "usbser.sys" on
     * MS-Windows.  Otherwise, most of these flags shouldn't affect
     * anything unless we were to actually hook up to a serial line.
     */
    at_tty_driver->init_termios.c_cflag =
        B9600 | CS8 | CREAD | HUPCL | CLOCAL;
    at_tty_driver->init_termios.c_ispeed = 9600;
    at_tty_driver->init_termios.c_ospeed = 9600;

    tty_set_operations(at_tty_driver, &at_tty_ops);

    /* make devices be openable */
    mutex_init(&gdata_ports[0].port->tty_lock);

    /* export the driver ... */
    status = tty_register_driver(at_tty_driver);
    if (status) {
        put_tty_driver(at_tty_driver);
        pr_err("%s: cannot register, err %d\n",
                __func__, status);
        goto fail;
    }

    /* ... and sysfs class devices, so mdev/udev make /dev/ttyGS* */
    tty_dev = tty_register_device(at_tty_driver, 0, &g->dev);
    if (IS_ERR(tty_dev))
        pr_warning("%s: no classdev, err %ld\n", __func__, PTR_ERR(tty_dev));

#ifdef ENABLE_DUMMY_HSIC_TTY
    status = hsic_setup(g);
    if (status)
        goto fail;
#endif

    pr_debug("%s: registered ttyGS0 device\n", __func__);

    return status;
fail:
    put_tty_driver(at_tty_driver);
    at_tty_driver = NULL;
    return status;
}

void atcmd_cleanup(void)
{
    if (!at_tty_driver)
        return;

    tty_unregister_device(at_tty_driver, 0);
    tty_unregister_driver(at_tty_driver);
    put_tty_driver(at_tty_driver);
    at_tty_driver = NULL;

#ifdef ENABLE_DUMMY_HSIC_TTY
    hsic_cleanup();
#endif

    pr_debug("%s: cleaned up ttyGS0 support\n", __func__);
}

#ifdef ENABLE_DUMMY_HSIC_TTY
/*
 * HSIC dummy driver
 */
#define HSIC_PREFIX "hsic"

static int hsic_open(struct tty_struct *tty, struct file *file)
{
    return 0;
}

static void hsic_close(struct tty_struct *tty, struct file *ffile)
{
}

static int hsic_write(struct tty_struct *tty, const unsigned char *buf, int len)
{
    return len;
}

static int hsic_write_room(struct tty_struct *tty)
{
    return 1;
}

static int hsic_chars_in_buffer(struct tty_struct *tty)
{
    return 1;
}

static void hsic_unthrottle(struct tty_struct *tty)
{
}

static int hsic_tiocmget(struct tty_struct *tty)
{
    int port_num = tty->index;
    struct gctrl_port *port = gctrl_ports[port_num].port;
    unsigned int ctrl_bits;
    unsigned int result = 0;

    spin_lock_irq(&port->port_lock);

    ctrl_bits = port->cbits_tohost;

    if (ctrl_bits | ACM_CTRL_DSR)
        result |= TIOCM_DSR;

    if (ctrl_bits | ACM_CTRL_DCD)
        result |= TIOCM_CD;

    if (ctrl_bits | ACM_CTRL_RI)
        result |= TIOCM_RI;

    spin_unlock_irq(&port->port_lock);
    return result;
}

static int hsic_tiocmset(struct tty_struct *tty, unsigned int set, unsigned int clear)
{
    return 0;
}


static struct tty_operations hsic_tty_ops = {
    .open = hsic_open,
    .close = hsic_close,
    .write = hsic_write,
    .write_room = hsic_write_room,
    .chars_in_buffer = hsic_chars_in_buffer,
    .unthrottle = hsic_unthrottle,
    .tiocmget = hsic_tiocmget,
    .tiocmset = hsic_tiocmset,
};

static struct tty_driver *hsic_tty_driver;

static int hsic_setup(struct usb_gadget *g)
{
    struct device *tty_dev;
    int status;

    hsic_tty_driver = alloc_tty_driver(1);
    if (!hsic_tty_driver)
        return -ENOMEM;

    hsic_tty_driver->driver_name = "hsic_serial";
    hsic_tty_driver->name = HSIC_PREFIX;
    /* uses dynamically assigned dev_t values */

    hsic_tty_driver->type = TTY_DRIVER_TYPE_SERIAL;
    hsic_tty_driver->subtype = SERIAL_TYPE_NORMAL;
    hsic_tty_driver->flags = TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV
        | TTY_DRIVER_RESET_TERMIOS;
    hsic_tty_driver->init_termios = tty_std_termios;

    /* 9600-8-N-1 ... matches defaults expected by "usbser.sys" on
     * MS-Windows.  Otherwise, most of these flags shouldn't affect
     * anything unless we were to actually hook up to a serial line.
     */
    hsic_tty_driver->init_termios.c_cflag =
        B9600 | CS8 | CREAD | HUPCL | CLOCAL;
    hsic_tty_driver->init_termios.c_ispeed = 9600;
    hsic_tty_driver->init_termios.c_ospeed = 9600;

    tty_set_operations(hsic_tty_driver, &hsic_tty_ops);

    status = tty_register_driver(hsic_tty_driver);
    if (status) {
        put_tty_driver(hsic_tty_driver);
        pr_err("%s: cannot register, err %d\n",
                __func__, status);
        goto fail;
    }

    tty_dev = tty_register_device(hsic_tty_driver, 0, &g->dev);
    if (IS_ERR(tty_dev))
        pr_warning("%s: no classdev, err %ld\n", __func__, PTR_ERR(tty_dev));

    pr_debug("%s: registered hsic0 device\n", __func__);

    return status;
fail:
    put_tty_driver(hsic_tty_driver);
    hsic_tty_driver = NULL;
    return status;
}

void hsic_cleanup(void)
{
    if (!hsic_tty_driver)
        return;

    tty_unregister_device(hsic_tty_driver, 0);
    tty_unregister_driver(hsic_tty_driver);
    put_tty_driver(hsic_tty_driver);
    hsic_tty_driver = NULL;

    pr_debug("%s: cleaned up hsic0 support\n", __func__);
}
#endif
