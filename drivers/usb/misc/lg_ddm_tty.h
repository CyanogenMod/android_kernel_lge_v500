#ifndef TTY_LGE_DDM_H_
#define TTY_LGE_DDM_H_

#ifdef CONFIG_USB_LGE_DDM_TTY

/* TTY driver status */
enum {
	DDM_TTY_INITIAL 	= 0,
	DDM_TTY_REGISTERED 	= 1,
	DDM_TTY_OPEN 		= 2,
	DDM_TTY_CLOSED 		= 3,
};

struct ddm_tty {
	wait_queue_head_t   waitq;
	struct task_struct *tty_ts;
	struct tty_driver *tty_drv;
	struct tty_struct *tty_str;
	int tty_state;
	int set_logging;
	char *read_buf;
	int read_length;
};

extern struct ddm_tty *lge_ddm_tty;

#endif
#endif /*               */
