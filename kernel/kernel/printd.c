/*
 * printd
 */

#include <linux/kernel.h>
#include <linux/workqueue.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/pagemap.h>
#include <linux/string.h>

#define PRINTD_WQ "printd_wq"
// #define PRINTD_PART "/dev/pro_info"
#define PRINTD_PART "/dev/block/mmcblk0"
#define PRINTD_PART_POS 0x100000
#define PRINTD_PART_LEN 0x300000
#define PRINTD_PART_OFFSET 1024
#define PRINTD_PL_SIZE (1024 * 8)
#define PRINTD_LK_SIZE (1024 * 32)
#define PRINTD_KN_OFFSET (PRINTD_PART_OFFSET + PRINTD_PL_SIZE + PRINTD_LK_SIZE)

#if 0
#define _printk(fmt, args...) printk(fmt, ##args)
#else
#define _printk(fmt, args...)
#endif

#define L_SHIFT 2
#define __LOG_BUF_LEN	(1 << (CONFIG_LOG_BUF_SHIFT + L_SHIFT))

static char __log_buf[__LOG_BUF_LEN + 4];
static char *log_buf = __log_buf;
static int log_buf_len = __LOG_BUF_LEN;
static int log_buf_pos = 0;
static int log_buf_out = 0;
static int part_pos = PRINTD_KN_OFFSET;

static struct workqueue_struct *print_wq;
static struct work_struct print_work;
static int count = 0;
static int game_status = 0;

void printd_dump (char *buf, int len)
{
	if (game_status == -1)
		return;

	if (log_buf_pos + len <= log_buf_len) {
		memcpy(log_buf + log_buf_pos, buf, len);
		log_buf_pos += len;
	} else if (log_buf_len - log_buf_pos != 0) {
		memcpy(log_buf + log_buf_pos, buf, log_buf_len - log_buf_pos);
		log_buf_pos = __LOG_BUF_LEN;
	}

	if ((game_status == 1) && (log_buf_pos == __LOG_BUF_LEN)) {
		queue_work(print_wq, &print_work);
	}
}
EXPORT_SYMBOL(printd_dump);

#define BLOCK_SIZE 512

static int output(int stage)
{
	struct file *filp;
	int open_mode;
	int mode = 0;
	ssize_t res;
	size_t len;
	loff_t pos;
	const char *buf;
	mm_segment_t old_fs;

	_printk("this is: %s\n", __func__);

	buf = log_buf + log_buf_out;
	pos = PRINTD_PART_POS + part_pos;
	len = (log_buf_pos - log_buf_out);

	if ((len == 0) || (len == len % BLOCK_SIZE))
		return 0;
	else
		len -= len % BLOCK_SIZE;

	if (part_pos + len > PRINTD_PART_LEN) {
		len -= PRINTD_PART_LEN - part_pos;
		game_status = -1;
	}

	filp = filp_open(PRINTD_PART, O_RDWR, 0777);
	if (IS_ERR(filp)) {
		_printk("%s filp_open err: %d\n", PRINTD_PART, PTR_ERR(filp));
		return 0;
	}

	old_fs = get_fs();
	set_fs(get_ds());
	// set_fs(KERNEL_DS);

	res = vfs_write(filp, buf, len, &pos);
	set_fs(old_fs);

	if (res == len) {
		_printk("write suc: %s\n", __func__);
		log_buf_out += len;
		part_pos += len;
	} else
		_printk("write err: %s\n", __func__);

	filp_close(filp, NULL);

	return len;
}

#define PRINTD_COUNT 5000
static void printd_func(struct work_struct *work)
{
	if (game_status == -1)
		return;

	count ++;
	if ((count == PRINTD_COUNT) || (game_status)) {
		count = 0;

		if (system_state == SYSTEM_BOOTING) {
		}

		if (system_state == SYSTEM_RUNNING) {
			if (output(1)) {
				if (game_status != -1)
					game_status += 1;
				// game_status = -1;
			}

			if (log_buf_pos == __LOG_BUF_LEN) {
				game_status = -1;
			}
		}
	}

	if (game_status == 0)
		queue_work(print_wq, &print_work);
}

static int __init printd_init(void)
{
	int ret = 0;

	_printk("start: %s\n", __func__);

	print_wq = create_singlethread_workqueue(PRINTD_WQ);
	if (!print_wq) {
		return -ENOMEM;
	}

	INIT_WORK(&print_work, printd_func);
	queue_work(print_wq, &print_work);

	return ret;
}

static void __exit printd_exit(void)
{
	_printk("stop: %s\n", __func__);
	if (print_wq) {
		destroy_workqueue(print_wq);
	}
}

module_init(printd_init);
module_exit(printd_exit);
MODULE_LICENSE("GPL");
