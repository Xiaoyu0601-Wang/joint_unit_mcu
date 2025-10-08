/*
 * icm42688.c
 *
 * Minimal, well-documented skeleton driver for the ICM-42688 IMU for NuttX.
 *
 * This file provides:
 *  - register definitions and useful constants
 *  - SPI/I2C register read/write helpers (adaptable to your board)
 *  - a probe / init sequence that configures accel + gyro
 *  - a simple sampling thread that polls the sensor and caches samples
 *  - a character-device interface (/dev/icm426880) with read() returning
 *      the latest accel+gyro sample as a packed struct
 *
 * This driver intentionally contains a few clearly-marked TODOs where
 * NuttX-specific integration choices must be adapted (SPI calls, work queues,
 * build system integration). Use this as a starting point and follow the
 * instructions below to wire it into your nuttx tree.
 *
 * Copyright (C) 2025  Your Name
 * License: BSD-3-Clause (suggested)
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <pthread.h>
#include <semaphore.h>

/* TODO: include correct NuttX SPI header for your version. Many NuttX
 * variants provide <nuttx/spi/spi.h> and helpers like SPI_LOCK/ SPI_SELECT,
 * spi_send/spi_exchange. Adjust the helpers below to match your NuttX API.
 */
#include <nuttx/spi/spi.h>

/* Driver configuration */
#define ICM42688_DEVICE_PATH "/dev/icm42688"
#define ICM42688_POLL_INTERVAL_US 10000 /* 100 Hz */

/* ----- ICM-42688 register map (subset) ----- */
#define ICM_REG_WHO_AM_I 0x0F
#define ICM_REG_BANK_SEL 0x7F
#define ICM_REG_RESET 0x01
#define ICM_REG_PWR_MGMT0 0x4E
#define ICM_REG_INT_STATUS 0x2D
#define ICM_REG_ACCEL_DATA_X1 0x1F /* accel/gyro data start (bank 0 default mapping may differ) */
#define ICM_REG_FIFO_CONFIG 0x16
#define ICM_REG_GYRO_CONFIG0 0x4F
#define ICM_REG_ACCEL_CONFIG0 0x50
#define ICM_REG_TEMP_DATA1 0x21
#define ICM_REG_ACCEL_DATA 0x1F

/* WHO_AM_I expected value for ICM-42688 (commonly 0x47 on some silicon - check your datasheet) */
#define ICM_WHO_AM_I_ID 0x47

/* SPI R/W masks */
#define ICM_SPI_READ_FLAG 0x80
#define ICM_SPI_WRITE_FLAG 0x00

/* Conversion factors (typical; consult datasheet and your selected FS/ODR) */
#define GRAVITY_MSS 9.80665f

/* Data layout we will return to users via read()
 * Packed to avoid ABI padding surprises.
 */
struct icm42688_sample_s
{
    int64_t timestamp_us; /* monotonic timestamp when sample was taken */
    int16_t accel_x;      /* raw counts */
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x; /* raw counts */
    int16_t gyro_y;
    int16_t gyro_z;
};

/* Device private structure */
struct icm42688_dev_s
{
    struct spi_dev_s *spi; /* SPI handle (from board-specific SPI bus init) */
    pthread_t thread;      /* sampling thread */
    sem_t lock;            /* protects sample */
    volatile bool stop;    /* request thread stop */
    struct icm42688_sample_s sample;
    int fd_refcount; /* open file count */
};

/* Single global instance for simplicity. For multi-device support, convert to
 * a linked list and device numbering.
 */
static struct icm42688_dev_s g_icm;

/* Utility: time in microseconds */
static int64_t get_time_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000LL + ts.tv_nsec / 1000LL;
}

/* ----- Low-level register R/W helpers (SPI) ----- */

/* NOTE: adapt these helpers to match your NuttX SPI API. The example below
 * uses a common pattern: select device, perform a transfer (write cmd + read),
 * deselect. If your NuttX version has spi_send(), spi_exchange(), or
 * interface-specific helpers, replace accordingly.
 */

static int icm_spi_lock(struct icm42688_dev_s *dev)
{
    /* TODO: use SPI_LOCK() if available in your Nuttx build. */
    return 0;
}

static int icm_spi_unlock(struct icm42688_dev_s *dev)
{
    /* TODO: use SPI_LOCK() if available in your Nuttx build. */
    return 0;
}

static int icm_spi_select(struct icm42688_dev_s *dev, bool select)
{
    /* TODO: call SPI_SELECT(dev->spi, devid, select) or equivalent.
     * For many boards, the device id is 0.
     */
    if (dev == NULL || dev->spi == NULL)
        return -ENODEV;

    /* Example placeholder: replace with real call */
    (void)dev;
    (void)select;
    return 0;
}

/* spi_transfer: write tx buffer and read rx buffer of given length.
 * NOTE: Replace internals with spi_exchange() or spi_send/spi_receive
 * depending on your NuttX version.
 */
static int icm_spi_transfer(struct icm42688_dev_s *dev,
                            const uint8_t *tx, uint8_t *rx, size_t len)
{
    /* TODO: implement using your board's SPI API. This skeleton simply
     * demonstrates expected behaviour and MUST be adapted.
     */
    if (!dev || !dev->spi)
        return -ENODEV;

    /* Example pseudo-code:
     * SPI_LOCK(dev->spi, true);
     * SPI_SELECT(dev->spi, 0, true);
     * spi_exchange(dev->spi, tx, rx, len);
     * SPI_SELECT(dev->spi, 0, false);
     * SPI_LOCK(dev->spi, false);
     */

    (void)tx;
    (void)rx;
    (void)len;
    return -ENOSYS; /* Not implemented; replace with functional code */
}

static int icm_reg_read(struct icm42688_dev_s *dev, uint8_t reg, uint8_t *val)
{
    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = reg | ICM_SPI_READ_FLAG;
    tx[1] = 0x00;

    int ret = icm_spi_transfer(dev, tx, rx, 2);
    if (ret < 0)
        return ret;

    *val = rx[1];
    return 0;
}

static int icm_reg_write(struct icm42688_dev_s *dev, uint8_t reg, uint8_t val)
{
    uint8_t tx[2];

    tx[0] = reg | ICM_SPI_WRITE_FLAG;
    tx[1] = val;

    int ret = icm_spi_transfer(dev, tx, NULL, 2);
    if (ret < 0)
        return ret;

    return 0;
}

/* Read consecutive registers (burst). reg = first reg; count bytes into buf.
 * Uses the same SPI assumption: send reg|READ then read N bytes.
 */
static int icm_reg_read_multi(struct icm42688_dev_s *dev, uint8_t reg,
                              uint8_t *buf, size_t count)
{
    if (count == 0)
        return -EINVAL;

    /* Allocate tx buffer = 1 + count, rx buffer same size */
    uint8_t tx[count + 1];
    uint8_t rx[count + 1];
    tx[0] = reg | ICM_SPI_READ_FLAG;
    memset(&tx[1], 0, count);

    int ret = icm_spi_transfer(dev, tx, rx, count + 1);
    if (ret < 0)
        return ret;

    memcpy(buf, &rx[1], count);
    return 0;
}

/* ----- Basic device control ----- */

static int icm_reset(struct icm42688_dev_s *dev)
{
    /* Software reset sequence - refer to datasheet for specific bit */
    int ret = icm_reg_write(dev, ICM_REG_RESET, 0x01);
    if (ret < 0)
        return ret;

    /* wait ~50 ms for reset to complete */
    usleep(50000);
    return 0;
}

static int icm_check_whoami(struct icm42688_dev_s *dev)
{
    uint8_t id;
    int ret = icm_reg_read(dev, ICM_REG_WHO_AM_I, &id);
    if (ret < 0)
        return ret;

    if (id != ICM_WHO_AM_I_ID)
    {
        syslog(LOG_ERR, "icm42688: WHO_AM_I mismatch: 0x%02x (expected 0x%02x)\n", id, ICM_WHO_AM_I_ID);
        return -ENODEV;
    }

    return 0;
}

static int icm_configure_default(struct icm42688_dev_s *dev)
{
    int ret;

    /* Put device to standby / reset internal state before configuring */
    /* Configure power management, ranges, ODR etc. The exact register
     * settings depend on desired full-scale ranges and ODR. Below are examples
     * (placeholders). Consult the ICM-42688 datasheet for details.
     */

    /* Example: enable accel & gyro with default ODR, disable I3C */
    ret = icm_reg_write(dev, ICM_REG_PWR_MGMT0, 0x01); /* clear sleep */
    if (ret < 0)
        return ret;

    /* Set gyro full scale (example) */
    ret = icm_reg_write(dev, ICM_REG_GYRO_CONFIG0, 0x00); /* +/-2000dps? (placeholder) */
    if (ret < 0)
        return ret;

    /* Set accel full scale (example) */
    ret = icm_reg_write(dev, ICM_REG_ACCEL_CONFIG0, 0x00); /* +/-16g? (placeholder) */
    if (ret < 0)
        return ret;

    /* Enable data ready interrupt or configure FIFO if desired */
    ret = icm_reg_write(dev, ICM_REG_INT_STATUS, 0x01); /* placeholder */
    if (ret < 0)
        return ret;

    usleep(10000);
    return 0;
}

/* ----- Sampling ----- */

/* Parse a burst read into sample structure. The register layout varies by
 * bank and configuration. Here we assume a contiguous accel + gyro + temp
 * sequence starting at ICM_REG_ACCEL_DATA with each axis in big-endian
 * high/low bytes (common for many InvenSense devices). Adjust according to
 * your device register map and BANK selected.
 */

static int icm_parse_sample(const uint8_t *buf, size_t len, struct icm42688_sample_s *out)
{
    if (len < 12)
        return -EINVAL;

    /* accel X/Y/Z (16-bit signed), then gyro X/Y/Z (16-bit signed) */
    out->accel_x = (int16_t)((buf[0] << 8) | buf[1]);
    out->accel_y = (int16_t)((buf[2] << 8) | buf[3]);
    out->accel_z = (int16_t)((buf[4] << 8) | buf[5]);
    out->gyro_x = (int16_t)((buf[6] << 8) | buf[7]);
    out->gyro_y = (int16_t)((buf[8] << 8) | buf[9]);
    out->gyro_z = (int16_t)((buf[10] << 8) | buf[11]);
    out->timestamp_us = get_time_us();
    return 0;
}

static void *icm_sampling_thread(void *arg)
{
    struct icm42688_dev_s *dev = (struct icm42688_dev_s *)arg;
    uint8_t buf[12];

    while (!dev->stop)
    {
        /* Read 12 bytes starting at ACCEL_DATA register */
        int ret = icm_reg_read_multi(dev, ICM_REG_ACCEL_DATA, buf, sizeof(buf));
        if (ret == 0)
        {
            struct icm42688_sample_s s;
            if (icm_parse_sample(buf, sizeof(buf), &s) == 0)
            {
                sem_wait(&dev->lock);
                dev->sample = s;
                sem_post(&dev->lock);
            }
        }
        else
        {
            /* Log but keep trying */
            syslog(LOG_ERR, "icm42688: sample read error %d\n", ret);
        }

        /* Sleep until next poll. Use usleep to keep things simple here. For a
         * production driver consider using irq-driven samples or NuttX work
         * queue / timers for better timing guarantees.
         */
        usleep(ICM42688_POLL_INTERVAL_US);
    }

    return NULL;
}

/* ----- Character device file ops ----- */

static int icm_open(struct icm42688_dev_s *dev)
{
    dev->fd_refcount++;
    return 0;
}

static int icm_close(struct icm42688_dev_s *dev)
{
    if (dev->fd_refcount > 0)
        dev->fd_refcount--;
    return 0;
}

/* read: copy latest sample to user buffer. If user buffer is smaller than
 * struct icm42688_sample_s, return EINVAL.
 */
static ssize_t icm_read(struct icm42688_dev_s *dev, char *buffer, size_t len)
{
    if (len < sizeof(struct icm42688_sample_s))
        return -EINVAL;

    sem_wait(&dev->lock);
    memcpy(buffer, &dev->sample, sizeof(dev->sample));
    sem_post(&dev->lock);

    return sizeof(dev->sample);
}

/* Minimal ioctl: allow user to request timestamp or raw conversion later */
#define ICM_IOCTL_GET_SAMPLE _IOR('I', 0x01, struct icm42688_sample_s)

static int icm_ioctl(struct icm42688_dev_s *dev, int cmd, unsigned long arg)
{
    if (cmd == ICM_IOCTL_GET_SAMPLE)
    {
        if ((void *)arg == NULL)
            return -EINVAL;
        sem_wait(&dev->lock);
        memcpy((void *)arg, &dev->sample, sizeof(dev->sample));
        sem_post(&dev->lock);
        return 0;
    }
    return -ENOTTY;
}

/* ----- Public init/uninit helpers ----- */

/* User supplies an initialized spi_dev_s * (board-specific) */
int icm42688_register(struct spi_dev_s *spi)
{
    int ret;

    memset(&g_icm, 0, sizeof(g_icm));
    g_icm.spi = spi;
    sem_init(&g_icm.lock, 0, 1);
    g_icm.fd_refcount = 0;

    /* Basic probe */
    ret = icm_reset(&g_icm);
    if (ret < 0)
        return ret;

    ret = icm_check_whoami(&g_icm);
    if (ret < 0)
        return ret;

    ret = icm_configure_default(&g_icm);
    if (ret < 0)
        return ret;

    /* Start sampling thread */
    g_icm.stop = false;
    int err = pthread_create(&g_icm.thread, NULL, icm_sampling_thread, &g_icm);
    if (err != 0)
    {
        syslog(LOG_ERR, "icm42688: failed to create sampling thread: %d\n", err);
        return -err;
    }

    /* Register a character device. In real NuttX driver this would use
     * register_driver() or similar. We'll create a simple /dev node using
     * register_driver (example). Replace with nuttx-specific call if needed.
     */

    extern int register_driver(const char *path, const struct file_operations *fops, mode_t mode, void *priv);

    /* Minimal file operations wrapper that binds to our helpers */
    static const struct file_operations ? fops = {0};
    /* TODO: Provide a proper struct file_operations instance using the
     * correct type for your NuttX version. The symbol layout varies.
     */

    /* Placeholder: the user should replace this with actual register_driver
     * invocation code appropriate for their NuttX version.
     */

    (void)register_driver;
    (void)fops;

    syslog(LOG_INFO, "icm42688: registered and sampling started\n");
    return 0;
}

int icm42688_unregister(void)
{
    g_icm.stop = true;
    pthread_join(g_icm.thread, NULL);
    sem_destroy(&g_icm.lock);

    /* Unregister device node: unregister_driver(DEVICE_PATH) - implement as needed */

    return 0;
}

/* ----- End of driver skeleton ----- */

/*
 * Installation notes (how to use this file):
 * 1. Place this file in your nuttx drivers tree (e.g. drivers/imu/icm42688/).
 * 2. Add a Kconfig option and Makefile entry (Kconfig and Make.defs) so it
 *    builds into the kernel. See existing drivers/imu/* entries for examples.
 * 3. Provide a board-specific initialization routine (board_app_initialize() or
 *    board-specific bringup) that calls icm42688_register(board_spibus_initialize())
 *    with an spi_dev_s * from your board's SPI bus setup code.
 * 4. Adapt the low-level SPI helpers (icm_spi_transfer/icm_spi_select) to the
 *    exact NuttX SPI API available in your version. Many NuttX boards provide
 *    helper functions spi_lock, SPI_SELECT, and spi_exchange.
 * 5. Replace placeholders and TODOs annotated in this file to match your
 *    project's coding standards and the exact ICM-42688 register field values
 *    desired for full-scale ranges and ODR.
 *
 * Example board bringup (pseudo):
 *   struct spi_dev_s *spi = board_spibus_initialize(1);
 *   if (!spi) return -ENODEV;
 *   icm42688_register(spi);
 *
 * Useful references:
 *  - ICM-42688 datasheet and register map
 *  - Existing NuttX IMU drivers for patterns and register_driver usage
 */
