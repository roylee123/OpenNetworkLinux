/************************************************************
 * <bsn.cl fy=2014 v=onl>
 *
 *           Copyright 2014 Big Switch Networks, Inc.
 *           Copyright 2016 Accton Technology Corporation.
 *
 * Licensed under the Eclipse Public License, Version 1.0 (the
 * "License"); you may not use this file except in compliance
 * with the License. You may obtain a copy of the License at
 *
 *        http://www.eclipse.org/legal/epl-v10.html
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific
 * language governing permissions and limitations under the
 * License.
 *
 * </bsn.cl>
 ************************************************************
 *
 *
 *
 ***********************************************************/
#include <time.h>
#include <onlp/platformi/sfpi.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/mman.h>
#include <fcntl.h>
#include "platform_lib.h"
#include "x86_64_accton_minipack_log.h"

/* PIM stands for "Port Interface Module".
 * For minipack, there are hot-pluggable 8 PIMs.
 * Each PIM can have 16*16Q or 4*4DD ports.
 */
#define NUM_OF_PIM              (PLATFOTM_NUM_OF_PIM)
#define TYPES_OF_PIM            (2)
#define NUM_OF_SFP_PORT         (128)
#define SFP_PORT_PER_PIM        (NUM_OF_SFP_PORT/NUM_OF_PIM)
#define PIM_POLL_INTERVAL       (1) /*in seconds*/
#define PORT_POLL_INTERVAL      (1) /*per PIM, in seconds*/

#define PORT_TO_PIM(_port)        (_port / SFP_PORT_PER_PIM)
#define PORT_OF_PIM(_port)        (_port % SFP_PORT_PER_PIM)

typedef struct {
    bool      valid;
    time_t    last_poll;
    uint32_t  present;
} present_status_t;

typedef struct {
    present_status_t pim;
    present_status_t port_at_pim[NUM_OF_SFP_PORT/NUM_OF_PIM];
} sfpi_port_status_t;

int onlp_read_pim_present(uint32_t *bmap);
static int get_ports_presence(uint32_t pimId, uint32_t *pbmp);
static int get_ports_lpmode(uint32_t pimId, uint32_t *pbmp);
static int get_ports_reset(uint32_t pimId, uint32_t *pbmp);
static int set_ports_lpmode(uint32_t pimId, uint32_t value);
static int set_ports_reset(uint32_t pimId, uint32_t value);



static sfpi_port_status_t g_sfpiPortStat = {0};
static int update_ports(int pim, bool valid, uint32_t present) {
    present_status_t *ports;

    ports = &g_sfpiPortStat.port_at_pim[pim];
    ports->valid = valid;
    ports->present = present;
    ports->last_poll = time (NULL);

    return ONLP_STATUS_OK;
}

static uint32_t fbfpgaio_read(uint32_t addr);
int onlp_sfpi_init(void)
{
    int i;
    /* Unleash the Reset pin again.
     * It might be unleashed too early for some types of transcievers.
     */
    for (i = 0; i < NUM_OF_PIM; i++) {
        set_ports_reset(i, 0);
    }
    return ONLP_STATUS_OK;
}

int onlp_sfpi_bitmap_get(onlp_sfp_bitmap_t* bmap)
{
    int p;
    AIM_BITMAP_CLR_ALL(bmap);

    for(p = 0; p < NUM_OF_SFP_PORT; p++) {
        AIM_BITMAP_SET(bmap, p);
    }

    return ONLP_STATUS_OK;
}

/*
0 @bit 14
1 @bit 15
2 @bit 12
3 @bit 13
4 @bit 10
5 @bit 11
6 @bit 8
7 @bit 9
... same order for port 8-15.
*/
static uint32_t
_sfpi_port_present_remap_reg(uint32_t value)
{
    return value;
}

/* "PIM" stands for "Port Interface Module". They are hot-pluggable.
 * A pim can have 16 QSFP ports of 100Gbps, or 4 DD ports of 400 Gbps.
 *
 * Return 1 if present.
 * Return 0 if not present.
 * Return < 0 if error.
 */
static int
onlp_pim_is_present(int pim)
{
    uint32_t present;
    int ret;
    sfpi_port_status_t *ps;

    ps = &g_sfpiPortStat;
    if (!ps->pim.valid ) {
        ret = onlp_read_pim_present(&present);
        if (ret < 0) {
            ps->pim.valid = false;
            ps->pim.present = 0;
            present = 0;
            return ONLP_STATUS_E_INTERNAL;
        } else {
            ps->pim.valid = true;
            ps->pim.present = present;
        }
    } else {
        present = ps->pim.present;
    }
    return !!(present & BIT(pim % NUM_OF_PIM));
}



/*bit_array is the present bitmap of a PIM, not all ports on this machine.*/
static int
get_pim_port_present_bmap(int port, uint32_t *bit_array)
{
    time_t cur, elapse;
    int    ret;
    uint32_t present, pim;
    present_status_t *ports;

    pim = PORT_TO_PIM(port);
    /*If PIM not present, set all 0's to pbmap.*/
    if(onlp_pim_is_present(pim) == 0) {
        present  = 0;
        update_ports(pim, 0, present);
        *bit_array = _sfpi_port_present_remap_reg(present);
        return ONLP_STATUS_OK;
    }

    ports = &g_sfpiPortStat.port_at_pim[pim];
    cur = time (NULL);
    elapse = cur - ports->last_poll;

    if (!ports->valid || (elapse > PORT_POLL_INTERVAL)) {
        ret = get_ports_presence(pim, &present);
        if (ret < 0) {
            present  = 0;
            update_ports(pim, 0, present);
            *bit_array = present;        /*No needs for remmaped.*/
            return ONLP_STATUS_E_INTERNAL;
        } else {
            update_ports(pim, 1, present);
        }
    } else {
        present = ports->present;
    }
    *bit_array = _sfpi_port_present_remap_reg(present);
    return ONLP_STATUS_OK;
}

/*---------------Public APIs------------------------*/
int
onlp_sfpi_is_present(int port)
{
    /*
     * Return 1 if present.
     * Return 0 if not present.
     * Return < 0 if error.
     */
    int present, pim, ret;
    uint32_t bit_array;

    pim = PORT_TO_PIM(port);
    present = onlp_pim_is_present(pim);
    if (present < 0) {
        return present;
    }
    if (!present) {
        update_ports(pim, 0, 0);
        return 0;
    }
    ret = get_pim_port_present_bmap(port, &bit_array);
    if (ret < 0) {
        return ret;
    }

    return !!(bit_array & BIT(PORT_OF_PIM(port)));
}


int
onlp_sfpi_presence_bitmap_get(onlp_sfp_bitmap_t* dst)
{
    int i, port, ret;
    uint32_t bmp;
    uint32_t bmap_pim[NUM_OF_PIM] = {0};

    /*Get present bitmap per PIM.*/
    for (i = 0; i < NUM_OF_PIM; i++) {
        port = i*SFP_PORT_PER_PIM;
        ret = get_pim_port_present_bmap(port, &bmap_pim[i]);
        if (ret < 0)
            return ret;
    }

    for (i = 0; i < NUM_OF_SFP_PORT; i++) {
        AIM_BITMAP_CLR(dst, i);
    }
    for (i = 0; i < NUM_OF_SFP_PORT; i++) {
        bmp  = bmap_pim[PORT_TO_PIM(i)];
        if ((bmp & BIT(PORT_OF_PIM(i)))) {
            AIM_BITMAP_SET(dst, i);
        }
    }

    return ONLP_STATUS_OK;
}

int
onlp_sfpi_rx_los_bitmap_get(onlp_sfp_bitmap_t* dst)
{
    AIM_BITMAP_CLR_ALL(dst);
    return ONLP_STATUS_OK;
}

int
onlp_sfpi_control_set(int port, onlp_sfp_control_t control, int value)
{
    uint32_t pbmp;
    int rv = ONLP_STATUS_OK;
    if (port < 0) {
        return ONLP_STATUS_E_UNSUPPORTED;
    }

    switch(control)
    {
    case ONLP_SFP_CONTROL_RESET:
        rv = get_ports_reset(PORT_TO_PIM(port), &pbmp);
        if (rv < 0) {
            AIM_LOG_ERROR("Unable to get_ports_lpmode for port(%d)\r\n", port);
            rv = ONLP_STATUS_E_INTERNAL;
        }
        if (value) {
            pbmp |= BIT(PORT_OF_PIM(port));
        } else {
            pbmp &= ~BIT(PORT_OF_PIM(port));
        }
        rv = set_ports_reset(PORT_TO_PIM(port), pbmp);
        if (rv < 0) {
            AIM_LOG_ERROR("Unable to set_ports_lpmode for port(%d)\r\n", port);
            rv = ONLP_STATUS_E_INTERNAL;
        }
        break;
    case ONLP_SFP_CONTROL_LP_MODE:
        rv = get_ports_lpmode(PORT_TO_PIM(port), &pbmp);
        if (rv < 0) {
            AIM_LOG_ERROR("Unable to get_ports_lpmode for port(%d)\r\n", port);
            rv = ONLP_STATUS_E_INTERNAL;
        }
        if(value) {
            pbmp |= BIT(PORT_OF_PIM(port));
        } else {
            pbmp &= ~BIT(PORT_OF_PIM(port));
        }
        rv = set_ports_lpmode(PORT_TO_PIM(port), pbmp);
        if (rv < 0) {
            AIM_LOG_ERROR("Unable to set_ports_lpmode for port(%d)\r\n", port);
            rv = ONLP_STATUS_E_INTERNAL;
        }
        break;
    default:
        rv = ONLP_STATUS_E_UNSUPPORTED;
    }

    return rv;
}

int
onlp_sfpi_control_get(int port, onlp_sfp_control_t control, int* value)
{
    uint32_t pbmp;
    int rv = ONLP_STATUS_OK;
    if (port < 0) {
        return ONLP_STATUS_E_UNSUPPORTED;
    }

    switch(control)
    {
    case ONLP_SFP_CONTROL_RESET:
        rv = get_ports_reset(PORT_TO_PIM(port), &pbmp);
        if (rv < 0) {
            AIM_LOG_ERROR("Unable to get_ports_reset for port(%d)\r\n", port);
            rv = ONLP_STATUS_E_INTERNAL;
        }
        else {
            *value = !(pbmp & BIT(PORT_OF_PIM(port)));
            rv = ONLP_STATUS_OK;
        }
        break;

    case ONLP_SFP_CONTROL_LP_MODE:
        rv = get_ports_lpmode(PORT_TO_PIM(port), &pbmp);
        if (rv < 0) {
            AIM_LOG_ERROR("Unable to get_ports_lpmode for port(%d)\r\n", port);
            rv = ONLP_STATUS_E_INTERNAL;
        }
        else {
            *value = !!(pbmp & BIT(PORT_OF_PIM(port)));
            rv = ONLP_STATUS_OK;
        }
        break;

    default:
        rv = ONLP_STATUS_E_UNSUPPORTED;
    }

    return rv;
}

int
onlp_sfpi_denit(void)
{
    return ONLP_STATUS_OK;
}


/*Access to FPGA register.*/
#define FPGA_RESOURCE_NODE "/sys/devices/pci0000:00/0000:00:03.0/0000:05:00.0/resource0"
#define FPGA_RESOURCE_LENGTH 0x80000
static int hw_handle = -1;
static void *io_base = NULL;

static int fbfpgaio_hw_init(void)
{
    const char fpga_resource_node[] = FPGA_RESOURCE_NODE;

    if (io_base != NULL && io_base != MAP_FAILED) {
        return ONLP_STATUS_OK;
    }

    /* Open hardware resource node */
    hw_handle = open(fpga_resource_node, O_RDWR|O_SYNC);
    if (hw_handle == -1) {
        AIM_LOG_ERROR("%d %s\\n",errno,strerror(errno));
        return ONLP_STATUS_E_INTERNAL;
    }

    /* Mapping hardware resource */
    io_base = mmap(NULL, FPGA_RESOURCE_LENGTH, PROT_READ|PROT_WRITE, MAP_SHARED|MAP_NORESERVE, hw_handle, 0);
    if (io_base == MAP_FAILED) {
        AIM_LOG_ERROR("%d %s\\n",errno,strerror(errno));
        return ONLP_STATUS_E_INTERNAL;
    }

    return ONLP_STATUS_OK;
}

static uint32_t fbfpgaio_write(uint32_t addr, uint32_t input_data)
{
    int ret = fbfpgaio_hw_init();

    if (ONLP_STATUS_OK != ret) {
        return 0;
    }

    unsigned int *address =  (unsigned int *) ((unsigned long) io_base
                             + (unsigned long) addr);
    unsigned int data = (unsigned int) (input_data & 0xFFFFFFFF);
    *address = data;
    return ONLP_STATUS_OK;
}

static uint32_t fbfpgaio_read(uint32_t addr)
{
    int ret = fbfpgaio_hw_init();

    if (ONLP_STATUS_OK != ret) {
        return 0;
    }

    void *offset = io_base + addr;
    return *(uint32_t*)offset;
}

#define IOB_PIM_STATUS_REG 0x40

static const uint32_t DomOffset[] = {
    0x40000,
    0x48000,
    0x50000,
    0x58000,
    0x60000,
    0x68000,
    0x70000,
    0x78000,
};
#define QSFP_PRESENT_REG    0x48
#define QSFP_RESET_REG      0x70
#define QSFP_LPMODE_REG     0x78

int onlp_read_pim_present(uint32_t *pbmp) {
    uint32_t pim_status = fbfpgaio_read(IOB_PIM_STATUS_REG );
    *pbmp = (pim_status >> 16); /*bit 23~16*/
    return ONLP_STATUS_OK;
}

static int read_pom_reg(uint32_t pimId, uint32_t reg, uint32_t *pbmp) {
    if (pimId >= AIM_ARRAYSIZE(DomOffset)) {
        return ONLP_STATUS_E_INTERNAL;
    }
    uint32_t addr = DomOffset[pimId] + reg;
    *pbmp = fbfpgaio_read(addr);
    return ONLP_STATUS_OK;
}

static int get_ports_presence(uint32_t pimId, uint32_t *pbmp) {
    return read_pom_reg(pimId, QSFP_PRESENT_REG, pbmp);
}

static int get_ports_lpmode(uint32_t pimId, uint32_t *pbmp) {
    read_pom_reg(pimId, QSFP_RESET_REG, pbmp);
    return read_pom_reg(pimId, QSFP_LPMODE_REG, pbmp);
}

static int get_ports_reset(uint32_t pimId, uint32_t *pbmp) {
    return read_pom_reg(pimId, QSFP_RESET_REG, pbmp);
}

static int write_pom_reg(uint32_t pimId, uint32_t reg, uint32_t value) {
    if (pimId >= AIM_ARRAYSIZE(DomOffset)) {
        return ONLP_STATUS_E_INTERNAL;
    }

    uint32_t addr = DomOffset[pimId] + reg;
    return fbfpgaio_write(addr, value);
}

static int set_ports_lpmode(uint32_t pimId, uint32_t value) {
    return write_pom_reg(pimId, QSFP_LPMODE_REG, value);
}

static int set_ports_reset(uint32_t pimId, uint32_t value) {
    return write_pom_reg(pimId, QSFP_RESET_REG, value);
}


static int
config_sideBands(int port)
{
    uint32_t data, pim, pip;

    pim = PORT_TO_PIM(port);
    pip = PORT_OF_PIM(port);

    /*Set reset = 0*/
    read_pom_reg(pim, QSFP_RESET_REG, &data);
    data &= ~(BIT(pip));
    write_pom_reg(pim, QSFP_RESET_REG, data);
    AIM_USLEEP(10000);
    write_pom_reg(pim, QSFP_RESET_REG, data);
    return 0;
}

#define RTC_MAX_LEN     128
struct {
    uint32_t  desc;
    uint32_t  status;
    uint32_t  wbuff;
    uint32_t  rbuff;
} static const RTCRegOffset[] = {
    {0x500, 0x600, 0x2000, 0x3000},
    {0x520, 0x604, 0x2200, 0x3200},
    {0x540, 0x608, 0x2400, 0x3400},
    {0x560, 0x60c, 0x2600, 0x3600},
    {0x580, 0x610, 0x2800, 0x3800},
};

typedef enum {
    DO_WRITE = 0,
    DO_READ = 1,
} RorW;

static int sleep_withRV(int us) {
    AIM_USLEEP(us);
    return 1;
}
static uint32_t
assemble_cmd_w0(bool read, uint8_t length)
{
    uint8_t len = (length > RTC_MAX_LEN)? RTC_MAX_LEN: length;

    return (read<<28) | len;
}

static uint32_t
assemble_cmd_w1(uint8_t channel, uint8_t bank, uint8_t page, uint8_t offset)
{
    return (0x1 << 31) + (channel << 24) + (bank << 16) + (page << 8) + offset;
}

static int accessViaFPGA
(int port, RorW toRead, uint8_t pos, uint32_t maxlen, uint32_t* data)
{
    uint32_t reg, mask, bp, pim, pip;
    uint32_t rtc, ch, desc, cmd, len;

    config_sideBands(port);

    pim = PORT_TO_PIM(port);
    pip = PORT_OF_PIM(port);
    rtc = pip / 4;
    ch  = pip % 4;
    desc = 0;   /*Only use descriptor 0*/

    /*Write 1 to clear flags*/
    mask = 3;
    bp = (desc*4);
    write_pom_reg(pim, RTCRegOffset[rtc].status, mask << bp);
    read_pom_reg(pim, RTCRegOffset[rtc].status, &reg);
    if ((reg >> bp) &  mask) {
        AIM_LOG_ERROR("Unable to clear RTC status for port(%d)(reg:0x%08x)", port, reg);
        return ONLP_STATUS_E_INTERNAL;
    }

    len = (maxlen+3)/4*4;   /*for word-aligned*/
    len = (len > RTC_MAX_LEN)? RTC_MAX_LEN : len;
    if (toRead == DO_WRITE) {
        uint32_t i;
        for(i=0; i<len; i+=4) {
            write_pom_reg(pim, RTCRegOffset[rtc].wbuff+i, data[i]);
        }
    }

    /*Issue command(2 words) */
    cmd = assemble_cmd_w0(toRead, len);
    write_pom_reg(pim, RTCRegOffset[rtc].desc, cmd);

    cmd = assemble_cmd_w1(ch, 0, 0, pos);
    write_pom_reg(pim, RTCRegOffset[rtc].desc+4, cmd);

    /*wait for transacting.*/
    AIM_USLEEP(10000 + len*100);

    /*check if completed*/
    uint32_t retry = 100;
    do {
        read_pom_reg(pim, RTCRegOffset[rtc].status, &reg);
        retry--;
        if(retry == 0) {
            AIM_LOG_ERROR("Unable to RTC transaction failed(%02x), port(%d)",reg, port);
            return ONLP_STATUS_E_INTERNAL;
        }
    } while(!((reg >> bp) & 1) && sleep_withRV(10000));

    /*Copy data from FPGA buffer*/
    if (toRead == DO_READ) {
        uint32_t i;
        for(i=0; i<len; i+=4) {
            read_pom_reg(pim, RTCRegOffset[rtc].rbuff+i, data++);
        }
    }
    return 0;
}

int
onlp_sfpi_eeprom_read(int port, uint8_t data[256])
{
    int rv, i, m;

    m = RTC_MAX_LEN;
    for (i=0; i < 256; i += m) {
        rv = accessViaFPGA(port, DO_READ, i, m, (uint32_t*)&data[i]);
        if (rv) {
            return rv;
        }
    }
    return rv;
}

int
onlp_sfpi_dev_writeb(int port, uint8_t devaddr, uint8_t addr, uint8_t value)
{
    int rv;
    uint32_t data = value;

    if (devaddr != 0x50) {
        AIM_LOG_ERROR("Unable to read for addr 0x%02x of port(%d)\r\n", devaddr, port);
        return ONLP_STATUS_E_INTERNAL;
    }
    rv = accessViaFPGA(port, DO_WRITE, addr, 1, &data);
    return rv;
}

int
onlp_sfpi_dev_writew(int port, uint8_t devaddr, uint8_t addr, uint16_t value)
{
    int rv;

    rv = onlp_sfpi_dev_writeb(port, devaddr, addr, value & 0xff);
    if(rv < 0) {
        return rv;
    }
    rv = onlp_sfpi_dev_writeb(port, devaddr, addr+1, value >> 8);
    return rv;
}

int
onlp_sfpi_dev_readb(int port, uint8_t devaddr, uint8_t addr) {
    int rv;
    uint32_t data;

    if (devaddr != 0x50) {
        AIM_LOG_ERROR("Unable to read for addr 0x%02x of port(%d)\r\n", devaddr, port);
        return ONLP_STATUS_E_INTERNAL;
    }

    rv = accessViaFPGA(port, DO_READ, addr, 1, &data);
    if(rv) {
        return rv;
    }
    return (data & 0xff);
}

int
onlp_sfpi_dev_readw(int port, uint8_t devaddr, uint8_t addr) {

    /*Border problem.*/
    if((addr % RTC_MAX_LEN) == (RTC_MAX_LEN - 1)) {
        int data[2], rv;

        rv = onlp_sfpi_dev_readb(port, devaddr, addr);
        if(rv < 0) {
            return rv;
        }
        data[0]= rv;
        rv = onlp_sfpi_dev_readb(port, devaddr, addr + 1);
        if(rv < 0) {
            return rv;
        }
        data[1]= rv;

        return data[0] | (data[1] << 8);
    } else {
        int rv;
        uint32_t data;  //Assume little endian here.

        if (devaddr != 0x50) {
            AIM_LOG_ERROR("Unable to read for addr 0x%02x of port(%d)\r\n", devaddr, port);
            return ONLP_STATUS_E_INTERNAL;
        }

        rv = accessViaFPGA(port, DO_READ, addr, 2, &data);
        if(rv) {
            return rv;
        }
        return (uint16_t)data;
    }
}

