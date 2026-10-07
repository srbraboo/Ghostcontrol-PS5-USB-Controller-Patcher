/* SPDX-License-Identifier: GPL-3.0-or-later
 * Ghost-Control v5: Multi-controller support
 * USB HID controllers → virtual DualSense devices on PS5
 *
 * Supports up to MAX_SLOTS (4) simultaneous controllers:
 *   - 8BitDo / Nintendo Switch Pro  (VID=057E PID=2009)
 *   - Xbox One S                    (VID=045E PID=02EA)
 *
 * Each detected controller gets its own VDA device + force_bind
 * assignment dialog + VDI injection thread.
 * Hotplug: plug in any time, disconnect any time.
 */

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <pthread.h>
#include <fcntl.h>
#include <signal.h>
#include <time.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <dev/usb/usb.h>
#include <dev/usb/usb_ioctl.h>
#include <dev/usb/usb_endian.h>
#include <dev/usb/usbhid.h>

#ifdef __PROSPERO__
#include <ps5/kernel.h>
#include <ps5/klog.h>
#include <ps5/mdbg.h>
#endif

#include "shellui_pad.h"
#include "gc_types.h"
#include "usb_helpers.h"
#include "controller_nintendo.h"
#include "controller_xbox.h"
#include "controller_steam.h"

/* ── Logging ──────────────────────────────────────────────────────────── */
#define LOG_DIR  "/data/ghostpad"
#define LOG_PATH "/data/ghostpad/gc_status.log"
#define PID_PATH "/data/ghostpad/gc_main.pid"
#define LOG_MAX  480

static pthread_mutex_t g_log_lock = PTHREAD_MUTEX_INITIALIZER;
static int g_log_fd = -1;

void ghostpad_status_log_reset(void) {
    pthread_mutex_lock(&g_log_lock);
    if (g_log_fd >= 0) { close(g_log_fd); g_log_fd = -1; }
    mkdir(LOG_DIR, 0755);
    g_log_fd = open(LOG_PATH, O_WRONLY|O_CREAT|O_TRUNC, 0600);
    pthread_mutex_unlock(&g_log_lock);
}

void ghostpad_status_log(const char *fmt, ...) {
    char buf[LOG_MAX]; va_list ap;
    va_start(ap, fmt); vsnprintf(buf, sizeof(buf)-1, fmt, ap); va_end(ap);
    buf[LOG_MAX-1] = '\0';
    klog_printf("%s", buf);
    pthread_mutex_lock(&g_log_lock);
    if (g_log_fd >= 0) {
        size_t n = strnlen(buf, sizeof(buf));
        write(g_log_fd, buf, n);
        if (n && buf[n-1] != '\n') write(g_log_fd, "\n", 1);
    }
    pthread_mutex_unlock(&g_log_lock);
}
#define gp_log(...) ghostpad_status_log("[GC] " __VA_ARGS__)

/* ── SCE stubs ────────────────────────────────────────────────────────── */
extern int32_t sceUserServiceInitialize(void *params);
extern int32_t sceUserServiceGetInitialUser(int32_t *outUserId);
extern int32_t sceUserServiceGetForegroundUser(int32_t *outUserId);
extern int32_t scePadInit(void);
extern int32_t scePadSetProcessPrivilege(int32_t privilege);
extern int32_t scePadGetHandle(int32_t userId, int32_t type, int32_t index);
extern int32_t scePadVirtualDeviceAddDevice(void *param, int32_t deviceType);
extern int32_t scePadVirtualDeviceDeleteDevice(int32_t handle);
extern int32_t scePadVirtualDeviceInsertData(int32_t handle, const void *padData);
extern int32_t sceKernelSendNotificationRequest(int unk0, void *req, size_t size, int unk1);

#define VIRTUAL_DEVICE_TYPE_DUALSENSE 3

/* ── Multi-controller slots ───────────────────────────────────────────── */
#define MAX_SLOTS 4

typedef struct {
    volatile int32_t handle;        /* VDA handle, -1 = slot free */
    volatile int     vdi_ready;
    volatile int     usb_active;    /* USB reader thread is running */
    volatile int     confirmed;     /* user pressed a button — assignment done */
    char             dev_path[32];  /* ugen path claimed by this slot */
    uint16_t         vid, pid;
    volatile uint32_t inject_count;
} ctrl_slot_t;

static ctrl_slot_t     g_slots[MAX_SLOTS];
static pthread_mutex_t g_slot_lock = PTHREAD_MUTEX_INITIALIZER;
static int32_t         g_inject_uid = 0x10000000;

/* Assignment serialization: only ONE controller may show its assignment
 * dialog at a time. g_assign_slot = slot awaiting user confirmation, or -1.
 * Without this, multiple dialogs stack and all bind to the same user. */
static volatile int    g_assign_slot = -1;

/* ── klog device-ID queue ─────────────────────────────────────────────── */
#define KLOG_QSIZE 16
static uint64_t        g_klog_q[KLOG_QSIZE];
static int             g_klog_qw = 0, g_klog_qr = 0;
static pthread_mutex_t g_klog_lock = PTHREAD_MUTEX_INITIALIZER;

static void klog_enqueue(uint64_t id) {
    pthread_mutex_lock(&g_klog_lock);
    int next = (g_klog_qw + 1) % KLOG_QSIZE;
    if (next != g_klog_qr) { g_klog_q[g_klog_qw] = id; g_klog_qw = next; }
    pthread_mutex_unlock(&g_klog_lock);
}

static uint64_t klog_dequeue_ms(int ms) {
    for (int t = 0; t < ms; t += 100) {
        pthread_mutex_lock(&g_klog_lock);
        if (g_klog_qw != g_klog_qr) {
            uint64_t id = g_klog_q[g_klog_qr];
            g_klog_qr = (g_klog_qr + 1) % KLOG_QSIZE;
            pthread_mutex_unlock(&g_klog_lock);
            return id;
        }
        pthread_mutex_unlock(&g_klog_lock);
        usleep(100000);
    }
    return 0;
}

/* ── Notification ─────────────────────────────────────────────────────── */
typedef struct { char _unk[45]; char message[3075]; } NotifyRequest;
static void notify(const char *fmt, ...) {
    NotifyRequest req; va_list ap;
    memset(&req, 0, sizeof(req));
    va_start(ap, fmt); vsnprintf(req.message, sizeof(req.message), fmt, ap); va_end(ap);
    sceKernelSendNotificationRequest(0, &req, sizeof(req), 0);
}

/* ── klog capture thread ──────────────────────────────────────────────── */
static uint64_t parse_hex_str(const char *s) {
    uint64_t v = 0;
    while (*s) {
        char c = *s++;
        if      (c >= '0' && c <= '9') v = (v<<4)|(c-'0');
        else if (c >= 'a' && c <= 'f') v = (v<<4)|(c-'a'+10);
        else if (c >= 'A' && c <= 'F') v = (v<<4)|(c-'A'+10);
        else break;
    }
    return v;
}

static void parse_klog_line(const char *line) {
    if (!strstr(line, "DEVICE_ADDED"))        return;
    if (!strstr(line, "subType:22"))          return;
    if (!strstr(line, "capabilityBattery:0")) return;
    const char *p = strstr(line, "DeviceId:0x");
    if (!p) p = strstr(line, "deviceId=0x");
    if (!p) return;
    p += 11;
    uint64_t id = parse_hex_str(p);
    if (!id) return;
    gp_log("klog: VDA device 0x%llx\n", (unsigned long long)id);
    klog_enqueue(id);
}

static void *klog_capture_thread(void *arg) {
    (void)arg;
    char buf[512], line[1024]; size_t line_len = 0;
    int fd = open("/dev/klog", O_RDONLY);
    if (fd < 0) {
        gp_log("klog: /dev/klog errno=%d, trying TCP 127.0.0.1:3232\n", errno);
        struct sockaddr_in sin; int sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0) return NULL;
        memset(&sin,0,sizeof(sin)); sin.sin_family=AF_INET;
        sin.sin_addr.s_addr=inet_addr("127.0.0.1"); sin.sin_port=htons(3232);
        for (int r=0; r<10; r++) {
            if (connect(sock,(struct sockaddr*)&sin,sizeof(sin))==0){fd=sock;gp_log("klog: connected\n");break;}
            usleep(500000);
        }
        if (fd<0){close(sock);return NULL;}
    }
    while (1) {
        ssize_t len = read(fd, buf, sizeof(buf));
        if (len < 0) { if (errno==EINTR) continue; break; }
        if (len == 0) { usleep(10000); continue; }
        for (ssize_t i=0; i<len; i++) {
            char c = buf[i];
            if (c=='\n'||line_len>=sizeof(line)-1){line[line_len]='\0';parse_klog_line(line);line_len=0;}
            else if (c!='\r') line[line_len++]=c;
        }
    }
    close(fd); return NULL;
}

/* ── VDI injection ────────────────────────────────────────────────────── */
static void inject_pad(int slot, const ScePadData *pad) {
    int32_t h = g_slots[slot].handle;
    if (h < 0 || !g_slots[slot].vdi_ready) return;
    int vr = scePadVirtualDeviceInsertData(h, pad);
    uint32_t n = ++g_slots[slot].inject_count;
    if ((n % 600) == 0)
        gp_log("slot[%d] VDI #%u ret=0x%08x\n", slot, n, (uint32_t)vr);
    static int vdi_err_logged = 0;
    if (vr != 0 && !vdi_err_logged) { gp_log("VDI error 0x%08x\n",(uint32_t)vr); vdi_err_logged=1; }
}

/* ── ugen detection ───────────────────────────────────────────────────── */
#define VID_NATIVE  0x2dc8u
#define PID_NATIVE  0x310bu
#define VID_SWITCH  0x057eu
#define PID_SWITCH  0x2009u
#define VID_XBOX    0x045eu
#define PID_XBOX    0x02eau
#define VID_STEAM   STEAM_VID
#define PID_STEAM   STEAM_PID
#define PID_STEAM_PUCK STEAM_PUCK_PID
#define PID_STEAM_PUCK_ALT STEAM_PUCK_ALT_PID

static const char *UGEN_PATHS[] = {
    "/dev/ugen2.2","/dev/ugen2.3","/dev/ugen2.4","/dev/ugen2.5",
    "/dev/ugen2.6","/dev/ugen2.7","/dev/ugen2.8","/dev/ugen2.9",
    "/dev/ugen1.2","/dev/ugen0.2","/dev/ugen0.3",
};
#define N_UGEN_PATHS ((int)(sizeof(UGEN_PATHS)/sizeof(UGEN_PATHS[0])))

static int steam_puck_disable_lizard(int fd, int slot) {
    uint8_t feature[64];
    struct usb_gen_descriptor report;
    struct usb_ctl_request request;
    int iface;

    memset(feature, 0, sizeof(feature));
    feature[0] = 0x01;
    feature[1] = 0x87;
    feature[2] = 0x03;
    feature[3] = 0x09;

    /* Proteus/Nereid expose controller slots as interfaces 2..5. Send the
     * HID class request directly so the composite device cannot route the
     * feature report to its keyboard/mouse interface. */
    for (iface = 2; iface <= 5; iface++) {
        memset(&request, 0, sizeof(request));
        request.ucr_data = feature;
        request.ucr_flags = 0;
        request.ucr_request.bmRequestType = UT_WRITE_CLASS_INTERFACE;
        request.ucr_request.bRequest = UR_SET_REPORT;
        USETW(request.ucr_request.wValue,
              (uint16_t)((UHID_FEATURE_REPORT << 8) | 0x01));
        USETW(request.ucr_request.wIndex, (uint16_t)iface);
        USETW(request.ucr_request.wLength, sizeof(feature));
        if (ioctl(fd, USB_DO_REQUEST, &request) == 0) {
            gp_log("slot[%d] puck lizard mode disabled via iface %d\n",
                   slot, iface);
            return 0;
        }
    }

    /* Keep a HID ioctl fallback for PS5 firmware revisions that reject
     * USB_DO_REQUEST on the ugen node. */
    memset(&report, 0, sizeof(report));
    report.ugd_data = feature;
    report.ugd_maxlen = sizeof(feature);
    report.ugd_report_type = UHID_FEATURE_REPORT;
    for (iface = 2; iface <= 5; iface++) {
        report.ugd_iface_index = (uint8_t)iface;
        if (ioctl(fd, USB_SET_REPORT, &report) == 0) {
            gp_log("slot[%d] puck lizard mode disabled via HID iface %d\n",
                   slot, iface);
            return 0;
        }
    }
    report.ugd_iface_index = USB_IFACE_INDEX_ANY;
    if (ioctl(fd, USB_SET_REPORT, &report) == 0) {
        gp_log("slot[%d] puck lizard mode disabled on any iface\n", slot);
        return 0;
    }
    gp_log("slot[%d] puck lizard-off failed errno=%d\n", slot, errno);
    return -1;
}

/* Probe one ugen2.x path to identify controller type.
 * Returns 1 with vid/pid set, 0 if not a known controller.
 * Skips non-ugen2 paths (not on external USB bus). */
static int probe_one_path(const char *path, uint16_t *out_vid, uint16_t *out_pid) {
    if (strncmp(path, "/dev/ugen2.", 11) != 0) return 0;

    int fd = open(path, O_RDWR|O_NONBLOCK);
    if (fd < 0) return 0;

    /* Identify Valve's composite receiver before probing generic HID
     * endpoints. Its first interface may be the short mouse report used by
     * lizard mode, which otherwise gets mistaken for a Nintendo pad. */
    struct usb_device_info dinfo;
    memset(&dinfo, 0, sizeof(dinfo));
    int device_info_ok = (ioctl(fd, USB_DEVICEINFO, &dinfo) == 0);
    if (device_info_ok)
        gp_log("probe: %s device VID=0x%04x PID=0x%04x\n",
               path, dinfo.udi_vendorNo, dinfo.udi_productNo);
    if (device_info_ok &&
        dinfo.udi_vendorNo == STEAM_VID &&
        dinfo.udi_productNo >= STEAM_PUCK_PID &&
        dinfo.udi_productNo <= STEAM_PUCK_ALT_PID) {
        *out_vid = VID_STEAM;
        *out_pid = dinfo.udi_productNo;
        gp_log("probe: %s Valve receiver pid=0x%04x\n",
               path, dinfo.udi_productNo);
        close(fd);
        return 1;
    }

    struct usb_fs_endpoint ep;
    struct usb_fs_init ini;
    struct usb_fs_uninit u;
    memset(&ep,0,sizeof(ep)); memset(&ini,0,sizeof(ini));
    ini.pEndpoints=&ep; ini.ep_index_max=1;
    if (ioctl(fd,USB_FS_INIT,&ini)!=0) { close(fd); return 0; }

    int ii=0; ioctl(fd,USB_IFACE_DRIVER_DETACH,&ii);
    ii=1;     ioctl(fd,USB_IFACE_DRIVER_DETACH,&ii);
    ii=2;     ioctl(fd,USB_IFACE_DRIVER_DETACH,&ii);

    struct usb_fs_open po;
    memset(&po,0,sizeof(po)); po.ep_index=0; po.max_bufsize=64; po.max_frames=1;

    int found = 0;

    /* Steam Controller 2: report-aware check avoids treating its 0x81
     * endpoint as a Nintendo controller merely because both use 64-byte
     * interrupt packets. */
    /* Nintendo: ep=0x81, maxpkt=64 */
    po.ep_no=0x81;
    if (ioctl(fd,USB_FS_OPEN,&po)==0 && po.max_packet_length==64) {
        uint8_t probe_buf[64]; memset(probe_buf, 0, sizeof(probe_buf));
        void *probe_ptrs[1] = { probe_buf };
        uint32_t probe_len[1] = { sizeof(probe_buf) };
        ep.ppBuffer = probe_ptrs; ep.pLength = probe_len;
        ep.nFrames = 1; ep.timeout = 100; ep.flags =
            USB_FS_FLAG_SINGLE_SHORT_OK | USB_FS_FLAG_MULTI_SHORT_OK;
        struct usb_fs_start probe_start;
        memset(&probe_start, 0, sizeof(probe_start));
        probe_start.ep_index = 0;
        if (ioctl(fd, USB_FS_START, &probe_start) == 0) {
            for (int w = 0; w < 4; w++) {
                struct usb_fs_complete probe_complete;
                memset(&probe_complete, 0, sizeof(probe_complete));
                probe_complete.ep_index = 0;
                if (ioctl(fd, USB_FS_COMPLETE, &probe_complete) == 0) break;
                if (errno != EBUSY) break;
                usleep(25000);
            }
        }
        if (probe_len[0] >= 18 && probe_buf[0] == STEAM_REPORT_STATE) {
            gp_log("probe: %s report=0x42 len=%u -> Steam Controller 2\n",
                   path, (unsigned)probe_len[0]);
            struct usb_fs_close pc; memset(&pc,0,sizeof(pc));
            pc.ep_index=0; ioctl(fd,USB_FS_CLOSE,&pc);
            *out_vid=VID_STEAM; *out_pid=PID_STEAM;
            found = 1;
            goto done;
        }
        gp_log("probe: %s ep=0x81 mpkt=%u → Nintendo\n", path,(unsigned)po.max_packet_length);
        struct usb_fs_close pc; memset(&pc,0,sizeof(pc)); pc.ep_index=0; ioctl(fd,USB_FS_CLOSE,&pc);
        *out_vid=VID_SWITCH; *out_pid=PID_SWITCH;
        found = 1;
        goto done;
    }

    /* Steam receiver: inspect every possible interrupt-IN endpoint. The
     * receiver may expose mouse/lizard data on 0x81 and controller data on a
     * later endpoint, so do not stop after the first HID interface. */
    { struct usb_fs_close pc;
      memset(&pc, 0, sizeof(pc));
      pc.ep_index = 0;
      ioctl(fd, USB_FS_CLOSE, &pc); }
    int receiver_candidate = 0;
    for (uint8_t puck_ep = 0x82; puck_ep <= 0x86; puck_ep++) {
        po.ep_no = puck_ep;
        if (ioctl(fd,USB_FS_OPEN,&po)!=0)
            continue;
        receiver_candidate++;
        uint8_t probe_buf[64]; memset(probe_buf, 0, sizeof(probe_buf));
        void *probe_ptrs[1] = { probe_buf };
        uint32_t probe_len[1] = { sizeof(probe_buf) };
        ep.ppBuffer = probe_ptrs; ep.pLength = probe_len; ep.nFrames = 1;
        ep.timeout = 100; ep.flags = USB_FS_FLAG_SINGLE_SHORT_OK | USB_FS_FLAG_MULTI_SHORT_OK;
        struct usb_fs_start probe_start; memset(&probe_start, 0, sizeof(probe_start));
        probe_start.ep_index = 0;
        if (ioctl(fd, USB_FS_START, &probe_start) == 0) {
            struct usb_fs_complete probe_complete; memset(&probe_complete, 0, sizeof(probe_complete));
            probe_complete.ep_index = 0; ioctl(fd, USB_FS_COMPLETE, &probe_complete);
        }
        struct usb_fs_close pc; memset(&pc, 0, sizeof(pc));
        pc.ep_index = 0; ioctl(fd, USB_FS_CLOSE, &pc);
        gp_log("probe: %s receiver ep=0x%02x report=0x%02x len=%u\n",
               path, puck_ep, probe_len[0] ? probe_buf[0] : 0,
               (unsigned)probe_len[0]);
        if (probe_len[0] >= 29 && probe_buf[0] == STEAM_REPORT_STATE) {
            gp_log("probe: %s report=0x42 on receiver endpoint -> Steam puck\n", path);
            *out_vid=VID_STEAM; *out_pid=PID_STEAM_PUCK;
            found = 1; goto done;
        }
    }
    if (receiver_candidate >= 1) {
        gp_log("probe: %s has %d Valve receiver endpoints; keeping puck type\n",
               path, receiver_candidate);
        *out_vid = VID_STEAM;
        *out_pid = PID_STEAM_PUCK;
        found = 1;
        goto done;
    }

    /* Xbox One: ep=0x82, maxpkt in (0,64] */
    memset(&po,0,sizeof(po)); po.ep_index=0; po.max_bufsize=64; po.max_frames=1;
    po.ep_no=0x82;
    if (ioctl(fd,USB_FS_OPEN,&po)==0 && po.max_packet_length>0 && po.max_packet_length<=64) {
        gp_log("probe: %s ep=0x82 mpkt=%u → Xbox One\n", path,(unsigned)po.max_packet_length);
        struct usb_fs_close pc; memset(&pc,0,sizeof(pc)); pc.ep_index=0; ioctl(fd,USB_FS_CLOSE,&pc);
        *out_vid=VID_XBOX; *out_pid=PID_XBOX;
        found = 1;
        goto done;
    }

done:
    memset(&u,0,sizeof(u)); ioctl(fd,USB_FS_UNINIT,&u);
    close(fd);
    return found;
}

/* ── Create VDA and force_bind for a slot ─────────────────────────────── */
static int32_t create_vda_for_slot(int slot) {
    struct { int32_t size; int32_t userId; int32_t pad[6]; } vdp;
    const int32_t SEN = (int32_t)0xDEADBEEFu;
    memset(&vdp,0,sizeof(vdp)); vdp.size=sizeof(vdp); vdp.userId=1;
    for(int k=0;k<6;k++) vdp.pad[k]=SEN;

    int ret = scePadVirtualDeviceAddDevice(&vdp, VIRTUAL_DEVICE_TYPE_DUALSENSE);
    gp_log("slot[%d] VDA ret=0x%08x\n", slot, (uint32_t)ret);

    int32_t handle = (ret > 0) ? ret : -1;
    for(int k=0;k<6;k++){
        if(vdp.pad[k]!=SEN && vdp.pad[k]>0){if(handle<0)handle=vdp.pad[k];break;}
    }

    uint64_t dev_id = klog_dequeue_ms(10000);
    if (dev_id) {
        handle = (int32_t)(dev_id & 0xffffffffu);
        int br = shellui_pad_force_bind(dev_id, g_inject_uid);
        gp_log("slot[%d] force_bind(0x%llx, 0x%08x) ret=%d\n",
               slot, (unsigned long long)dev_id, (uint32_t)g_inject_uid, br);
    } else if (handle >= 0) {
        gp_log("slot[%d] klog timeout — using direct handle %d\n", slot, handle);
    } else {
        gp_log("slot[%d] GetHandle scan...\n", slot);
        static const int32_t uids[]={1,0x10000000,(int32_t)0xffffffff};
        for(int ui=0;ui<3&&handle<0;ui++)
            for(int idx=0;idx<8&&handle<0;idx++){
                handle=scePadGetHandle(uids[ui],3,idx);
                if(handle>=0) gp_log("slot[%d] GetHandle uid=0x%08x idx=%d h=%d\n",
                                     slot,(uint32_t)uids[ui],idx,handle);
            }
    }
    if (handle>=0)
        gp_log("slot[%d] VDI handle=0x%x ready\n", slot, (uint32_t)handle);
    else
        gp_log("slot[%d] ERROR: no VDA handle\n", slot);
    return handle;
}

/* ── USB HID thread ───────────────────────────────────────────────────── */
/* Single-session: receives slot+path+vid+pid, runs until disconnect, then exits.
 * Manager thread handles re-detection after exit. */

typedef struct {
    int      slot;
    char     dev_path[32];
    uint16_t vid, pid;
} usb_thread_arg_t;

static void *usb_hid_thread(void *arg) {
    usb_thread_arg_t *targ = (usb_thread_arg_t *)arg;
    int slot = targ->slot;
    char dev_path[32]; memcpy(dev_path, targ->dev_path, sizeof(dev_path));
    uint16_t vid = targ->vid, pid = targ->pid;
    free(targ);

    struct usb_fs_endpoint eps[2];
    struct usb_fs_init     init;
    struct usb_fs_open     fs_open;
    struct usb_fs_start    start;
    struct usb_fs_complete complete;
    struct usb_fs_stop     stop;
    struct usb_fs_close    fs_close;
    struct usb_fs_uninit   uninit;
    uint8_t  buf[64];
    void    *buffers[1]; uint32_t lengths[1];
    int fd = -1, out_opened = 0;
    int usb_ready_notified = 0;
    uint64_t steam_lizard_keepalive_ms = 0;
    int steam_first_report_logged = 0;

    gp_log("slot[%d] USB thread: %s VID=0x%04x PID=0x%04x\n",
           slot, dev_path, vid, pid);

    /* ── Steam Controller 2: native HID state reports ─────────────────── */
    if (pid == PID_STEAM || pid == PID_STEAM_PUCK || pid == PID_STEAM_PUCK_ALT) {
        fd = open(dev_path, O_RDWR);
        if (fd < 0) {
            gp_log("slot[%d] Steam Controller 2 open fail errno=%d\n", slot, errno);
            goto exit_slot;
        }

        { int ii; for (ii = 0; ii < 4; ii++) {
            int iface = ii;
            ioctl(fd, USB_IFACE_DRIVER_DETACH, &iface);
        } }
        usleep(120000);
        memset(eps, 0, sizeof(eps)); memset(&init, 0, sizeof(init));
        init.pEndpoints = eps; init.ep_index_max = 1;
        if (ioctl(fd, USB_FS_INIT, &init) != 0) {
            gp_log("slot[%d] Steam FS_INIT fail errno=%d\n", slot, errno);
            close(fd); goto exit_slot;
        }

        if (pid == PID_STEAM_PUCK || pid == PID_STEAM_PUCK_ALT)
            steam_puck_disable_lizard(fd, slot);
        steam_lizard_keepalive_ms =
            (uint64_t)time(NULL) * 1000u;

        memset(&fs_open, 0, sizeof(fs_open));
        fs_open.ep_index = 0;
        fs_open.ep_no = STEAM_EP_IN;
        fs_open.max_bufsize = 64; fs_open.max_frames = 1;
        if (pid == PID_STEAM_PUCK || pid == PID_STEAM_PUCK_ALT) {
            /* The receiver exposes a short mouse endpoint while lizard mode
             * is active. Do not bind that endpoint as controller input:
             * search the interrupt-IN endpoints for a gamepad-sized packet. */
            static const uint8_t puck_eps[] = {
                0x82, 0x83, 0x84, 0x85, 0x86, 0x81
            };
            int opened = 0;
            int state_endpoint = 0;
            size_t pi;
            for (pi = 0; pi < sizeof(puck_eps); pi++) {
                memset(&fs_open, 0, sizeof(fs_open));
                fs_open.ep_index = 0;
                fs_open.ep_no = puck_eps[pi];
                fs_open.max_bufsize = 64;
                fs_open.max_frames = 1;
                if (ioctl(fd, USB_FS_OPEN, &fs_open) == 0) {
                    opened = 1;
                    gp_log("slot[%d] puck candidate endpoint 0x%02x maxpkt=%u\n",
                           slot, fs_open.ep_no,
                           (unsigned)fs_open.max_packet_length);
                    buffers[0] = buf; lengths[0] = 64;
                    eps[0].ppBuffer = buffers; eps[0].pLength = lengths;
                    eps[0].nFrames = 1; eps[0].timeout = 150;
                    eps[0].flags = USB_FS_FLAG_SINGLE_SHORT_OK |
                                   USB_FS_FLAG_MULTI_SHORT_OK;
                    memset(buf, 0, sizeof(buf));
                    memset(&start, 0, sizeof(start));
                    start.ep_index = 0;
                    if (ioctl(fd, USB_FS_START, &start) == 0) {
                        for (int wait = 0; wait < 20; wait++) {
                            memset(&complete, 0, sizeof(complete));
                            complete.ep_index = 0;
                            if (ioctl(fd, USB_FS_COMPLETE, &complete) == 0)
                                break;
                            if (errno != EBUSY) break;
                            usleep(5000);
                        }
                    }
                    if (lengths[0] > 0 && buf[0] == STEAM_REPORT_STATE) {
                        state_endpoint = 1;
                        gp_log("slot[%d] puck state endpoint 0x%02x selected\n",
                               slot, fs_open.ep_no);
                        break;
                    }
                    gp_log("slot[%d] puck endpoint 0x%02x report=0x%02x; skipping\n",
                           slot, fs_open.ep_no, lengths[0] ? buf[0] : 0);
                    { struct usb_fs_close close_ep;
                      memset(&close_ep, 0, sizeof(close_ep));
                      close_ep.ep_index = 0;
                      ioctl(fd, USB_FS_CLOSE, &close_ep); }
                }
            }
            if (!opened || !state_endpoint) {
                gp_log("slot[%d] puck has no 0x42 controller-state endpoint\n",
                       slot);
                goto uninit_exit;
            }
        } else if (ioctl(fd, USB_FS_OPEN, &fs_open) != 0) {
            if (pid == PID_STEAM ||
                (fs_open.ep_no = STEAM_PUCK_EP_IN,
                 ioctl(fd, USB_FS_OPEN, &fs_open) != 0)) {
                gp_log("slot[%d] Steam IN fail errno=%d\n", slot, errno);
                goto uninit_exit;
            }
        }
        gp_log("slot[%d] Steam IN ep=0x%02x maxpkt=%u\n",
               slot, fs_open.ep_no, (unsigned)fs_open.max_packet_length);

        buffers[0] = buf; lengths[0] = 64;
        eps[0].ppBuffer = buffers; eps[0].pLength = lengths;
        eps[0].nFrames = 1; eps[0].timeout = 50;
        eps[0].flags = USB_FS_FLAG_SINGLE_SHORT_OK | USB_FS_FLAG_MULTI_SHORT_OK;
        goto main_loop;
    }

    /* ── Xbox One: single-pass ─────────────────────────────────────────── */
    if (pid == PID_XBOX) {
        fd = open(dev_path, O_RDWR);
        if (fd < 0) { gp_log("slot[%d] Xbox open fail errno=%d\n", slot, errno); goto exit_slot; }

        { int ii; for(ii=0;ii<4;ii++){int i2=ii; ioctl(fd,USB_IFACE_DRIVER_DETACH,&i2);} }
        usleep(120000); /* hub settle: give USB hub time to propagate detach */
        memset(eps,0,sizeof(eps)); memset(&init,0,sizeof(init));
        init.pEndpoints=eps; init.ep_index_max=4;
        if (ioctl(fd,USB_FS_INIT,&init)!=0){
            gp_log("slot[%d] Xbox FS_INIT fail errno=%d\n",slot,errno);
            close(fd); goto exit_slot;
        }

        memset(&fs_open,0,sizeof(fs_open));
        fs_open.ep_index=0; fs_open.ep_no=XBOX_EP_IN;
        fs_open.max_bufsize=64; fs_open.max_frames=1;
        if (ioctl(fd,USB_FS_OPEN,&fs_open)!=0){
            gp_log("slot[%d] Xbox IN fail errno=%d\n",slot,errno);
            goto uninit_exit;
        }
        gp_log("slot[%d] Xbox IN ep=0x%02x ok maxpkt=%u\n",
               slot, XBOX_EP_IN, (unsigned)fs_open.max_packet_length);

        buffers[0]=buf; lengths[0]=64;
        eps[0].ppBuffer=buffers; eps[0].pLength=lengths; eps[0].nFrames=1;
        eps[0].timeout=50; eps[0].flags=USB_FS_FLAG_SINGLE_SHORT_OK|USB_FS_FLAG_MULTI_SHORT_OK;

        memset(&fs_open,0,sizeof(fs_open));
        fs_open.ep_index=1; fs_open.ep_no=XBOX_EP_OUT;
        fs_open.max_bufsize=64; fs_open.max_frames=1;
        out_opened = (ioctl(fd,USB_FS_OPEN,&fs_open)==0) ? 1 : 0;
        gp_log("slot[%d] Xbox OUT ep=0x%02x opened=%d\n", slot, XBOX_EP_OUT, out_opened);

        xbox_gip_handshake(fd, eps);
        goto main_loop;
    }

    /* ── Nintendo: two-pass ────────────────────────────────────────────── */
    fd = open(dev_path, O_RDWR);
    if (fd < 0) { gp_log("slot[%d] open fail errno=%d\n", slot, errno); goto exit_slot; }

    memset(eps,0,sizeof(eps)); memset(&init,0,sizeof(init));
    init.pEndpoints=eps; init.ep_index_max=1;
    if (ioctl(fd,USB_FS_INIT,&init)!=0){
        gp_log("slot[%d] Nintendo FS_INIT p1 fail\n",slot); close(fd); goto exit_slot;
    }
    { int i0=0,i1=1; ioctl(fd,USB_IFACE_DRIVER_DETACH,&i0); ioctl(fd,USB_IFACE_DRIVER_DETACH,&i1); }
    memset(&fs_open,0,sizeof(fs_open));
    fs_open.ep_index=0; fs_open.ep_no=0x81; fs_open.max_bufsize=64; fs_open.max_frames=1;
    if (ioctl(fd,USB_FS_OPEN,&fs_open)!=0){
        gp_log("slot[%d] Nintendo IN p1 fail errno=%d\n",slot,errno); goto uninit_exit;
    }
    gp_log("slot[%d] Nintendo p1 IN ok maxpkt=%u\n",slot,(unsigned)fs_open.max_packet_length);
    memset(&uninit,0,sizeof(uninit)); ioctl(fd,USB_FS_UNINIT,&uninit);
    close(fd); fd=-1;

    /* Pass 2: retry DETACH+OPEN up to 5 times to beat usb_hid0 re-attach
     * (real Switch Pro is claimed by PS5 native HID driver after probe releases it) */
    { int detach_try;
      for (detach_try = 0; detach_try < 5; detach_try++) {
        fd = open(dev_path, O_RDWR);
        if (fd < 0) { gp_log("slot[%d] reopen fail attempt %d\n",slot,detach_try); goto exit_slot; }
        { int i0=0,i1=1,i2=2;
          ioctl(fd,USB_IFACE_DRIVER_DETACH,&i0);
          ioctl(fd,USB_IFACE_DRIVER_DETACH,&i1);
          ioctl(fd,USB_IFACE_DRIVER_DETACH,&i2); }
        memset(eps,0,sizeof(eps)); memset(&init,0,sizeof(init));
        init.pEndpoints=eps; init.ep_index_max=2;
        if (ioctl(fd,USB_FS_INIT,&init)!=0){
            close(fd); fd=-1; usleep(50000); continue;
        }
        memset(&fs_open,0,sizeof(fs_open));
        fs_open.ep_index=0; fs_open.ep_no=0x81; fs_open.max_bufsize=64; fs_open.max_frames=1;
        if (ioctl(fd,USB_FS_OPEN,&fs_open)==0) break; /* claimed it */
        memset(&uninit,0,sizeof(uninit)); ioctl(fd,USB_FS_UNINIT,&uninit);
        close(fd); fd=-1;
        gp_log("slot[%d] Nintendo p2 OPEN retry %d errno=%d\n",slot,detach_try,errno);
        usleep(50000);
      }
      if (fd < 0) { gp_log("slot[%d] Nintendo p2 give up\n",slot); goto exit_slot; }
    }
    gp_log("slot[%d] Nintendo p2 IN ok maxpkt=%u\n",slot,(unsigned)fs_open.max_packet_length);
    if (fs_open.max_packet_length != 64){ gp_log("slot[%d] wrong maxpkt, reinit\n",slot); goto reinit; }

    buffers[0]=buf; lengths[0]=64;
    eps[0].ppBuffer=buffers; eps[0].pLength=lengths; eps[0].nFrames=1;
    eps[0].timeout=200; eps[0].flags=USB_FS_FLAG_SINGLE_SHORT_OK|USB_FS_FLAG_MULTI_SHORT_OK;

    /* 8BitDo uses ep=0x02; real Nintendo Switch Pro Controller uses ep=0x01 */
    memset(&fs_open,0,sizeof(fs_open));
    fs_open.ep_index=1; fs_open.ep_no=0x02; fs_open.max_bufsize=64; fs_open.max_frames=1;
    out_opened = (ioctl(fd,USB_FS_OPEN,&fs_open)==0) ? 1 : 0;
    if (!out_opened) {
        memset(&fs_open,0,sizeof(fs_open));
        fs_open.ep_index=1; fs_open.ep_no=0x01; fs_open.max_bufsize=64; fs_open.max_frames=1;
        out_opened = (ioctl(fd,USB_FS_OPEN,&fs_open)==0) ? 1 : 0;
        if (out_opened) gp_log("slot[%d] Nintendo OUT ep=0x01 (real Switch Pro)\n", slot);
    }
    gp_log("slot[%d] Nintendo OUT opened=%d\n", slot, out_opened);
    if (out_opened) {
        usb_send_cmd(fd,&eps[1],0x80,0x02); usleep(30000);
        usb_send_cmd(fd,&eps[1],0x80,0x04); usleep(50000);
        gp_log("slot[%d] Nintendo [80 02]+[80 04] sent\n", slot);
    }

main_loop: ;
    int hs_state = (pid==PID_XBOX || pid==PID_STEAM ||
                pid==PID_STEAM_PUCK || pid==PID_STEAM_PUCK_ALT)
                   ? HS_STREAMING : HS_WAIT_81_01;
    uint8_t nintendo_seq = 1;

    while (1) {
        if ((pid == PID_STEAM_PUCK || pid == PID_STEAM_PUCK_ALT) &&
            ((uint64_t)time(NULL) * 1000u >=
             steam_lizard_keepalive_ms + 3000u)) {
            steam_puck_disable_lizard(fd, slot);
            steam_lizard_keepalive_ms =
                (uint64_t)time(NULL) * 1000u;
        }
        memset(buf,0,64);
        buffers[0]=buf; lengths[0]=64;
        eps[0].ppBuffer=buffers; eps[0].pLength=lengths;
        eps[0].aFrames=0; eps[0].status=0;

        memset(&start,0,sizeof(start)); start.ep_index=0;
        if (ioctl(fd,USB_FS_START,&start)!=0) {
            if (errno==EBUSY){
                memset(&stop,0,sizeof(stop)); stop.ep_index=0; ioctl(fd,USB_FS_STOP,&stop);
                usleep(5000);
            } else if (errno==ENXIO||errno==ENOTTY){
                gp_log("slot[%d] START errno=%d — device gone\n",slot,errno); goto reinit;
            } else {
                gp_log("slot[%d] START fatal errno=%d\n",slot,errno); goto reinit;
            }
            continue;
        }

        int ok=0, cerr=0, cw=0;
        for(cw=0;cw<60;cw++){
            memset(&complete,0,sizeof(complete)); complete.ep_index=0;
            if(ioctl(fd,USB_FS_COMPLETE,&complete)==0){ok=1;break;}
            cerr=errno;
            if(cerr==ENXIO||cerr==ENOTTY){gp_log("slot[%d] COMPLETE errno=%d — gone\n",slot,cerr);goto reinit;}
            if(cerr!=EBUSY) break;
            usleep(500);
        }
        if(!ok){
            memset(&stop,0,sizeof(stop)); stop.ep_index=0; ioctl(fd,USB_FS_STOP,&stop);
            continue;
        }
        if(lengths[0]<1) continue;

        uint32_t len = lengths[0];

        if ((pid == PID_STEAM_PUCK || pid == PID_STEAM_PUCK_ALT) &&
            !steam_first_report_logged && len > 0) {
            gp_log("slot[%d] puck first report=0x%02x len=%u\n",
                   slot, buf[0], (unsigned)len);
            steam_first_report_logged = 1;
        }

        ScePadData pad; memset(&pad,0,sizeof(pad)); pad.quat.w=1.0f;
        int injected = 0;

        if (pid == PID_XBOX) {
            injected = xbox_handle_packet(fd, eps, buf, len, &pad);
        } else if (pid == PID_STEAM ||
                   pid == PID_STEAM_PUCK || pid == PID_STEAM_PUCK_ALT) {
            injected = steam_handle_packet(buf, len, &pad);
        } else {
            injected = nintendo_handle_packet(fd, eps, buf, len, &hs_state, &nintendo_seq, &pad);
        }

        if (injected > 0) {
            if (!usb_ready_notified) {
                notify("Ghostcontrol: slot[%d] streaming — controller active", slot);
                usb_ready_notified = 1;
            }
            /* First real button press confirms the assignment — release the gate
             * so the manager can start the next controller's dialog. */
            if (!g_slots[slot].confirmed && pad.buttons != 0) {
                g_slots[slot].confirmed = 1;
                if (g_assign_slot == slot) g_assign_slot = -1;
                gp_log("slot[%d] assignment confirmed (button press)\n", slot);
            }
            inject_pad(slot, &pad);
        }
    }

reinit:
    if (usb_ready_notified) { notify("Ghostcontrol: slot[%d] controller disconnected", slot); usb_ready_notified=0; }
    memset(&stop,0,sizeof(stop)); stop.ep_index=0; ioctl(fd,USB_FS_STOP,&stop);
    if (out_opened) {
        memset(&fs_close,0,sizeof(fs_close)); fs_close.ep_index=1; ioctl(fd,USB_FS_CLOSE,&fs_close);
        out_opened=0;
    }
    memset(&fs_close,0,sizeof(fs_close)); fs_close.ep_index=0; ioctl(fd,USB_FS_CLOSE,&fs_close);
uninit_exit:
    memset(&uninit,0,sizeof(uninit)); ioctl(fd,USB_FS_UNINIT,&uninit);
    close(fd); fd=-1;

exit_slot:
    gp_log("slot[%d] USB thread exiting — freeing slot\n", slot);
    scePadVirtualDeviceDeleteDevice(g_slots[slot].handle);
    pthread_mutex_lock(&g_slot_lock);
    g_slots[slot].handle    = -1;
    g_slots[slot].vdi_ready = 0;
    g_slots[slot].usb_active= 0;
    g_slots[slot].dev_path[0] = '\0';
    pthread_mutex_unlock(&g_slot_lock);
    return NULL;
}

/* ── Controller manager thread ────────────────────────────────────────── */
static void *controller_manager_thread(void *arg) {
    (void)arg;
    int scan = 0;
    gp_log("Manager thread started (MAX_SLOTS=%d)\n", MAX_SLOTS);

    while (1) {
        for (int i = 0; i < N_UGEN_PATHS; i++) {
            const char *path = UGEN_PATHS[i];

            /* Skip non-external-bus paths */
            if (strncmp(path, "/dev/ugen2.", 11) != 0) continue;

            /* Serialize assignment: if a controller is still awaiting the user's
             * button press to confirm its dialog, do not start another one. */
            if (g_assign_slot >= 0) break;

            /* Skip if already claimed by an active slot */
            int busy = 0;
            pthread_mutex_lock(&g_slot_lock);
            for (int s = 0; s < MAX_SLOTS; s++) {
                if (g_slots[s].usb_active && strcmp(g_slots[s].dev_path, path) == 0) {
                    busy = 1; break;
                }
            }
            pthread_mutex_unlock(&g_slot_lock);
            if (busy) continue;

            /* Try to identify controller */
            uint16_t vid=0, pid=0;
            if (!probe_one_path(path, &vid, &pid)) continue;

            /* Find free slot */
            int slot = -1;
            pthread_mutex_lock(&g_slot_lock);
            for (int s = 0; s < MAX_SLOTS; s++) {
                if (g_slots[s].handle < 0 && !g_slots[s].usb_active) { slot=s; break; }
            }
            pthread_mutex_unlock(&g_slot_lock);

            if (slot < 0) {
                if ((scan % 5) == 0) gp_log("manager: all %d slots full\n", MAX_SLOTS);
                continue;
            }

            const char *name =
                (vid==VID_SWITCH && pid==PID_SWITCH) ? "Nintendo Switch Pro / 8BitDo" :
                (vid==VID_NATIVE && pid==PID_NATIVE) ? "8BitDo Native" :
                (vid==VID_XBOX   && pid==PID_XBOX)   ? "Xbox One S" :
                (vid==VID_STEAM  && pid==PID_STEAM)  ? "Steam Controller 2" :
                (vid==VID_STEAM  && (pid==PID_STEAM_PUCK ||
                                     pid==PID_STEAM_PUCK_ALT)) ? "Steam Controller 2 puck" :
                "Unknown";

            gp_log("manager: %s at %s → slot[%d]\n", name, path, slot);
            notify("Ghostcontrol: %s detected — assign user on screen", name);

            /* Claim the slot path before VDA so manager skips it if we retry.
             * Set the assignment gate — released when user confirms (button press). */
            pthread_mutex_lock(&g_slot_lock);
            strncpy(g_slots[slot].dev_path, path, sizeof(g_slots[slot].dev_path)-1);
            g_slots[slot].usb_active = 1; /* tentatively claimed */
            g_slots[slot].confirmed  = 0;
            g_slots[slot].vid = vid;
            g_slots[slot].pid = pid;
            pthread_mutex_unlock(&g_slot_lock);
            g_assign_slot = slot;

            /* Create VDA and force_bind (shows PS5 assignment dialog) */
            int32_t handle = create_vda_for_slot(slot);
            if (handle < 0) {
                gp_log("manager: slot[%d] VDA failed — releasing\n", slot);
                pthread_mutex_lock(&g_slot_lock);
                g_slots[slot].usb_active = 0;
                g_slots[slot].dev_path[0] = '\0';
                g_slots[slot].handle = -1;
                pthread_mutex_unlock(&g_slot_lock);
                g_assign_slot = -1;
                continue;
            }

            pthread_mutex_lock(&g_slot_lock);
            g_slots[slot].handle      = handle;
            g_slots[slot].vdi_ready   = 1;
            g_slots[slot].inject_count = 0;
            pthread_mutex_unlock(&g_slot_lock);

            /* Launch USB reader thread */
            usb_thread_arg_t *targ = malloc(sizeof(*targ));
            if (!targ) {
                gp_log("manager: malloc fail for slot[%d]\n", slot);
                scePadVirtualDeviceDeleteDevice(handle);
                pthread_mutex_lock(&g_slot_lock);
                g_slots[slot].handle=-1; g_slots[slot].vdi_ready=0;
                g_slots[slot].usb_active=0; g_slots[slot].dev_path[0]='\0';
                pthread_mutex_unlock(&g_slot_lock);
                g_assign_slot = -1;
                continue;
            }
            targ->slot = slot;
            strncpy(targ->dev_path, path, sizeof(targ->dev_path)-1);
            targ->vid=vid; targ->pid=pid;

            pthread_t tid;
            if (pthread_create(&tid, NULL, usb_hid_thread, targ) != 0) {
                gp_log("manager: pthread_create fail slot[%d]\n", slot);
                free(targ);
                scePadVirtualDeviceDeleteDevice(handle);
                pthread_mutex_lock(&g_slot_lock);
                g_slots[slot].handle=-1; g_slots[slot].vdi_ready=0;
                g_slots[slot].usb_active=0; g_slots[slot].dev_path[0]='\0';
                pthread_mutex_unlock(&g_slot_lock);
                g_assign_slot = -1;
            } else {
                pthread_detach(tid);
                gp_log("manager: slot[%d] USB thread started handle=0x%x\n",
                       slot, (uint32_t)handle);
                notify("Ghostcontrol: slot[%d] ready — press a button to assign", slot);
            }
            /* One controller per scan pass — assignment gate blocks the rest
             * until the user confirms this one with a button press. */
            break;
        }

        /* Assignment timeout: if the user never presses a button, release the
         * gate after ~30s so the queue does not stall forever. */
        static int assign_wait = 0;
        if (g_assign_slot >= 0) {
            if (++assign_wait > 15) {  /* 15 * 2s = 30s */
                gp_log("manager: assignment timeout slot[%d] — releasing gate\n", g_assign_slot);
                g_assign_slot = -1;
                assign_wait = 0;
            }
        } else {
            assign_wait = 0;
        }

        scan++;
        if ((scan % 5) == 0) {
            /* Log active slots every 10s */
            int active = 0;
            for (int s = 0; s < MAX_SLOTS; s++)
                if (g_slots[s].usb_active) active++;
            if (active == 0 && (scan % 10) == 0)
                gp_log("manager: scan #%d — no controllers\n", scan);
        }
        usleep(2000000);
    }
    return NULL;
}

/* ── Credential elevation ─────────────────────────────────────────────── */
static void elevate_credentials(void) {
    pid_t p = getpid();
    uint8_t caps[16]; memset(caps,0xff,sizeof(caps));
    kernel_set_ucred_authid(p, 0x3800000000010003l);
    kernel_set_ucred_caps(p, caps);
}

/* ── main ─────────────────────────────────────────────────────────────── */
int main(void) {
    int32_t userId=-1, fgUser=-1; int ret;

    ghostpad_status_log_reset();
    gp_log("Ghost-Control v5 starting — %d slots\n", MAX_SLOTS);
    notify("Ghostcontrol by StonedModder — plug in controllers now");

    /* Kill previous instance */
    { int pfd=open(PID_PATH,O_RDONLY);
      if(pfd>=0){char pb[16]={0};read(pfd,pb,15);close(pfd);
        pid_t old=(pid_t)atoi(pb);
        if(old>0&&old!=getpid()){gp_log("Killing prev pid=%d\n",old);kill(old,SIGTERM);usleep(600000);}
      }
      int pfd2=open(PID_PATH,O_WRONLY|O_CREAT|O_TRUNC,0600);
      if(pfd2>=0){char pb[16];snprintf(pb,sizeof(pb),"%d",getpid());write(pfd2,pb,strlen(pb));close(pfd2);}
    }

    /* Init slots */
    for (int s = 0; s < MAX_SLOTS; s++) {
        g_slots[s].handle     = -1;
        g_slots[s].vdi_ready  = 0;
        g_slots[s].usb_active = 0;
        g_slots[s].confirmed  = 0;
        g_slots[s].dev_path[0]= '\0';
    }
    g_assign_slot = -1;

    sceUserServiceInitialize(NULL);
    sceUserServiceGetInitialUser(&userId);
    sceUserServiceGetForegroundUser(&fgUser);
    gp_log("userId=0x%08x fgUser=0x%08x\n", (uint32_t)userId, (uint32_t)fgUser);
    g_inject_uid = (fgUser > 0) ? fgUser : userId;
    if ((uint32_t)userId<0x10000000u||(uint32_t)userId>0x1000000Fu) userId=0x10000000;
    gp_log("inject_uid=0x%08x\n", (uint32_t)g_inject_uid);

    elevate_credentials();

    ret=scePadInit(); gp_log("scePadInit: 0x%08x\n", ret);
    ret=scePadSetProcessPrivilege(1); gp_log("scePadSetProcessPrivilege: 0x%08x\n", ret);

    /* Clean up any orphaned VDA devices */
    for (int dh=0; dh<64; dh++) {
        if (scePadVirtualDeviceDeleteDevice(dh)==0) gp_log("deleteDevice(%d)\n", dh);
    }

    /* Start klog capture thread first — must be running before any VDA call */
    pthread_t klog_tid;
    if (pthread_create(&klog_tid, NULL, klog_capture_thread, NULL)==0) {
        pthread_detach(klog_tid);
        gp_log("klog thread started\n");
    }
    usleep(300000); /* let klog thread connect before first VDA */

    /* Start controller manager — handles all detection, VDA creation, USB threads */
    pthread_t mgr_tid;
    if (pthread_create(&mgr_tid, NULL, controller_manager_thread, NULL)==0) {
        pthread_detach(mgr_tid);
        gp_log("Manager thread started\n");
    }

    /* Keep-alive */
    uint32_t tick = 0;
    while (1) {
        usleep(1000000);
        tick++;
        if (tick % 10 == 0) {
            int active = 0;
            for (int s = 0; s < MAX_SLOTS; s++) if (g_slots[s].usb_active) active++;
            gp_log("alive tick=%u active_slots=%d\n", tick, active);
        }
    }
    return 0;
}
