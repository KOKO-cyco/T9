#include "tkl_dma2d.h"

#include <string.h>
#include <os/os.h>
#include <driver/hpdma.h>
#include <driver/hal/hal_hpdma_types.h>

extern void bk_printf(const char *fmt, ...);

#define TKL_DMA2D_HPDMA_CHANNEL   HPDMA_ID_3
#define TKL_DMA2D_HPDMA_USER_ID   0x5444
#define TKL_DMA2D_MAX_XSIZE       0xFFFF
#define TKL_DMA2D_MAX_STEP        0x7FFF

typedef struct {
    uint8_t           inited;
    volatile uint8_t  in_use;
    volatile uint8_t  faulted;
    hpdma_id_t        id;
    void             *link_table;
    beken_semaphore_t finish_sem;
    volatile uint32_t generation;
    volatile uint32_t completed;
    TUYA_DMA2D_IRQ_CB cb;
    VOID_T           *arg;
} TKL_DMA2D_CTX_T;

typedef struct {
    uint32_t addr;
    uint32_t xsize;
    uint32_t ysize;
    uint32_t step;
} TKL_DMA2D_PORT_T;

static TKL_DMA2D_CTX_T s_dma2d = {0};

#define TKL_DMA2D_LOG(fmt, ...) bk_printf("[tkl_dma2d] " fmt, ##__VA_ARGS__)

static int __tkl_dma2d_bpp(TUYA_FRAME_FMT_E type)
{
    switch (type) {
    case TUYA_FRAME_FMT_RGB565:
    case TUYA_FRAME_FMT_YUV422:
        return 2;
    case TUYA_FRAME_FMT_RGB888:
        return 3;
    default:
        return 0;
    }
}

static OPERATE_RET __tkl_dma2d_region(const TKL_DMA2D_FRAME_INFO_T *src,
                                      const TKL_DMA2D_FRAME_INFO_T *dst,
                                      uint32_t *w, uint32_t *h)
{
    uint32_t bw = src->width_cp ? src->width_cp : src->width;
    uint32_t bh = src->height_cp ? src->height_cp : src->height;

    if (src->axis.x_axis >= src->width || src->axis.y_axis >= src->height) {
        return OPRT_INVALID_PARM;
    }
    if (dst->axis.x_axis >= dst->width || dst->axis.y_axis >= dst->height) {
        return OPRT_INVALID_PARM;
    }

    if (bw > src->width - src->axis.x_axis) {
        bw = src->width - src->axis.x_axis;
    }
    if (bh > src->height - src->axis.y_axis) {
        bh = src->height - src->axis.y_axis;
    }
    if (bw > dst->width - dst->axis.x_axis) {
        bw = dst->width - dst->axis.x_axis;
    }
    if (bh > dst->height - dst->axis.y_axis) {
        bh = dst->height - dst->axis.y_axis;
    }

    if (bw == 0 || bh == 0) {
        return OPRT_INVALID_PARM;
    }

    *w = bw;
    *h = bh;

    return OPRT_OK;
}

static OPERATE_RET __tkl_dma2d_port(const TKL_DMA2D_FRAME_INFO_T *f, uint32_t bpp,
                                    uint32_t w, uint32_t h, TKL_DMA2D_PORT_T *p)
{
    uint32_t row    = w * bpp;
    uint32_t stride = (uint32_t)(f->width - w) * bpp;

    if (row > TKL_DMA2D_MAX_XSIZE || stride > TKL_DMA2D_MAX_STEP) {
        return OPRT_INVALID_PARM;
    }

    p->addr  = (uint32_t)(uintptr_t)f->pbuf +
               ((uint32_t)f->axis.y_axis * f->width + f->axis.x_axis) * bpp;
    p->xsize = row;
    p->ysize = h;
    p->step  = stride;

    return OPRT_OK;
}

static void __tkl_dma2d_cpu_copy(const TKL_DMA2D_FRAME_INFO_T *src,
                                 const TKL_DMA2D_FRAME_INFO_T *dst,
                                 uint32_t bpp, uint32_t w, uint32_t h)
{
    const uint8_t *s = (const uint8_t *)src->pbuf +
                       ((uint32_t)src->axis.y_axis * src->width + src->axis.x_axis) * bpp;
    uint8_t *d = (uint8_t *)dst->pbuf +
                 ((uint32_t)dst->axis.y_axis * dst->width + dst->axis.x_axis) * bpp;
    uint32_t row = w * bpp;

    for (uint32_t i = 0; i < h; i++) {
        memcpy(d, s, row);
        s += (uint32_t)src->width * bpp;
        d += (uint32_t)dst->width * bpp;
    }
}

static void __tkl_dma2d_finish_isr(hpdma_id_t id, void *user_data)
{
    uint8_t notify = 0;
    uint32_t flags;

    (void)id;
    (void)user_data;

    flags = rtos_enter_critical();
    if (s_dma2d.in_use) {
        s_dma2d.completed = s_dma2d.generation;
        s_dma2d.in_use    = 0;
        notify = 1;
    }
    rtos_exit_critical(flags);

    if (notify) {
        rtos_set_semaphore(&s_dma2d.finish_sem);
        if (s_dma2d.cb) {
            s_dma2d.cb(TUYA_DMA2D_TRANS_COMPLETE_ISR, s_dma2d.arg);
        }
    }
}

static void __tkl_dma2d_err_isr(hpdma_id_t id, void *user_data)
{
    uint8_t notify = 0;
    uint32_t flags;

    (void)id;
    (void)user_data;

    flags = rtos_enter_critical();
    if (s_dma2d.in_use) {
        s_dma2d.in_use  = 0;
        s_dma2d.faulted = 1;
        notify = 1;
    }
    rtos_exit_critical(flags);

    if (notify) {
        TKL_DMA2D_LOG("transfer error, channel %d disabled\r\n", s_dma2d.id);
        rtos_set_semaphore(&s_dma2d.finish_sem);
        if (s_dma2d.cb) {
            s_dma2d.cb(TUYA_DMA2D_TRANS_ERROR_ISR, s_dma2d.arg);
        }
    }
}

static OPERATE_RET __tkl_dma2d_start(const TKL_DMA2D_PORT_T *src, const TKL_DMA2D_PORT_T *dst)
{
    hpdma_link_config_t cfg = {0};
    bk_err_t ret;
    uint32_t flags;

    if (s_dma2d.link_table == NULL || s_dma2d.id >= HPDMA_ID_MAX) {
        return OPRT_COM_ERROR;
    }

    cfg.src_addr          = src->addr;
    cfg.dst_addr          = dst->addr;
    cfg.src_xsize         = (uint16_t)src->xsize;
    cfg.src_ysize         = (uint16_t)src->ysize;
    cfg.dst_xsize         = (uint16_t)dst->xsize;
    cfg.dst_ysize         = (uint16_t)dst->ysize;
    cfg.src_step          = (uint16_t)src->step;
    cfg.dst_step          = (uint16_t)dst->step;
    cfg.finish_int_en     = 1;
    cfg.half_finish_int_en = 0;

    ret = bk_hpdma_link_set_desc(s_dma2d.link_table, 0, &cfg);
    if (ret != BK_OK) {
        return OPRT_COM_ERROR;
    }

    ret = bk_hpdma_register_isr(s_dma2d.id, NULL, NULL, __tkl_dma2d_finish_isr, NULL);
    if (ret == BK_OK) {
        ret = bk_hpdma_register_bus_err_isr(s_dma2d.id, __tkl_dma2d_err_isr, NULL);
    }
    if (ret == BK_OK) {
        ret = bk_hpdma_enable_finish_interrupt(s_dma2d.id);
    }
    if (ret != BK_OK) {
        return OPRT_COM_ERROR;
    }

    flags = rtos_enter_critical();
    s_dma2d.generation++;
    if (s_dma2d.generation == 0) {
        s_dma2d.generation++;
    }
    s_dma2d.completed = 0;
    s_dma2d.in_use    = 1;
    rtos_exit_critical(flags);

    ret = bk_hpdma_link_transfer(s_dma2d.id, s_dma2d.link_table);
    if (ret != BK_OK) {
        flags = rtos_enter_critical();
        s_dma2d.in_use = 0;
        s_dma2d.generation++;
        rtos_exit_critical(flags);
        bk_hpdma_disable_finish_interrupt(s_dma2d.id);
        return OPRT_COM_ERROR;
    }

    return OPRT_OK;
}

OPERATE_RET tkl_dma2d_init(CONST TUYA_DMA2D_BASE_CFG_T *cfg)
{
    hpdma_id_t id;
    void *table;
    bk_err_t ret;

    if (s_dma2d.inited) {
        if (cfg && cfg->cb) {
            s_dma2d.cb  = cfg->cb;
            s_dma2d.arg = cfg->arg;
        }
        return OPRT_OK;
    }

    ret = bk_hpdma_driver_init();
    if (ret != BK_OK) {
        TKL_DMA2D_LOG("hpdma driver init failed, ret=%d\r\n", ret);
        return OPRT_COM_ERROR;
    }

    id = bk_fixed_hpdma_alloc(TKL_DMA2D_HPDMA_USER_ID, TKL_DMA2D_HPDMA_CHANNEL);
    if (id >= HPDMA_ID_MAX) {
        id = bk_hpdma_alloc(TKL_DMA2D_HPDMA_USER_ID);
    }
    if (id >= HPDMA_ID_MAX) {
        TKL_DMA2D_LOG("no free hpdma channel\r\n");
        return OPRT_COM_ERROR;
    }

    table = bk_hpdma_link_init(1);
    if (table == NULL) {
        TKL_DMA2D_LOG("link table alloc failed\r\n");
        bk_hpdma_free(TKL_DMA2D_HPDMA_USER_ID, id);
        return OPRT_COM_ERROR;
    }

    ret = rtos_init_semaphore_ex(&s_dma2d.finish_sem, 1, 0);
    if (ret != BK_OK) {
        TKL_DMA2D_LOG("semaphore init failed, ret=%d\r\n", ret);
        bk_hpdma_link_deinit(table);
        bk_hpdma_free(TKL_DMA2D_HPDMA_USER_ID, id);
        return OPRT_COM_ERROR;
    }

    s_dma2d.id         = id;
    s_dma2d.link_table = table;
    s_dma2d.generation = 0;
    s_dma2d.completed  = 0;
    s_dma2d.in_use     = 0;
    s_dma2d.faulted    = 0;
    s_dma2d.cb         = cfg ? cfg->cb : NULL;
    s_dma2d.arg        = cfg ? cfg->arg : NULL;

    ret = bk_hpdma_register_isr(id, NULL, NULL, __tkl_dma2d_finish_isr, NULL);
    if (ret == BK_OK) {
        ret = bk_hpdma_register_bus_err_isr(id, __tkl_dma2d_err_isr, NULL);
    }
    if (ret != BK_OK) {
        TKL_DMA2D_LOG("isr register failed, ret=%d\r\n", ret);
        rtos_deinit_semaphore(&s_dma2d.finish_sem);
        bk_hpdma_link_deinit(table);
        bk_hpdma_free(TKL_DMA2D_HPDMA_USER_ID, id);
        memset(&s_dma2d, 0, sizeof(s_dma2d));
        return OPRT_COM_ERROR;
    }

    s_dma2d.inited = 1;

    TKL_DMA2D_LOG("init ok, channel %d\r\n", id);

    return OPRT_OK;
}

OPERATE_RET tkl_dma2d_deinit(void)
{
    if (!s_dma2d.inited) {
        return OPRT_OK;
    }

    bk_hpdma_stop(s_dma2d.id);
    bk_hpdma_wait_to_idle(s_dma2d.id);
    bk_hpdma_disable_finish_interrupt(s_dma2d.id);

    if (s_dma2d.link_table) {
        bk_hpdma_link_deinit(s_dma2d.link_table);
    }

    if (bk_hpdma_free(TKL_DMA2D_HPDMA_USER_ID, s_dma2d.id) != BK_OK) {
        bk_hpdma_force_reclaim(TKL_DMA2D_HPDMA_USER_ID, s_dma2d.id);
    }

    rtos_deinit_semaphore(&s_dma2d.finish_sem);
    memset(&s_dma2d, 0, sizeof(s_dma2d));

    return OPRT_OK;
}

OPERATE_RET tkl_dma2d_memcpy(TKL_DMA2D_FRAME_INFO_T *src, TKL_DMA2D_FRAME_INFO_T *dst)
{
    TKL_DMA2D_PORT_T sp = {0};
    TKL_DMA2D_PORT_T dp = {0};
    uint32_t w = 0;
    uint32_t h = 0;
    int bpp;

    if (src == NULL || dst == NULL || src->pbuf == NULL || dst->pbuf == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (src->type != dst->type) {
        return tkl_dma2d_convert(src, dst);
    }

    bpp = __tkl_dma2d_bpp(src->type);
    if (bpp == 0) {
        return OPRT_NOT_SUPPORTED;
    }

    if (__tkl_dma2d_region(src, dst, &w, &h) != OPRT_OK) {
        return OPRT_INVALID_PARM;
    }

    if (!s_dma2d.inited || s_dma2d.faulted || s_dma2d.in_use) {
        __tkl_dma2d_cpu_copy(src, dst, (uint32_t)bpp, w, h);
        return OPRT_COM_ERROR;
    }

    if (__tkl_dma2d_port(src, (uint32_t)bpp, w, h, &sp) != OPRT_OK ||
        __tkl_dma2d_port(dst, (uint32_t)bpp, w, h, &dp) != OPRT_OK) {
        __tkl_dma2d_cpu_copy(src, dst, (uint32_t)bpp, w, h);
        return OPRT_COM_ERROR;
    }

    if (__tkl_dma2d_start(&sp, &dp) != OPRT_OK) {
        __tkl_dma2d_cpu_copy(src, dst, (uint32_t)bpp, w, h);
        return OPRT_COM_ERROR;
    }

    return OPRT_OK;
}

OPERATE_RET tkl_dma2d_convert(TKL_DMA2D_FRAME_INFO_T *src, TKL_DMA2D_FRAME_INFO_T *dst)
{
    if (src == NULL || dst == NULL) {
        return OPRT_INVALID_PARM;
    }

    if (src->type == dst->type) {
        return tkl_dma2d_memcpy(src, dst);
    }

    return OPRT_NOT_SUPPORTED;
}
