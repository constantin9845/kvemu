#include "hw/femu/kvssd/lksv/lksv3_ftl.h"
#include <execinfo.h>
#include "hw/femu/kvssd/lksv/skiplist.h"

static void *ftl_thread(void *arg);

bool lksv3_should_meta_gc_high(int margin)
{
    return lksv_ssd->lm.meta.free_line_cnt <= 1 + margin;
}

bool lksv3_should_data_gc_high(int margin)
{
    return lksv_ssd->lm.data.free_line_cnt <= 1 + margin;
}

static inline int victim_line_cmp_vsc(pqueue_pri_t next, pqueue_pri_t curr)
{
    return (next > curr);
}

static inline pqueue_pri_t victim_line_get_vsc(void *a)
{
    return ((struct line *)a)->vsc;
}

static inline void victim_line_set_vsc(void *a, pqueue_pri_t pri)
{
    ((struct line *)a)->vsc = pri;
}

static inline int victim_line_cmp_pri(pqueue_pri_t next, pqueue_pri_t curr)
{
    return (next > curr);
}

static inline pqueue_pri_t victim_line_get_pri(void *a)
{
    return ((struct line *)a)->vpc;
}

static inline void victim_line_set_pri(void *a, pqueue_pri_t pri)
{
    ((struct line *)a)->vpc = pri;
}

static inline size_t victim_line_get_pos(void *a)
{
    return ((struct line *)a)->pos;
}

static inline void victim_line_set_pos(void *a, size_t pos)
{
    ((struct line *)a)->pos = pos;
}

static void _ssd_init_write_pointer(struct line_partition *lm)
{
    struct write_pointer *wpp = &lm->wp;
    struct line *curline = NULL;

    curline = QTAILQ_FIRST(&lm->free_line_list);
    QTAILQ_REMOVE(&lm->free_line_list, curline, entry);
    lm->free_line_cnt--;

    // wpp->curline is always our next-to-write super-block 
    wpp->curline = curline;
    wpp->ch = 0;
    wpp->lun = 0;
    wpp->pg = 0;
    wpp->blk = wpp->curline->id;
    wpp->pl = 0;
}

static void ssd_init_write_pointer(void)
{
    _ssd_init_write_pointer(&lksv_ssd->lm.meta);
    _ssd_init_write_pointer(&lksv_ssd->lm.data);
}

static void ssd_init_lines(void)
{
    struct ssdparams *spp = &lksv_ssd->sp;
    struct line_mgmt *lm = &lksv_ssd->lm;
    struct line *line;

    lm->tt_lines = spp->blks_per_pl;
    kv_assert(lm->tt_lines == spp->tt_lines);

    lm->meta.lines = spp->meta_lines;
    lm->data.lines = spp->data_lines;
    kv_assert(lm->meta.lines + lm->data.lines <= lm->tt_lines);

    lm->lines = (struct line*) calloc(lm->tt_lines, sizeof(struct line));

    QTAILQ_INIT(&lm->meta.free_line_list);
    lm->meta.victim_line_pq = pqueue_init(spp->meta_lines, victim_line_cmp_pri, victim_line_get_pri, victim_line_set_pri, victim_line_get_pos, victim_line_set_pos);
    QTAILQ_INIT(&lm->meta.full_line_list);

    lm->meta.free_line_cnt = 0;
    for (int i = 0; i < lm->meta.lines; i++) {
        line = &lm->lines[i];
        line->id = i;
        line->ipc = 0;
        line->vpc = 0;
        line->isc = 0;
        line->vsc = 0;
        line->pos = 0;
        line->meta = true;
        /* initialize all the lines as free lines */
        QTAILQ_INSERT_TAIL(&lm->meta.free_line_list, line, entry);
        lm->meta.free_line_cnt++;
    }

    kv_assert(lm->meta.free_line_cnt == lm->meta.lines);
    lm->meta.victim_line_cnt = 0;
    lm->meta.full_line_cnt = 0;
    lm->meta.age = 0;

    QTAILQ_INIT(&lm->data.free_line_list);
    lm->data.victim_line_pq = pqueue_init(spp->data_lines, victim_line_cmp_pri, victim_line_get_pri, victim_line_set_pri, victim_line_get_pos, victim_line_set_pos);
    QTAILQ_INIT(&lm->data.full_line_list);

    lm->data.free_line_cnt = 0;
    for (int i = lm->meta.lines; i < lm->meta.lines + lm->data.lines; i++) {
        line = &lm->lines[i];
        line->id = i;
        line->ipc = 0;
        line->vpc = 0;
        line->pos = 0;
        /* initialize all the lines as free lines */
        QTAILQ_INSERT_TAIL(&lm->data.free_line_list, line, entry);
        lm->data.free_line_cnt++;
    }

    kv_assert(lm->data.free_line_cnt == lm->data.lines);
    lm->data.victim_line_cnt = 0;
    lm->data.full_line_cnt = 0;
    lm->data.age = 0;
}

static inline void check_addr(int a, int max)
{
    kv_assert(a >= 0 && a < max);
}

struct line *lksv3_get_next_free_line(struct line_partition *lm)
{
    struct line *curline = NULL;

    curline = QTAILQ_FIRST(&lm->free_line_list);
    if (!curline) {
        printf("No free lines left!!!!\n");
        return NULL;
    }

    QTAILQ_REMOVE(&lm->free_line_list, curline, entry);
    lm->free_line_cnt--;
    curline->age = lm->age++;
    return curline;
}

void lksv3_ssd_advance_write_pointer(struct line_partition *lm)
{
    struct ssdparams *spp = &lksv_ssd->sp;
    struct write_pointer *wpp = &lm->wp;

    check_addr(wpp->ch, spp->nchs);
    wpp->ch++;
    if (wpp->ch == spp->nchs) {
        wpp->ch = 0;
        check_addr(wpp->lun, spp->luns_per_ch);
        wpp->lun++;
        /* in this case, we should go to next lun */
        if (wpp->lun == spp->luns_per_ch) {
            wpp->lun = 0;
            /* go to next page in the block */
            check_addr(wpp->pg, spp->pgs_per_blk);
            wpp->pg++;
            if (wpp->pg == spp->pgs_per_blk) {
                wpp->pg = 0;
                /* move current line to {victim,full} line list */
                if (lm == &lksv_ssd->lm.meta) {
                    // We don't use vsc, isc in meta lines.
                    kv_assert(wpp->curline->vsc == 0);
                    kv_assert(wpp->curline->isc == 0);
                    if (wpp->curline->vpc == spp->pgs_per_line) {
                        // All pgs are still valid, move to full line list
                        kv_assert(wpp->curline->ipc == 0);
                        QTAILQ_INSERT_TAIL(&lm->full_line_list, wpp->curline, entry);
                        lm->full_line_cnt++;
                    } else {
                        // There must be some invalid pages in this line.
                        kv_assert(wpp->curline->ipc > 0);
                        pqueue_insert(lm->victim_line_pq, wpp->curline);
                        lm->victim_line_cnt++;
                    }
                } else {
                    lm->victim_line_cnt++;
                }
                /* current line is used up, pick another empty line */
                check_addr(wpp->blk, spp->blks_per_pl);
                wpp->curline = NULL;
                wpp->curline = lksv3_get_next_free_line(lm);
                if (!wpp->curline) {
                    /* TODO */
                    abort();
                }
                wpp->blk = wpp->curline->id;
                check_addr(wpp->blk, spp->blks_per_pl);
                /* make sure we are starting from page 0 in the super block */
                kv_assert(wpp->pg == 0);
                kv_assert(wpp->lun == 0);
                kv_assert(wpp->ch == 0);
                /* TODO: assume # of pl_per_lun is 1, fix later */
                kv_assert(wpp->pl == 0);
                kv_assert(wpp->curline->vpc == 0);
                kv_assert(wpp->curline->ipc == 0);
            }
        }
    }
}

static struct femu_ppa get_new_page(struct write_pointer *wpp)
{
    struct femu_ppa ppa;
    ppa.ppa = 0;
    ppa.g.ch = wpp->ch;
    ppa.g.lun = wpp->lun;
    ppa.g.pg = wpp->pg;
    ppa.g.blk = wpp->blk;
    ppa.g.pl = wpp->pl;
    kv_assert(ppa.g.pl == 0);

    return ppa;
}

struct femu_ppa lksv3_get_new_meta_page(void)
{
    return get_new_page(&lksv_ssd->lm.meta.wp);
}

struct femu_ppa lksv3_get_new_data_page(void)
{
    return get_new_page(&lksv_ssd->lm.data.wp);
}

static void ssd_init_params(struct ssdparams *spp)
{
#ifdef GB_96
    /* 96 GB address */
    spp->secsz = 512;
    spp->secs_per_pg = 16;
    spp->pgs_per_blk = 384;
    spp->blks_per_pl = 512;
    spp->pls_per_lun = 1;
    spp->luns_per_ch = 8;
    spp->nchs = 8;
#else

    /* 64 GB address */
    spp->secsz = 512;
#if defined(PAGE4)
    spp->secs_per_pg = 8;
    spp->pgs_per_blk = 512;
#elif defined(PAGE16)
    spp->secs_per_pg = 32;
    spp->pgs_per_blk = 128;
#else
    spp->secs_per_pg = 16;
    spp->pgs_per_blk = 256;
#endif
    spp->blks_per_pl = 512;
    spp->pls_per_lun = 1;
    spp->luns_per_ch = 8;
    spp->nchs = 8;
#endif

    /* calculated values */
    spp->secs_per_blk = spp->secs_per_pg * spp->pgs_per_blk;
    spp->secs_per_pl = spp->secs_per_blk * spp->blks_per_pl;
    spp->secs_per_lun = spp->secs_per_pl * spp->pls_per_lun;
    spp->secs_per_ch = spp->secs_per_lun * spp->luns_per_ch;
    spp->tt_secs = spp->secs_per_ch * spp->nchs;

    spp->pgs_per_pl = spp->pgs_per_blk * spp->blks_per_pl;
    spp->pgs_per_lun = spp->pgs_per_pl * spp->pls_per_lun;
    spp->pgs_per_ch = spp->pgs_per_lun * spp->luns_per_ch;
    spp->tt_pgs = spp->pgs_per_ch * spp->nchs;

    spp->blks_per_lun = spp->blks_per_pl * spp->pls_per_lun;
    spp->blks_per_ch = spp->blks_per_lun * spp->luns_per_ch;
    spp->tt_blks = spp->blks_per_ch * spp->nchs;

    spp->pls_per_ch =  spp->pls_per_lun * spp->luns_per_ch;
    spp->tt_pls = spp->pls_per_ch * spp->nchs;

    spp->tt_luns = spp->luns_per_ch * spp->nchs;

    /* line is special, put it at the end */
    spp->blks_per_line = spp->tt_luns; /* TODO: to fix under multiplanes */
    spp->pgs_per_line = spp->blks_per_line * spp->pgs_per_blk;
    spp->secs_per_line = spp->pgs_per_line * spp->secs_per_pg;
    spp->tt_lines = spp->blks_per_lun; /* TODO: to fix under multiplanes */

    kv_assert(PG_N % spp->nchs == 0);
    kv_assert(spp->pgs_per_line % PG_N == 0);

    // Half of the 10% over-provisioned space is allocated for value-log.
    spp->meta_lines = spp->tt_lines * 95 / 100;
    spp->data_lines = spp->tt_lines - spp->meta_lines;
    kv_log("initial meta_lines: %d\n", spp->meta_lines);

    spp->gc_thres_pcent = 0.75;
    spp->gc_thres_lines = (int)((1 - spp->gc_thres_pcent) * spp->tt_lines);
    spp->meta_gc_thres_lines = (int)((1 - spp->gc_thres_pcent) * spp->meta_lines);
    spp->gc_thres_lines = (int)((1 - spp->gc_thres_pcent) * spp->tt_lines);
    spp->data_gc_thres_lines = (int)((1 - spp->gc_thres_pcent) * spp->data_lines);

    spp->gc_thres_pcent_high = 0.95;
    spp->gc_thres_lines_high = (int)((1 - spp->gc_thres_pcent_high) * spp->tt_lines);
    spp->meta_gc_thres_lines_high = (int)((1 - spp->gc_thres_pcent_high) * spp->meta_lines);
    spp->data_gc_thres_lines_high = (int)((1 - spp->gc_thres_pcent_high) * spp->data_lines);
    spp->enable_gc_delay = true;

    spp->enable_comp_delay = true;
}

static void ssd_init_nand_page(struct nand_page *pg, struct ssdparams *spp)
{
    pg->nsecs = spp->secs_per_pg;
    pg->sec = (nand_sec_status_t*) malloc(sizeof(nand_sec_status_t) * pg->nsecs);
    for (int i = 0; i < pg->nsecs; i++) {
        pg->sec[i] = SEC_FREE;
    }
    pg->status = PG_FREE;
    pg->data = calloc(1, PAGESIZE);
}

static void ssd_init_nand_blk(struct nand_block *blk, struct ssdparams *spp)
{
    blk->npgs = spp->pgs_per_blk;
    blk->pg = g_malloc0(sizeof(struct nand_page) * blk->npgs);
    for (int i = 0; i < blk->npgs; i++) {
        ssd_init_nand_page(&blk->pg[i], spp);
    }
    blk->ipc = 0;
    blk->vpc = 0;
    blk->erase_cnt = 0;
    blk->wp = 0;
}

static void ssd_init_nand_plane(struct nand_plane *pl, struct ssdparams *spp)
{
    pl->nblks = spp->blks_per_pl;
    pl->blk = g_malloc0(sizeof(struct nand_block) * pl->nblks);
    for (int i = 0; i < pl->nblks; i++) {
        ssd_init_nand_blk(&pl->blk[i], spp);
    }
}

static void ssd_init_nand_lun(struct nand_lun *lun, struct ssdparams *spp)
{
    lun->npls = spp->pls_per_lun;
    lun->pl = g_malloc0(sizeof(struct nand_plane) * lun->npls);
    for (int i = 0; i < lun->npls; i++) {
        ssd_init_nand_plane(&lun->pl[i], spp);
    }
    lun->next_lun_avail_time = 0;
    lun->busy = false;
}

static void ssd_init_ch(struct ssd_channel *ch, struct ssdparams *spp)
{
    ch->nluns = spp->luns_per_ch;
    ch->lun = g_malloc0(sizeof(struct nand_lun) * ch->nluns);
    for (int i = 0; i < ch->nluns; i++) {
        ssd_init_nand_lun(&ch->lun[i], spp);
    }
    ch->next_ch_avail_time = 0;
    ch->busy = 0;
}

static void lksv_lsm_setup_cache_size(struct kv_lsm_options *lopts)
{
    uint64_t ssd_capacity = (unsigned long long) lksv_ssd->sp.tt_pgs * PAGESIZE;
    // 10% of SSD capacity is assigned to OP.
    uint64_t ssd_capacity_exclude_op = ssd_capacity * 90 / 100;
    lopts->cache_memory_size = ssd_capacity_exclude_op / 1024;   // 0.10%.
    kv_log("Set cache memory to %lu MB\n", lopts->cache_memory_size / 1024 / 1024);
}

void lksv3ssd_init(FemuCtrl *n) {
    struct ssdparams *spp = &lksv_ssd->sp;
    struct kv_lsm_options *lopts;

#ifdef OURS
    printf("This is ours\n");
#else
    printf("This is ours+\n");
#endif

    kv_assert(lksv_ssd);

    ssd_init_params(spp);

    /* initialize ssd internal layout architecture */
    lksv_ssd->ch = g_malloc0(sizeof(struct ssd_channel) * spp->nchs);
    for (int i = 0; i < spp->nchs; i++) {
        ssd_init_ch(&lksv_ssd->ch[i], spp);
    }

    /* initialize all the lines */
    ssd_init_lines();

    /* initialize write pointer, this is how we allocate new pages for writes */
    ssd_init_write_pointer();

    lopts = kv_lsm_default_opts();
    // set custom options
    lksv_lsm_setup_cache_size(lopts);
    /////////////////////
    kv_lsm_setup_db(&lksv_ssd->lops, LKSV);

    lksv_ssd->lops->open(lopts);

    qemu_thread_create(&lksv_ssd->ftl_thread, "FEMU-FTL-Thread", ftl_thread, n, QEMU_THREAD_JOINABLE);
    lksv_ssd->do_reset = true;
}

static inline struct ssd_channel *get_ch(struct femu_ppa *ppa)
{
    return &(lksv_ssd->ch[ppa->g.ch]);
}

inline struct nand_lun *lksv3_get_lun(struct femu_ppa *ppa)
{
    struct ssd_channel *ch = get_ch(ppa);
    return &(ch->lun[ppa->g.lun]);
}

static inline struct nand_plane *get_pl(struct femu_ppa *ppa)
{
    struct nand_lun *lun = lksv3_get_lun(ppa);
    return &(lun->pl[ppa->g.pl]);
}

static inline struct nand_block *get_blk(struct femu_ppa *ppa)
{
    struct nand_plane *pl = get_pl(ppa);
    return &(pl->blk[ppa->g.blk]);
}

inline struct line *lksv3_get_line(struct femu_ppa *ppa)
{
    return &(lksv_ssd->lm.lines[ppa->g.blk]);
}

inline struct nand_page *lksv3_get_pg(struct femu_ppa *ppa)
{
    struct nand_block *blk = get_blk(ppa);
    return &(blk->pg[ppa->g.pg]);
}

static bool is_meta_page(struct femu_ppa *ppa)
{
    return lksv_ssd->lm.lines[ppa->g.blk].meta;
}

uint64_t lksv3_ssd_advance_status(struct femu_ppa *ppa, struct nand_cmd *ncmd)
{
    if (!lksv_ssd->start_log) {
        return 0;
    }

    int c = ncmd->cmd;
    uint64_t cmd_stime = (ncmd->stime == 0) ? \
        qemu_clock_get_ns(QEMU_CLOCK_REALTIME) : ncmd->stime;
    uint64_t nand_stime;
    struct nand_lun *lun = lksv3_get_lun(ppa);
    uint64_t lat = 0;

    if (ncmd->type == USER_IO) {
        if (c == NAND_READ) {
            qatomic_inc(&lksv_ssd->n->host_read_cnt);
        } else if (c == NAND_WRITE) {
            qatomic_inc(&lksv_ssd->n->host_write_cnt);
        }
    } else if (ncmd->type == COMP_IO) {
        if (c == NAND_READ) {
            qatomic_inc(&lksv_ssd->n->comp_read_cnt);
        } else if (c == NAND_WRITE) {
            qatomic_inc(&lksv_ssd->n->comp_write_cnt);
        }
    } else if (ncmd->type == GC_IO) {
        if (c == NAND_READ) {
            qatomic_inc(&lksv_ssd->n->gc_read_cnt);
        } else if (c == NAND_WRITE) {
            qatomic_inc(&lksv_ssd->n->gc_write_cnt);
        }
    }

    uint8_t page_type = kvssd_get_page_type(&lksv_ssd->lat, ppa->g.pg);
    switch (c) {
    case NAND_READ:
        /* read: perform NAND cmd first */
        nand_stime = (lun->next_lun_avail_time < cmd_stime) ? cmd_stime : \
                     lun->next_lun_avail_time;
        lun->next_lun_avail_time = nand_stime + kvssd_get_page_read_latency(&lksv_ssd->lat, page_type);
        lat = lun->next_lun_avail_time - cmd_stime;
#if 0
        lun->next_lun_avail_time = nand_stime + spp->pg_rd_lat;

        /* read: then data transfer through channel */
        chnl_stime = (ch->next_ch_avail_time < lun->next_lun_avail_time) ? \
            lun->next_lun_avail_time : ch->next_ch_avail_time;
        ch->next_ch_avail_time = chnl_stime + spp->ch_xfer_lat;

        lat = ch->next_ch_avail_time - cmd_stime;
#endif
        break;

    case NAND_WRITE:
        /* write: transfer data through channel first */
        nand_stime = (lun->next_lun_avail_time < cmd_stime) ? cmd_stime : \
                     lun->next_lun_avail_time;
        if (ncmd->type == USER_IO) {
            lun->next_lun_avail_time = nand_stime + kvssd_get_page_write_latency(&lksv_ssd->lat, page_type);
        } else {
            lun->next_lun_avail_time = nand_stime + kvssd_get_page_write_latency(&lksv_ssd->lat, page_type);
        }
        lat = lun->next_lun_avail_time - cmd_stime;

#if 0
        chnl_stime = (ch->next_ch_avail_time < cmd_stime) ? cmd_stime : \
                     ch->next_ch_avail_time;
        ch->next_ch_avail_time = chnl_stime + spp->ch_xfer_lat;

        /* write: then do NAND program */
        nand_stime = (lun->next_lun_avail_time < ch->next_ch_avail_time) ? \
            ch->next_ch_avail_time : lun->next_lun_avail_time;
        lun->next_lun_avail_time = nand_stime + spp->pg_wr_lat;

        lat = lun->next_lun_avail_time - cmd_stime;
#endif
        break;

    case NAND_ERASE:
        /* erase: only need to advance NAND status */
        nand_stime = (lun->next_lun_avail_time < cmd_stime) ? cmd_stime : \
                     lun->next_lun_avail_time;
        lun->next_lun_avail_time = nand_stime + kvssd_get_blk_erase_latency(&lksv_ssd->lat);

        lat = lun->next_lun_avail_time - cmd_stime;
        break;

    default:
        kv_err("Unsupported NAND command: 0x%x\n", c);
    }

    return lat;
}

/* update SSD status about one page from PG_VALID -> PG_VALID */
void lksv3_mark_page_invalid(struct femu_ppa *ppa)
{
    struct line_partition *lm;
    struct ssdparams *spp = &lksv_ssd->sp;
    struct nand_block *blk = NULL;
    struct nand_page *pg = NULL;
    bool was_full_line = false;
    struct line *line;

    if (is_meta_page(ppa)) {
        lm = &lksv_ssd->lm.meta;
    } else {
        lm = &lksv_ssd->lm.data;
    }

    /* update corresponding page status */
    pg = lksv3_get_pg(ppa);
    kv_assert(pg->status == PG_VALID);
    pg->status = PG_INVALID;
    if (pg->data) {
        //memset(pg->data, 0, PAGESIZE);
    }

    /* update corresponding block status */
    blk = get_blk(ppa);
    kv_assert(blk->ipc >= 0 && blk->ipc < spp->pgs_per_blk);
    blk->ipc++;
    kv_assert(blk->vpc > 0 && blk->vpc <= spp->pgs_per_blk);
    blk->vpc--;

    /* update corresponding line status */
    line = lksv3_get_line(ppa);
    kv_assert(line->ipc >= 0 && line->ipc < spp->pgs_per_line);
    if (line->vpc == spp->pgs_per_line) {
        kv_assert(line->ipc == 0);
        was_full_line = true;
    }
    line->ipc++;
    kv_assert(line->vpc > 0 && line->vpc <= spp->pgs_per_line);

    /* meta only. fix me. */
    kv_assert(is_meta_page(ppa));
    /* Adjust the position of the victime line in the pq under over-writes */
    if (line->pos) {
        /* Note that line->vpc will be updated by this call */
        pqueue_change_priority(lm->victim_line_pq, line->vpc - 1, line);
    } else {
        line->vpc--;
    }

    if (was_full_line) {
        /* move line: "full" -> "victim" */
        QTAILQ_REMOVE(&lm->full_line_list, line, entry);
        lm->full_line_cnt--;
        pqueue_insert(lm->victim_line_pq, line);
        lm->victim_line_cnt++;
    }
}

void lksv3_mark_page_valid(struct femu_ppa *ppa)
{
    struct nand_block *blk = NULL;
    struct nand_page *pg = NULL;
    struct line *line;

    /* update page status */
    pg = lksv3_get_pg(ppa);
    kv_assert(pg->status == PG_FREE);
    pg->status = PG_VALID;

    /* update corresponding block status */
    blk = get_blk(ppa);
    kv_assert(blk->vpc >= 0 && blk->vpc < lksv_ssd->sp.pgs_per_blk);
    blk->vpc++;

    /* update corresponding line status */
    line = lksv3_get_line(ppa);
    kv_assert(line->vpc >= 0 && line->vpc < lksv_ssd->sp.pgs_per_line);
    line->vpc++;
}

void lksv3_mark_block_free(struct femu_ppa *ppa)
{
    struct ssdparams *spp = &lksv_ssd->sp;
    struct nand_block *blk = get_blk(ppa);
    struct nand_page *pg = NULL;

    for (int i = 0; i < spp->pgs_per_blk; i++) {
        /* reset page status */
        pg = &blk->pg[i];
        kv_assert(pg->nsecs == spp->secs_per_pg);
        pg->status = PG_FREE;
    }

    /* reset block status */
    kv_assert(blk->npgs == spp->pgs_per_blk);
    blk->ipc = 0;
    blk->vpc = 0;
    blk->erase_cnt++;
}

static struct line *select_victim_line(struct line_partition *lm, bool force, bool meta)
{
    struct line *victim_line = NULL;

    victim_line = pqueue_peek(lm->victim_line_pq);
    if (!victim_line && meta) {
        return NULL;
    }

    if (meta) {
        if (!force && victim_line->ipc < lksv_ssd->sp.pgs_per_line / 8) {
            return NULL;
        }
    } else {
        abort();
    }

    pqueue_pop(lm->victim_line_pq);
    victim_line->pos = 0;
    lm->victim_line_cnt--;

    /* victim_line is a danggling node now */
    return victim_line;
}

struct line *lksv3_select_victim_meta_line(bool force)
{
    return select_victim_line(&lksv_ssd->lm.meta, force, true);
}

struct line *lksv3_select_victim_data_line(bool force)
{
    return select_victim_line(&lksv_ssd->lm.data, force, false);
}

void lksv3_mark_line_free(struct femu_ppa *ppa)
{
    struct line_partition *lm;
    struct line *line;

    if (is_meta_page(ppa)) {
        lm = &lksv_ssd->lm.meta;
    } else {
        lm = &lksv_ssd->lm.data;
    }

    line = lksv3_get_line(ppa);

    line->ipc = 0;
    line->vpc = 0;
    line->isc = 0;
    line->vsc = 0;
    if (line->vsc != 0) {
        printf("missing kv pairs: %d\n", line->vsc);
        abort();
    }
    line->vsc = 0;
    line->isc = 0;
    /* move this line to free line list */
    QTAILQ_INSERT_TAIL(&lm->free_line_list, line, entry);
    lm->free_line_cnt++;
}

static int
make_room_for_write(void)
{
    while (true)
    {
        if (kv_skiplist_approximate_memory_usage(lksv_lsm->mem) < WRITE_BUFFER_SIZE)
            break;

        if (lksv_lsm->imm)
        {
            qemu_mutex_unlock(&lksv_lsm->mu);
            usleep(1000);
            qemu_mutex_lock(&lksv_lsm->mu);
            continue;
        }

        lksv_lsm->imm = lksv_lsm->mem;
        lksv_lsm->mem = kv_skiplist_init();

        lksv_maybe_schedule_compaction();
    }

    return 0;
}

static kv_value*
find_from_list(kv_key key, kv_skiplist *skl, NvmeRequest *req)
{
    struct nand_page *pg;
    lksv_kv_descriptor d;
    kv_value *v;
    kv_snode *n;

    if (!skl)
        return NULL;

    n = kv_skiplist_find(skl, key);
    if (!n)
        return NULL;

    v = (kv_value *) malloc(sizeof(kv_value));
    if (n->private)
    {
        d.ppa = *snode_ppa(n); 
        d.value_log_offset = *snode_off(n);
        d.hash = *snode_hash(n);

        lksv_user_read_delay(&d.ppa, req);

        pg = lksv3_get_pg(&d.ppa);
        int offset = PAGESIZE -
                     LKSV3_SSTABLE_FOOTER_BLK_SIZE -
                     (LKSV3_SSTABLE_META_BLK_SIZE * (d.value_log_offset + 1));
        lksv_block_meta meta = *(lksv_block_meta *) (pg->data + offset);
        kv_assert(meta.g1.hash == d.hash);
        v->length = meta.g2.slen;
        v->value = (char *) malloc(v->length);
        memcpy(v->value, pg->data + meta.g1.off + meta.g1.klen, v->length);
    }
    else
    {
        v->length = n->value->length;
        v->value = (char *) malloc(v->length);
        memcpy(v->value, n->value->value, v->length);
    }

    return v;
}

static uint64_t ssd_retrieve(NvmeRequest *req)
{
    kv_skiplist *mem, *imm, *key_only_mem, *key_only_imm;
    kv_key k;
    kv_value *v;

    qemu_mutex_lock(&lksv_lsm->mu);

    mem = lksv_lsm->mem;
    kv_skiplist_get(mem);

    imm = lksv_lsm->imm;
    kv_skiplist_get(imm);

    key_only_mem = lksv_lsm->key_only_mem;
    kv_skiplist_get(key_only_mem);

    key_only_imm = lksv_lsm->key_only_imm;
    kv_skiplist_get(key_only_imm);

    qemu_mutex_unlock(&lksv_lsm->mu);

    req->value = NULL;
    req->etime = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);

    k.key = (char *) req->key_buf;
    k.len = req->key_length;

    v = find_from_list(k, mem, req);
    if (v) {
        req->value_length = v->length;
        req->value = (uint8_t *) v->value;
        FREE(v);

        kv_skiplist_put(mem);
        kv_skiplist_put(imm);
        kv_skiplist_put(key_only_mem);
        kv_skiplist_put(key_only_imm);
        return req->etime - req->stime;
    }
    kv_skiplist_put(mem);

    v = find_from_list(k, imm, req);
    if (v) {
        req->value_length = v->length;
        req->value = (uint8_t *) v->value;
        FREE(v);

        kv_skiplist_put(imm);
        kv_skiplist_put(key_only_mem);
        kv_skiplist_put(key_only_imm);
        return req->etime - req->stime;
    }
    kv_skiplist_put(imm);

    v = find_from_list(k, key_only_mem, req);
    if (v) {
        req->value_length = v->length;
        req->value = (uint8_t *) v->value;
        FREE(v);

        kv_skiplist_put(key_only_mem);
        kv_skiplist_put(key_only_imm);
        return req->etime - req->stime;
    }
    kv_skiplist_put(key_only_mem);

    v = find_from_list(k, key_only_imm, req);
    if (v) {
        req->value_length = v->length;
        req->value = (uint8_t *) v->value;
        FREE(v);

        kv_skiplist_put(key_only_imm);
        return req->etime - req->stime;
    }
    kv_skiplist_put(key_only_imm);

    qemu_mutex_lock(&lksv_lsm->mu);
    v = lksv_get(k, req);
    qemu_mutex_unlock(&lksv_lsm->mu);
    if (v) {
        req->value_length = v->length;
        req->value = (uint8_t *) v->value;
        FREE(v);

        return req->etime - req->stime;
    }

    abort();

    return req->etime - req->stime;
}

static uint64_t ssd_read(NvmeRequest *req)
{
    return 0;
}

static uint64_t ssd_store(NvmeRequest *req)
{
    kv_key k;
    kv_value *v;

    k.key = g_malloc0(req->key_length);
    memcpy(k.key, req->key_buf, req->key_length);
    k.len = req->key_length;
    kv_assert(k.len >= KV_MIN_KEY_LEN && k.len <= KV_MAX_KEY_LEN);

    v = g_malloc0(sizeof(kv_value));
    v->length = req->value_length;
    v->value = (char *) req->value;

    qemu_mutex_lock(&lksv_lsm->mu);

    if (make_room_for_write() != 0)
    {
        return -1;
    }
    lksv3_skiplist_insert(lksv_lsm->mem, k, v, true);

    qemu_mutex_unlock(&lksv_lsm->mu);

    return 0;
}

static uint64_t ssd_write(NvmeRequest *req)
{
    return 0;
}

static uint64_t ssd_delete(NvmeRequest *req)
{
    return 0;
}

static void reset_delay(void)
{
    struct ssdparams *spp = &lksv_ssd->sp;
    struct femu_ppa ppa;
    uint64_t now;

    now = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);
    ppa.ppa = 0;
    for (int ch = 0; ch < spp->nchs; ch++) {
        for (int lun = 0; lun < spp->luns_per_ch; lun++) {
            ppa.g.ch = ch;
            ppa.g.lun = lun;
            struct nand_lun *lun = lksv3_get_lun(&ppa);

            lun->next_lun_avail_time = now;
        }
    }
}

static void *ftl_thread(void *arg)
{
    FemuCtrl *n = (FemuCtrl *)arg;
    NvmeRequest *req = NULL;
    uint64_t lat = 0;
    int rc;
    int i;

    while (!*(lksv_ssd->dataplane_started_ptr)) {
        usleep(100000);
    }

    /* FIXME: not safe, to handle ->to_ftl and ->to_poller gracefully */
    lksv_ssd->to_ftl = n->to_ftl;
    lksv_ssd->to_poller = n->to_poller;

    lksv_ssd->start_log = false;
    while (1) {
        for (i = 1; i <= n->num_poller; i++) {
            if (!lksv_ssd->to_ftl[i] || !femu_ring_count(lksv_ssd->to_ftl[i]))
                continue;

            rc = femu_ring_dequeue(lksv_ssd->to_ftl[i], (void *)&req, 1);
            if (rc != 1) {
                printf("FEMU: FTL to_ftl dequeue failed\n");
            }

            kv_assert(req);
            // FIXME: cmd.opcode and cmd_opcode; this should be merged
            switch (req->cmd_opcode) {
            case NVME_CMD_KV_STORE:
                lat = ssd_store(req);
                break;
            case NVME_CMD_KV_RETRIEVE:
                if (lksv_ssd->start_log == false) {
                    kv_debug("Reset latency timer!\n");
                    lksv_ssd->start_log = true;
                    for (int ch = 0; ch < lksv_ssd->sp.nchs; ch++) {
                        for (int lun = 0; lun < lksv_ssd->sp.luns_per_ch; lun++) {
                            lksv_ssd->ch[ch].lun[lun].next_lun_avail_time = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);
                        }
                    }
                }
                req->flash_access_count = 0;
                lat = ssd_retrieve(req);
                break;
            case NVME_CMD_KV_DELETE:
                if (lksv_ssd->do_reset) {
                    reset_delay();
                    lksv_ssd->do_reset = false;
                }
                lat = ssd_delete(req);
                break;
            case NVME_CMD_KV_ITERATE_REQUEST:
            case NVME_CMD_KV_ITERATE_READ:
            case NVME_CMD_KV_DUMP:
                lat = 1;
                break;
            default:
                switch (req->cmd.opcode) {
                    case NVME_CMD_WRITE:
                        lat = ssd_write(req);
                        break;
                    case NVME_CMD_READ:
                        lat = ssd_read(req);
                        break;
                    case NVME_CMD_DSM:
                        lat = 0;
                        break;
                    default:
                        //kv_err("FTL received unkown request type, ERROR\n");
                        ;
                }
            }

            req->reqlat = lat;
            req->expire_time += lat;

            rc = femu_ring_enqueue(lksv_ssd->to_poller[i], (void *)&req, 1);
            if (rc != 1) {
                kv_err("FTL to_poller enqueue failed\n");
            }
        }
    }

    return NULL;
}

