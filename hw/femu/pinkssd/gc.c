#include "hw/femu/kvssd/pink/pink_ftl.h"

static void
gc_erase_delay(struct femu_ppa *ppa)
{
    struct nand_cmd gce;
    gce.type = GC_IO;
    gce.cmd = NAND_ERASE;
    gce.stime = 0;
    pink_ssd_advance_status(ppa, &gce);
}

static void
gc_read_delay(struct femu_ppa *ppa)
{
    struct nand_cmd gcr;
    gcr.type = GC_IO;
    gcr.cmd = NAND_READ;
    gcr.stime = 0;
    pink_ssd_advance_status(ppa, &gcr);
}

static void
gc_write_delay(struct femu_ppa *ppa)
{
    struct nand_cmd gcw;
    gcw.type = GC_IO;
    gcw.cmd = NAND_WRITE;
    gcw.stime = 0;
    pink_ssd_advance_status(ppa, &gcw);
}

int
pink_gc_meta_femu(void)
{
    struct line *victim_line = NULL;
    struct ssdparams *spp = &pink_ssd->sp;
    struct femu_ppa ppa;
    int ch, lun, pg_n;

    victim_line = select_victim_meta_line(true);
    if (victim_line == NULL)
        return -1;

    kv_log("%d gc_meta! (line: %d) invalid_pgs / pgs_per_line: %d / %d, vpc: %d \n", ++pink_lsm->header_gc_cnt, victim_line->id, victim_line->ipc, pink_ssd->sp.pgs_per_line, victim_line->vpc);

    ppa.ppa = 0;
    ppa.g.blk = victim_line->id;

    struct nand_page *pg;
    for (ch = 0; ch < spp->nchs; ch++)
    {
        for (lun = 0; lun < spp->luns_per_ch; lun++)
        {
            ppa.g.ch = ch;
            ppa.g.lun = lun;
            ppa.g.pl = 0;
            ppa.g.sec = 0;
            ppa.g.rsv = 0;

            for (pg_n = 0; pg_n < spp->pgs_per_blk; pg_n++)
            {
                ppa.g.pg = pg_n;
                pg = get_pg(&ppa);
                kv_assert(pg->status != PG_FREE);
                if (pg->status == PG_INVALID)
                {
                    pg->status = PG_FREE;
                    continue;
                }
                gc_read_delay(&ppa);

                uint64_t eid = *((uint64_t *)(pg->data + (PAGESIZE - sizeof(uint64_t))));
                pink_level_list_entry *e = pink_lget(eid);
                struct femu_ppa new_ppa = get_new_meta_page();
                struct nand_page *new_pg = get_pg(&new_ppa);
                gc_write_delay(&new_ppa);
                memcpy(new_pg->data, pg->data, PAGESIZE);
                e->ppa = new_ppa;

                mark_page_invalid(&ppa);
                mark_page_valid(&new_ppa);
                ssd_advance_write_pointer(&pink_ssd->lm.meta);

                pink_lput(e);
            }

            gc_erase_delay(&ppa);
            mark_block_free(&ppa);
        }
    }

    mark_line_free(&ppa);

    return 0;
}

static int
binary_search(int level, kv_key k)
{
    int end = pink_lsm->versions.n_files[level];
    int start = 0;
    int mid = 0;
    int res = 0;

    while (start < end)
    {
        mid = (start + end) / 2;

        if (!kv_is_cached(pink_lsm->lsm_cache,
                          pink_lsm->versions.files[level][mid]->cache[LEVEL_LIST_ENTRY]))
        {
            // TODO: Change the fake PPA to real one.
            struct femu_ppa fake_ppa = { .ppa = 0 };
            fake_ppa.g.ch = mid % 8;
            fake_ppa.g.lun = (mid * 3) % 8;
            gc_read_delay(&fake_ppa);
        }

        res = kv_cmp_key(pink_lsm->versions.files[level][mid]->largest, k);

        if (res < 0)
            start = mid + 1;
        else
            end = mid;
    }

    return start;
}

static int
binary_search2(void *data, kv_key k, struct femu_ppa *data_seg_ppa, struct line_age *offset)
{
    int end = *(uint16_t *)data;
    int start = 0;
    int mid = 0;
    int res = 0;

    uint16_t *bitmap = (uint16_t *)data;
    uint16_t *vbitmap = (uint16_t *)(data + (end+2) * sizeof(uint16_t));

    while (start < end)
    {
        mid = (start + end) / 2;

        kv_key k2;
        k2.len = bitmap[mid+2] - bitmap[mid+1] - PPA_LENGTH;
        k2.key = data + bitmap[mid+1] + PPA_LENGTH;

        res = kv_cmp_key(k2, k);

        if (res < 0)
            start = mid + 1;
        else if (res > 0)
            end = mid;
        else
        {
            start = mid;
            break;
        }
    }

    if (start != mid)
    {
        kv_key k2;
        k2.len = bitmap[start+2] - bitmap[start+1] - PPA_LENGTH;
        k2.key = data + bitmap[start+1] + PPA_LENGTH;

        res = kv_cmp_key(k2, k);
    }

    if (res == 0)
    {
        *data_seg_ppa = *((struct femu_ppa *)(data + bitmap[start+1]));
        offset->age = vbitmap[start+1];
        return start;
    }
    else
        return -1;
}

static bool
is_data_valid(kv_key key, struct femu_ppa ppa, int idx)
{
    // pink_lsm->mu protects below memtable lookups.
    kv_snode *target_node = kv_skiplist_find(pink_lsm->mem, key);
    if (target_node) {
        return false;
    }

    target_node = kv_skiplist_find(pink_lsm->imm, key);
    if (target_node) {
        return false;
    }

    target_node = kv_skiplist_find(pink_lsm->key_only_mem, key);
    if (target_node) {
        return false;
    }

    target_node = kv_skiplist_find(pink_lsm->key_only_imm, key);
    if (target_node) {
        return false;
    }

    for (int i = 0; i < LSM_LEVELN; i++)
    {
        if (pink_lsm->versions.n_files[i] == 0)
            continue;

        int idx = binary_search(i, key);

        if (idx == pink_lsm->versions.n_files[i])
            continue;

        pink_level_list_entry *e = pink_lsm->versions.files[i][idx];

        if (kv_cmp_key(e->smallest, key) > 0)
            continue;

        if (!kv_is_cached(pink_lsm->lsm_cache, e->cache[META_SEGMENT]))
            gc_read_delay(&e->ppa);

        struct nand_page *pg = get_pg(&e->ppa);
        struct femu_ppa data_seg_ppa;
        struct line_age offset;

        int i = binary_search2(pg->data, key, &data_seg_ppa, &offset);
        if (i >= 0)
            return data_seg_ppa.ppa == ppa.ppa;
    }

    return false;
}

int gc_erased;
int gc_moved;

static void
gc_data_one_block(struct femu_ppa ppa)
{
    struct ssdparams *spp = &pink_ssd->sp;

    struct nand_page *pg;
    for (int pg_n = 0; pg_n < spp->pgs_per_blk; pg_n++) {
        ppa.g.pg = pg_n;
        pg = get_pg(&ppa);
        gc_read_delay(&ppa);
        kv_assert(pg->status != PG_FREE);

        int nheaders = ((uint16_t *)pg->data)[0];
        for (int i = 0; i < nheaders; i++) {
            kv_key key;
            key.len = ((uint16_t *)pg->data)[(nheaders+2)+i+1];
            key.key = pg->data + ((uint16_t *)pg->data)[i+1];

            bool valid = is_data_valid(key, ppa, i);
            if (valid) {
                gc_moved++;

                key.key = (char*) calloc(key.len+1, sizeof(char));
                memcpy(key.key, pg->data + ((uint16_t *)pg->data)[i+1], key.len);

                kv_value *value = (kv_value*) calloc(1, sizeof(kv_value));
                value->length = ((uint16_t *)pg->data)[i+2] - ((uint16_t *)pg->data)[i+1] - key.len;
                value->value = (char *) calloc(value->length, sizeof(char));
                memcpy(value->value, pg->data + ((uint16_t *)pg->data)[i+1] + key.len, value->length);

                // Give our m allocated key to skiplist.
                // No need to free that.
                //compaction_check(ssd);
                kv_skiplist_insert(pink_lsm->mem, key, value);
            } else {
                gc_erased++;
            }
        }

        pg->status = PG_FREE;
    }
}

int
pink_gc_data_femu(void)
{
    struct line *victim_line = NULL;
    struct ssdparams *spp = &pink_ssd->sp;
    struct femu_ppa ppa;
    //int ch, lun, pg_n;
    int ch, lun;

    victim_line = select_victim_data_line(true);
    if (victim_line == NULL) {
        return -1;
    }
    kv_log("%d gc_data!!!!!!!\n", ++pink_lsm->data_gc_cnt);
    kv_log("vpc: %d, vsc: %d, isc: %d, secs_per_line: %d\n", victim_line->vpc, victim_line->vsc, victim_line->isc, spp->secs_per_line);

    gc_moved = 0;
    gc_erased = 0;

    ppa.g.blk = victim_line->id;
    for (ch = 0; ch < spp->nchs; ch++) {
        for (lun = 0; lun < spp->luns_per_ch; lun++) {
            ppa.g.ch = ch;
            ppa.g.lun = lun;
            ppa.g.pl = 0;
            ppa.g.sec = 0;
            ppa.g.rsv = 0;

            gc_data_one_block(ppa);
            gc_erase_delay(&ppa);
            mark_block_free(&ppa);
        }
    }
    mark_line_free(&ppa);

    kv_log("gc_data_erased: %d, gc_data_moved: %d, moved percentage: %f\n", gc_erased, gc_moved, ((float)gc_moved) / (gc_moved + gc_erased));
    return gc_erased > 0 ? 0 : -2;
}

