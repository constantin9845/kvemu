#include "hw/femu/kvssd/lksv/lksv3_ftl.h"
#include "hw/femu/kvssd/lksv/skiplist.h"

static void
next_page(lksv_data_seg_writer *w)
{
    // TODO: find more proper ways for gc.
    if (lksv3_should_meta_gc_high(0))
        lksv_gc_meta_erase_only();

    // TODO: sharding
    const int margin = 4 * PAGESIZE;

    w->left = (PG_N * PAGESIZE) -
              (PG_N * LKSV3_SSTABLE_FOOTER_BLK_SIZE) -
              margin;

    w->writing_ppa = lksv3_get_new_meta_page();
    kv_assert(is_pivot_ppa(w->writing_ppa));

    w->n = 0;
}

static int
compare_key(const void *a, const void *b)
{
    return kv_cmp_key(
               (*(lksv_level_list_entry **) a)->smallest,
               (*(lksv_level_list_entry **) b)->smallest
           );
}

static void
flush_page(lksv_data_seg_writer *w)
{
    int i;

    lksv_level_list_entry *e = w->results[(* w->result_n)] = lksv_lnew();
    e->level = w->level;
    kv_copy_key(&e->smallest, &w->buffer[0]->key);
    kv_copy_key(&e->largest, &w->buffer[w->n-1]->key);
    e->ppa = w->writing_ppa;

    lksv_bucket_sort(w->buffer, w->n);

    bool ref_line_ids[1024];
    memset(ref_line_ids, 0, 1024);

    int flushed_n = 0;
    for (i = 0; i < PG_N; i++)
    {
        int left = PAGESIZE - LKSV3_SSTABLE_FOOTER_BLK_SIZE;
        struct nand_page *pg = lksv3_get_pg(&w->writing_ppa);

        lksv_block_footer footer;
        footer.f = 0;
        footer.level_list_entry_id = e->id;

        int wp = 0;

        while (flushed_n < w->n)
        {
            int write_size = w->buffer[flushed_n]->key.len +
                             w->buffer[flushed_n]->value.length +
                             LKSV3_SSTABLE_META_BLK_SIZE;

            if (left < write_size)
                break;

            left -= write_size;

            footer.g.n++;

            lksv_block_meta meta;
            meta.m1 = meta.m2 = 0;
            meta.g1.off = wp;
            meta.g1.klen = w->buffer[flushed_n]->key.len;
            meta.g1.hash = w->buffer[flushed_n]->hash;
            meta.g2.snum = 1;
            meta.g2.slen = w->buffer[flushed_n]->value.length;
            meta.g2.voff = w->buffer[flushed_n]->value_log_offset;
            meta.g2.rsv = w->buffer[flushed_n]->str_order;
            // TODO: fix this.
            if (meta.g2.slen == PPA_LENGTH)
                meta.g1.flag = KEY_PPA_PAIR;
            else
                meta.g1.flag = KEY_VALUE_PAIR;
            memcpy(pg->data + (
                        PAGESIZE -
                        LKSV3_SSTABLE_FOOTER_BLK_SIZE -
                        (LKSV3_SSTABLE_META_BLK_SIZE * footer.g.n)
                   ), &meta, sizeof(lksv_block_meta));

            memcpy(pg->data + wp, w->buffer[flushed_n]->key.key, meta.g1.klen);
            wp += meta.g1.klen;

            if (meta.g1.flag == KEY_PPA_PAIR)
            {
                int line_id = w->buffer[flushed_n]->ppa.g.blk;
                struct line *line = &lksv_ssd->lm.lines[line_id];

                if (!ref_line_ids[line_id])
                {
                    ref_line_ids[line_id] = true;

                    g_hash_table_insert(per_line_data(line)->files[e->level], &e->id, e);
                    per_line_data(line)->referenced_by_files++;

                    e->value_log_rmap[w->buffer[flushed_n]->ppa.g.blk] = true;
                }
                memcpy(pg->data + wp, &w->buffer[flushed_n]->ppa, meta.g2.slen);
            }
            else
                memcpy(pg->data + wp, w->buffer[flushed_n]->value.value, meta.g2.slen);
            wp += meta.g2.slen;

            flushed_n++;
        }

        if (footer.g.n > 0)
        {
            e->hash_lists[i].hashes = malloc(footer.g.n * sizeof(uint32_t));

            int k = flushed_n - footer.g.n;
            for (int j = 0; j < footer.g.n; j++)
                e->hash_lists[i].hashes[j] = w->buffer[k+j]->hash;
            e->hash_lists[i].n = footer.g.n;

            e->pg_start_hashes[i] = w->buffer[k]->hash >> LEVELLIST_HASH_SHIFTS;

            e->hash_list_n++;
        }

        memcpy(pg->data + (PAGESIZE - LKSV3_SSTABLE_FOOTER_BLK_SIZE),
               &footer, sizeof(lksv_block_footer));

        lksv3_mark_page_valid(&w->writing_ppa);
        if (footer.g.n == 0)
            lksv3_mark_page_invalid(&w->writing_ppa);

        lksv3_ssd_advance_write_pointer(&lksv_ssd->lm.meta);
        lksv_comp_write_delay(&w->writing_ppa);
        w->writing_ppa = lksv3_get_new_meta_page();
    }

    kv_assert(flushed_n == w->n);

    for (i = 0; i < w->n; i++)
        free(w->buffer[i]);

    (* w->result_n)++;

    const int margin = 32;
    uint32_t entry_size = e->smallest.len +
                          e->largest.len +
                          (LEVELLIST_HASH_BYTES * PG_N) +
                          margin;
    kv_cache_insert(lksv_lsm->lsm_cache,
                    &e->cache[LEVEL_LIST_ENTRY],
                    entry_size,
                    cache_level(LEVEL_LIST_ENTRY, e->level),
                    KV_CACHE_WITHOUT_FLAGS);

    if (e->level < LSM_LEVELN - 1)
    {
        entry_size = (w->n * HASH_BYTES) + 20;
        kv_cache_insert(lksv_lsm->lsm_cache,
                        &e->cache[HASH_LIST],
                        entry_size,
                        cache_level(HASH_LIST, e->level),
                        KV_CACHE_WITHOUT_FLAGS);
    }
}

static lksv_data_seg_writer *
lksv_new_data_seg_writer(lksv_level_list_entry **results, int *result_n, int level)
{
    lksv_data_seg_writer *w = calloc(1, sizeof(lksv_data_seg_writer));

    w->results = results;
    w->result_n = result_n;
    w->level = level;

    next_page(w);

    return w;
}

static void
lksv_set_data_seg_writer(lksv_data_seg_writer *w, lksv_kv_descriptor *d)
{
    int write_size = d->key.len +
                     d->value.length +
                     LKSV3_SSTABLE_META_BLK_SIZE;

    if (w->left < write_size || w->n == 8192)
    {
        flush_page(w);
        next_page(w);
    }
    w->left -= write_size;

    w->buffer[w->n] = d;
    d->str_order = w->n;
    w->n++;
}

static void
lksv_close_data_seg_writer(lksv_data_seg_writer *w)
{
    if (w->n > 0)
        flush_page(w);

    free(w);
}

static kv_snode *
skl_iter_next(kv_skiplist *skl, kv_snode *n)
{
    if (n->list[1] == skl->header)
        return NULL;

    return n->list[1];
}

static void
mark_compacting_meta_lines(lksv_file_iterator *iter)
{
    struct femu_ppa ppa;
    int i = iter->upper ? 0 : 1;

    if (iter->current_file > 0)
    {
        ppa = iter->files[iter->current_file-1]->ppa;
        kv_assert(lksv_lsm->compacting_meta_lines[i][ppa.g.blk]);
        lksv_lsm->compacting_meta_lines[i][ppa.g.blk] = false;
    }

    if (iter->current_file < iter->files_n)
    {
        ppa = iter->files[iter->current_file]->ppa;
        kv_assert(!lksv_lsm->compacting_meta_lines[i][ppa.g.blk]);
        lksv_lsm->compacting_meta_lines[i][ppa.g.blk] = true;
    }
}

static void
next_data_seg_group(lksv_file_iterator *iter)
{
    iter->buffer_n = iter->current_buffer = 0;

    lksv_level_list_entry *e = iter->files[iter->current_file];

    bool bottom_level = (iter->level == LSM_LEVELN - 2);

#ifndef OURS
    lksv_value_log_writer *log_w = NULL;

    if (iter->reinsert_values)
        log_w = lksv_new_value_log_writer();
#endif

    for (int i = 0; i < e->hash_list_n; i++)
    {
        struct femu_ppa ppa = get_next_write_ppa(e->ppa, i);
        struct nand_page *pg = lksv3_get_pg(&ppa);

        lksv_comp_read_delay(&ppa);

        lksv_block_footer footer;
        footer = *(lksv_block_footer *) (pg->data + (PAGESIZE - LKSV3_SSTABLE_FOOTER_BLK_SIZE));

        for (int j = 0; j < footer.g.n; j++)
        {
            lksv_block_meta *meta;
            meta = (lksv_block_meta *) (pg->data + (PAGESIZE - LKSV3_SSTABLE_FOOTER_BLK_SIZE - (LKSV3_SSTABLE_META_BLK_SIZE * (j + 1))));

            lksv_kv_descriptor *desc = calloc(1, sizeof(lksv_kv_descriptor));

            desc->key.key = pg->data + meta->g1.off;
            desc->key.len = meta->g1.klen;
            desc->value.value = pg->data + meta->g1.off + meta->g1.klen;
            desc->value.length = meta->g2.slen;
            desc->value_log_offset = meta->g2.voff;
            desc->hash = meta->g1.hash;
            if (meta->g1.flag == KEY_VALUE_PAIR)
                desc->ppa.ppa = UNMAPPED_PPA;
            else
            {
                desc->ppa = *(struct femu_ppa *) desc->value.value;
                if (bottom_level || (iter->fetch_values && desc->ppa.g.blk == iter->fetch_line_id))
                {
                    lksv_comp_read_delay(&desc->ppa);

                    struct nand_page *value_log = lksv3_get_pg(&desc->ppa);
                    int offset = PAGESIZE -
                                 LKSV3_SSTABLE_FOOTER_BLK_SIZE -
                                 (LKSV3_SSTABLE_META_BLK_SIZE * (meta->g2.voff + 1));
                    lksv_block_meta meta2 = *(lksv_block_meta *) (value_log->data + offset);
                    kv_assert(meta->g1.hash == meta2.g1.hash);
                    desc->value.length = meta2.g2.slen;
                    desc->value.value = value_log->data + meta2.g1.off + meta2.g1.klen;

                    desc->ppa.ppa = UNMAPPED_PPA;
                    desc->value_log_offset = 0;

#ifndef OURS
                    if (!bottom_level && iter->reinsert_values && lksv_ssd->lm.data.free_line_cnt > 0 && desc->value.length > 512)
                    {
                        lksv_set_value_log_writer(log_w, desc);
                        desc->value.length = PPA_LENGTH;
                        desc->value.value = NULL;
                    }
#endif
                }
            }
            desc->str_order = meta->g2.rsv;

            iter->buffer[desc->str_order] = desc;
        }

        iter->buffer_n += footer.g.n;
    }

#ifndef OURS
    if (iter->reinsert_values)
        lksv_close_value_log_writer(log_w);
#endif

    iter->current_file++;
}

static void
free_buffer(lksv_file_iterator *iter)
{
    for (int i = 0; i < iter->buffer_n; i++)
        free(iter->buffer[i]);
}

static lksv_kv_descriptor *
file_iter_next(lksv_file_iterator *iter)
{
retry:
    if (iter->current_buffer < iter->buffer_n)
        return iter->buffer[iter->current_buffer++];

    free_buffer(iter);

    mark_compacting_meta_lines(iter);

    if (iter->files_n == iter->current_file)
        return NULL;

    next_data_seg_group(iter);
    goto retry;
}

void
lksv_write_level0_table(kv_skiplist *mem)
{
    kv_key smallest, largest;
    lksv_file_iterator file_iter;
    lksv_kv_descriptor *desc;

    kv_skiplist_get_start_end_key(mem, &smallest, &largest);

    file_iter.upper = false;
    file_iter.fetch_values = false;
    file_iter.fetch_line_id = -1;
    file_iter.level = 0;
#ifndef OURS
    file_iter.reinsert_values = true;
#endif
    file_iter.files = lksv_overlaps(0, smallest, largest, &file_iter.files_n);
    file_iter.current_file = 0;
    file_iter.buffer_n = 0;
    file_iter.current_buffer = 0;
    desc = file_iter_next(&file_iter);

    kv_snode *snode = mem->header->list[1];

    lksv_level_list_entry *results[512];
    int                   result_n = 0;
    lksv_data_seg_writer *w = lksv_new_data_seg_writer(results, &result_n, 0);

    while (true)
    {
        if (!snode && !desc)
            break;

        int res;
        if (snode && desc)
            res = kv_cmp_key(desc->key, snode->key);
        else if (!snode)
            res = -1;
        else
            res = +1;

        lksv_kv_descriptor *target = malloc(sizeof(lksv_kv_descriptor));

        if (res < 0)
        {
            *target = *desc;

            lksv_set_data_seg_writer(w, target);

            desc = file_iter_next(&file_iter);
        }
        else
        {
            target->key = snode->key;
            target->value = *snode->value;
            target->hash = *snode_hash(snode);
            target->ppa = *snode_ppa(snode);
            target->value_log_offset = *snode_off(snode);

            per_line_data(
                    &lksv_ssd->lm.lines[target->ppa.g.blk]
            )->referenced_by_memtable--;

            lksv_set_data_seg_writer(w, target);

            snode = skl_iter_next(mem, snode);
            if (res == 0)
                desc = file_iter_next(&file_iter);
        }

        kv_assert(result_n < 512);
    }
    
    lksv_close_data_seg_writer(w);

    int new_files_n = lksv_lsm->versions.n_files[0] - file_iter.files_n + result_n;
    lksv_level_list_entry **new_files = malloc(new_files_n * sizeof(lksv_level_list_entry *));

    int i, j, k;
    i = j = k = 0;
    while (k < lksv_lsm->versions.n_files[0])
    {
        if (file_iter.files_n > 0 &&
            lksv_lsm->versions.files[0][k]->id == file_iter.files[0]->id)
            break;

        new_files[i] = lksv_lsm->versions.files[0][k];
        i++;
        k++;
    }
    while (j < result_n)
    {
        new_files[i] = results[j];
        i++;
        j++;
    }
    k += file_iter.files_n;
    while (k < lksv_lsm->versions.n_files[0])
    {
        new_files[i] = lksv_lsm->versions.files[0][k];
        i++;
        k++;
    }
    kv_assert(i == new_files_n);

    qsort(new_files, new_files_n, sizeof(lksv_level_list_entry *), compare_key);

    if (lksv_lsm->versions.files[0])
        FREE(lksv_lsm->versions.files[0]);
    lksv_lsm->versions.files[0] = new_files;
    lksv_lsm->versions.n_files[0] = new_files_n;

    for (int i = 0; i < file_iter.files_n; i++)
        lksv_lput(file_iter.files[i]);
    
    free(file_iter.files);
}

void
lksv_write_level123_table(lksv_compaction *c)
{
    lksv_file_iterator upper_iter;
    lksv_file_iterator lower_iter;
    lksv_kv_descriptor *upper, *lower;

    upper_iter.upper = true;
    upper_iter.fetch_values = c->log_triggered;
    upper_iter.fetch_line_id = c->log_triggered_line_id;
    upper_iter.level = c->level;
#ifndef OURS
    upper_iter.reinsert_values = (c->level < LSM_LEVELN - 2);
#endif
    upper_iter.files = c->inputs[0];
    upper_iter.files_n = c->input_n[0];
    upper_iter.current_file = 0;
    upper_iter.buffer_n = 0;
    upper_iter.current_buffer = 0;
    upper = file_iter_next(&upper_iter);

    lower_iter.upper = false;
    lower_iter.fetch_values = (c->log_triggered || c->level == LSM_LEVELN-2);
    lower_iter.fetch_line_id = c->log_triggered_line_id;
    lower_iter.level = c->level;
#ifndef OURS
    lower_iter.reinsert_values = (c->level < LSM_LEVELN - 2);
#endif
    lower_iter.files = c->inputs[1];
    lower_iter.files_n = c->input_n[1];
    lower_iter.current_file = 0;
    lower_iter.buffer_n = 0;
    lower_iter.current_buffer = 0;
    lower = file_iter_next(&lower_iter);

    lksv_level_list_entry **results;
    int                   result_n = 0;
    const int             alloc_margin = 2048;

    results = calloc(c->input_n[0] + c->input_n[1] + alloc_margin, sizeof(lksv_level_list_entry **));
    lksv_data_seg_writer *w = lksv_new_data_seg_writer(results, &result_n, c->level+1);

    while (true)
    {
        if (!upper && !lower)
            break;

        int res;
        if (upper && lower)
            res = kv_cmp_key(lower->key, upper->key);
        else if (!upper)
            res = -1;
        else
            res = +1;

        lksv_kv_descriptor *target = malloc(sizeof(lksv_kv_descriptor));

        if (res < 0)
        {
            *target = *lower;

            lksv_set_data_seg_writer(w, target);

            lower = file_iter_next(&lower_iter);
        }
        else
        {
            *target = *upper;

            lksv_set_data_seg_writer(w, target);

            upper = file_iter_next(&upper_iter);
            if (res == 0)
                lower = file_iter_next(&lower_iter);
        }

        kv_assert(result_n < c->input_n[0] + c->input_n[1] + alloc_margin);
    }
    
    lksv_close_data_seg_writer(w);

    int new_files_n = lksv_lsm->versions.n_files[c->level+1] - c->input_n[1] + result_n;
    lksv_level_list_entry **new_files = malloc(new_files_n * sizeof(lksv_level_list_entry *));

    int i, j, k;
    i = j = k = 0;
    while (k < lksv_lsm->versions.n_files[c->level+1])
    {
        if (lower_iter.files_n > 0 && 
            lksv_lsm->versions.files[c->level+1][k]->id == lower_iter.files[0]->id)
            break;

        new_files[i] = lksv_lsm->versions.files[c->level+1][k];
        i++;
        k++;
    }
    while (j < result_n)
    {
        new_files[i] = results[j];
        i++;
        j++;
    }
    k += lower_iter.files_n;
    while (k < lksv_lsm->versions.n_files[c->level+1])
    {
        new_files[i] = lksv_lsm->versions.files[c->level+1][k];
        i++;
        k++;
    }
    kv_assert(i == new_files_n);

    qsort(new_files, new_files_n, sizeof(lksv_level_list_entry *), compare_key);

    if (lksv_lsm->versions.files[c->level+1])
        FREE(lksv_lsm->versions.files[c->level+1]);
    lksv_lsm->versions.files[c->level+1] = new_files;
    lksv_lsm->versions.n_files[c->level+1] = new_files_n;

    int old_files_n = lksv_lsm->versions.n_files[c->level] - c->input_n[0];
    lksv_level_list_entry **old_files = malloc(old_files_n * sizeof(lksv_level_list_entry *));

    i = j = k = 0;
    while (k < lksv_lsm->versions.n_files[c->level])
    {
        if (lksv_lsm->versions.files[c->level][k]->id == upper_iter.files[0]->id)
            break;

        old_files[i] = lksv_lsm->versions.files[c->level][k];
        i++;
        k++;
    }
    k += upper_iter.files_n;
    while (k < lksv_lsm->versions.n_files[c->level])
    {
        old_files[i] = lksv_lsm->versions.files[c->level][k];
        i++;
        k++;
    }

    if (lksv_lsm->versions.files[c->level])
        FREE(lksv_lsm->versions.files[c->level]);
    lksv_lsm->versions.files[c->level] = old_files;
    lksv_lsm->versions.n_files[c->level] = old_files_n;

    for (int i = 0; i < upper_iter.files_n; i++)
        lksv_lput(upper_iter.files[i]);
    for (int i = 0; i < lower_iter.files_n; i++)
        lksv_lput(lower_iter.files[i]);

    free(results);
}

kv_value *
internal_get(int eid, kv_key k1, uint32_t hash, NvmeRequest *req)
{
    kv_value *v = NULL;
    lksv_level_list_entry *e = lksv_lget(eid);

    int i;
    uint16_t shifted_hash = hash >> LEVELLIST_HASH_SHIFTS;
    for (i = e->hash_list_n-1; i > 0; i--)
    {
        if (e->pg_start_hashes[i] <= shifted_hash)
            break;
    }

    struct femu_ppa ppa;
    struct nand_page *pg;

retry:

    ppa = get_next_write_ppa(e->ppa, i);
    pg = lksv3_get_pg(&ppa);
    lksv_user_read_delay(&ppa, req);

    lksv_block_footer footer;
    footer = *(lksv_block_footer *) (pg->data + (PAGESIZE - LKSV3_SSTABLE_FOOTER_BLK_SIZE));

    for (int j = 0; j < footer.g.n; j++)
    {
        lksv_block_meta *meta;
        meta = (lksv_block_meta *) (pg->data + (PAGESIZE - LKSV3_SSTABLE_FOOTER_BLK_SIZE - (LKSV3_SSTABLE_META_BLK_SIZE * (j + 1))));

        if (meta->g1.hash < hash)
            continue;
        if (meta->g1.hash > hash)
            break;

        kv_key k2;
        k2.key = pg->data + meta->g1.off;
        k2.len = meta->g1.klen;
        if (kv_cmp_key(k1, k2) == 0)
        {
            v = malloc(sizeof(kv_value));
            if (meta->g1.flag == KEY_VALUE_PAIR)
            {
                v->length = meta->g2.slen;
                v->value = malloc(v->length);
                memcpy(v->value, pg->data + meta->g1.off + meta->g1.klen, v->length);
            }
            else if (meta->g1.flag == KEY_PPA_PAIR)
            {
                struct femu_ppa value_log_ppa = *(struct femu_ppa *) (pg->data + meta->g1.off + meta->g1.klen);
                struct nand_page *value_log = lksv3_get_pg(&value_log_ppa);
                int offset = PAGESIZE -
                             LKSV3_SSTABLE_FOOTER_BLK_SIZE -
                             (LKSV3_SSTABLE_META_BLK_SIZE * (meta->g2.voff + 1));
                lksv_block_meta meta2 = *(lksv_block_meta *) (value_log->data + offset);
                kv_assert(meta->g1.hash == meta2.g1.hash);
                v->length = meta2.g2.slen;
                v->value = malloc(v->length);
                memcpy(v->value, value_log->data + meta2.g1.off + meta2.g1.klen, v->length);

                lksv_user_read_delay(&value_log_ppa, req);
            }
            break;
        }
    }

    if (!v && i > 0 && e->hash_lists[i].hashes[0] >= hash)
    {
        i--;
        goto retry;
    }

    lksv_lput(e);

    return v;
}

