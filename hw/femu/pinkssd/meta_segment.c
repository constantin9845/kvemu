#include "hw/femu/kvssd/pink/pink_ftl.h"
#include "hw/femu/kvssd/pink/skiplist.h"

static void
next_page(pink_meta_seg_writer *w)
{
    // TODO: find more proper ways for gc.
    if (pink_should_meta_gc_high(0))
        pink_gc_meta_femu();

    w->left = PAGESIZE - (4 * sizeof(uint16_t)) - sizeof(uint64_t);
    w->writing_ppa = get_new_meta_page();
    w->n = 0;
}

static void
flush_page(pink_meta_seg_writer *w)
{
    pink_level_list_entry *e = w->results[(* w->result_n)] = pink_lnew();
    kv_copy_key(&e->smallest, &w->buffer[0]->key);
    kv_copy_key(&e->largest, &w->buffer[w->n-1]->key);
    e->ppa = w->writing_ppa;

    struct nand_page *pg = get_pg(&w->writing_ppa);
    char *ptr = pg->data;
    uint16_t *bitmap = (uint16_t *)ptr;
    uint16_t *vbitmap = (uint16_t *)(ptr + ((w->n+2) * sizeof(uint16_t)));
    uint16_t data_start = 2 * (w->n+2) * sizeof(uint16_t);

    for (int i = 0; i < w->n; i++)
    {
        memcpy(&ptr[data_start], &w->buffer[i]->ppa, sizeof(struct femu_ppa));
        memcpy(&ptr[data_start+sizeof(struct femu_ppa)], w->buffer[i]->key.key, w->buffer[i]->key.len);
        bitmap[i+1] = data_start;

        struct line_age age;
        age.g.in_page_idx = w->buffer[i]->data_seg_offset.g.in_page_idx;
        age.g.line_age = (get_line(&w->buffer[i]->ppa)->age % LINE_AGE_MAX);
        vbitmap[i+1] = age.age;

        data_start += w->buffer[i]->key.len + sizeof(struct femu_ppa);
    }

    bitmap[0] = w->n;
    vbitmap[0] = w->n;
    bitmap[w->n+1] = data_start;
    vbitmap[w->n+1] = -1;

    // for gc.
    memcpy(pg->data + (PAGESIZE - sizeof(uint64_t)), &e->id, sizeof(uint64_t));

    mark_page_valid(&w->writing_ppa);
    ssd_advance_write_pointer(&pink_ssd->lm.meta);
    pink_comp_write_delay(&w->writing_ppa);

    for (int i = 0; i < w->n; i++)
        free(w->buffer[i]);

    (* w->result_n)++;

    const int margin = 8;
    uint32_t entry_size = e->smallest.len +
                          PPA_LENGTH +
                          margin;
    kv_cache_insert(pink_lsm->lsm_cache,
                    &e->cache[LEVEL_LIST_ENTRY],
                    entry_size,
                    cache_level(LEVEL_LIST_ENTRY, w->level),
                    KV_CACHE_WITHOUT_FLAGS);

    if (w->level < LSM_LEVELN - 1)
    {
        entry_size = PAGESIZE;
        kv_cache_insert(pink_lsm->lsm_cache,
                        &e->cache[META_SEGMENT],
                        entry_size,
                        cache_level(META_SEGMENT, w->level),
                        KV_CACHE_WITHOUT_FLAGS);
    }
}

static pink_meta_seg_writer *
pink_new_meta_seg_writer(pink_level_list_entry **results, int *result_n, int level)
{
    pink_meta_seg_writer *w = calloc(1, sizeof(pink_meta_seg_writer));

    w->results = results;
    w->result_n = result_n;
    w->level = level;

    next_page(w);

    return w;
}

static void
pink_set_meta_seg_writer(pink_meta_seg_writer *w, pink_kv_descriptor *d)
{
    int write_size = d->key.len +
                     PPA_LENGTH +
                     (2 * sizeof(uint16_t));

    if (w->left < write_size || w->n == 8192)
    {
        flush_page(w);
        next_page(w);
    }
    w->left -= write_size;

    w->buffer[w->n] = d;
    w->n++;
}

static void
pink_close_meta_seg_writer(pink_meta_seg_writer *w)
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
next_meta_segment(pink_file_iterator *iter)
{
    iter->buffer_n = iter->current_buffer = 0;

    pink_level_list_entry *e = iter->files[iter->current_file];
    if (!kv_is_cached(pink_lsm->lsm_cache, e->cache[LEVEL_LIST_ENTRY]))
    {
        // TODO: Change the fake PPA to real one.
        struct femu_ppa fake_ppa = { .ppa = 0 };
        fake_ppa.g.ch = e->id % 8;
        fake_ppa.g.lun = (e->id * 3) % 8;
        pink_comp_read_delay(&fake_ppa);
    }

    struct nand_page *pg = get_pg(&e->ppa);
    int n = *(uint16_t *)pg->data;

    if (!kv_is_cached(pink_lsm->lsm_cache, e->cache[META_SEGMENT]))
        pink_comp_read_delay(&e->ppa);

    char *ptr = pg->data;
    uint16_t *bitmap = (uint16_t *)ptr;
    uint16_t *vbitmap = (uint16_t *)(ptr + ((n+2) * sizeof(uint16_t)));

    for (int i = 0; i < n; i++)
    {
        pink_kv_descriptor *desc = calloc(1, sizeof(pink_kv_descriptor));

        desc->ppa = *((struct femu_ppa *)(pg->data + bitmap[i+1]));
        desc->key.len = bitmap[i+2] - bitmap[i+1] - PPA_LENGTH;
        desc->key.key = pg->data + bitmap[i+1] + PPA_LENGTH;
        desc->data_seg_offset.age = vbitmap[i+1];
        desc->value.value = NULL;
        desc->value.length = 0;

        iter->buffer[i] = desc;
    }

    iter->buffer_n += n;
    iter->current_file++;
}

static void
free_buffer(pink_file_iterator *iter)
{
    for (int i = 0; i < iter->buffer_n; i++)
        free(iter->buffer[i]);
}

static pink_kv_descriptor *
file_iter_next(pink_file_iterator *iter)
{
retry:
    if (iter->current_buffer < iter->buffer_n)
        return iter->buffer[iter->current_buffer++];

    free_buffer(iter);

    if (iter->files_n == iter->current_file)
        return NULL;

    next_meta_segment(iter);
    goto retry;
}

static int
compare_key(const void *a, const void *b)
{
    return kv_cmp_key(
               (*(pink_level_list_entry **) a)->smallest,
               (*(pink_level_list_entry **) b)->smallest
           );
}

void
pink_write_level0_table(kv_skiplist *mem)
{
    kv_key smallest, largest;
    pink_file_iterator file_iter;
    pink_kv_descriptor *desc;

    kv_skiplist_get_start_end_key(mem, &smallest, &largest);

    file_iter.files = pink_overlaps(0, smallest, largest, &file_iter.files_n);
    file_iter.current_file = 0;
    file_iter.buffer_n = 0;
    file_iter.current_buffer = 0;
    desc = file_iter_next(&file_iter);

    kv_snode *snode = mem->header->list[1];

    pink_level_list_entry *results[2048];
    int                   result_n = 0;
    pink_meta_seg_writer *w = pink_new_meta_seg_writer(results, &result_n, 0);

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

        pink_kv_descriptor *target = malloc(sizeof(pink_kv_descriptor));

        if (res < 0)
        {
            *target = *desc;

            pink_set_meta_seg_writer(w, target);

            desc = file_iter_next(&file_iter);
        }
        else
        {
            struct line *line;

            target->key = snode->key;
            target->value = *snode->value;
            target->ppa = *snode_ppa(snode);
            target->data_seg_offset.g.in_page_idx = *snode_off(snode);
            line = get_line(&target->ppa);
            target->data_seg_offset.g.line_age = line->age % LINE_AGE_MAX;

            pink_set_meta_seg_writer(w, target);

            snode = skl_iter_next(mem, snode);
            if (res == 0)
            {
                // Same key from upper level.
                // Invalidate the lowwer level section.
                line = get_line(&desc->ppa);
                if (line->vsc > 0 && line->age % LINE_AGE_MAX == desc->data_seg_offset.g.line_age) {
                    // TODO: We avoid increasing invalid sector counter on the erased block.
                    // We do this approximately by bypassing the page when the line is free status, but this is not always be true.
                    // For example, the case if the erased block is assigned to other block, this approximation is not working.
                    mark_sector_invalid(&desc->ppa);
                }
                desc = file_iter_next(&file_iter);
            }
        }

        kv_assert(result_n < 2048);
    }

    pink_close_meta_seg_writer(w);

    int new_files_n = pink_lsm->versions.n_files[0] - file_iter.files_n + result_n;
    pink_level_list_entry **new_files = malloc(new_files_n * sizeof(pink_level_list_entry *));

    int i, j, k;
    i = j = k = 0;
    while (k < pink_lsm->versions.n_files[0])
    {
        if (file_iter.files_n > 0 &&
            pink_lsm->versions.files[0][k]->id == file_iter.files[0]->id)
            break;

        new_files[i] = pink_lsm->versions.files[0][k];
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
    while (k < pink_lsm->versions.n_files[0])
    {
        new_files[i] = pink_lsm->versions.files[0][k];
        i++;
        k++;
    }
    kv_assert(i == new_files_n);

    qsort(new_files, new_files_n, sizeof(pink_level_list_entry *), compare_key);

    if (pink_lsm->versions.files[0])
        FREE(pink_lsm->versions.files[0]);
    pink_lsm->versions.files[0] = new_files;
    pink_lsm->versions.n_files[0] = new_files_n;

    for (int i = 0; i < file_iter.files_n; i++)
        pink_lput(file_iter.files[i]);

    free(file_iter.files);
}

void
pink_write_level123_table(pink_compaction *c)
{
    pink_file_iterator upper_iter;
    pink_file_iterator lower_iter;
    pink_kv_descriptor *upper, *lower;

    upper_iter.files = c->inputs[0];
    upper_iter.files_n = c->input_n[0];
    upper_iter.current_file = 0;
    upper_iter.buffer_n = 0;
    upper_iter.current_buffer = 0;
    upper = file_iter_next(&upper_iter);

    lower_iter.files = c->inputs[1];
    lower_iter.files_n = c->input_n[1];
    lower_iter.current_file = 0;
    lower_iter.buffer_n = 0;
    lower_iter.current_buffer = 0;
    lower = file_iter_next(&lower_iter);

    pink_level_list_entry **results;
    int                   result_n = 0;
    const int             alloc_margin = 2048;

    results = calloc(c->input_n[0] + c->input_n[1] + alloc_margin, sizeof(pink_level_list_entry **));
    pink_meta_seg_writer *w = pink_new_meta_seg_writer(results, &result_n, c->level+1);

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

        pink_kv_descriptor *target = malloc(sizeof(pink_kv_descriptor));

        if (res < 0)
        {
            *target = *lower;

            pink_set_meta_seg_writer(w, target);

            lower = file_iter_next(&lower_iter);
        }
        else
        {
            *target = *upper;

            pink_set_meta_seg_writer(w, target);

            upper = file_iter_next(&upper_iter);
            if (res == 0)
            {
                // Same key from upper level.
                // Invalidate the lowwer level section.
                struct line *line = get_line(&lower->ppa);
                if (line->vsc > 0 && line->age % LINE_AGE_MAX == lower->data_seg_offset.g.line_age)
                {
                    // TODO: We avoid increasing invalid sector counter on the erased block.
                    // We do this approximately by bypassing the page when the line is free status, but this is not always be true.
                    // For example, the case if the erased block is assigned to other block, this approximation is not working.
                    mark_sector_invalid(&lower->ppa);
                }
                lower = file_iter_next(&lower_iter);
            }
        }

        kv_assert(result_n < c->input_n[0] + c->input_n[1] + alloc_margin);
    }
    
    pink_close_meta_seg_writer(w);

    int new_files_n = pink_lsm->versions.n_files[c->level+1] - c->input_n[1] + result_n;
    pink_level_list_entry **new_files = malloc(new_files_n * sizeof(pink_level_list_entry *));

    int i, j, k;
    i = j = k = 0;
    while (k < pink_lsm->versions.n_files[c->level+1])
    {
        if (lower_iter.files_n > 0 && 
            pink_lsm->versions.files[c->level+1][k]->id == lower_iter.files[0]->id)
            break;

        new_files[i] = pink_lsm->versions.files[c->level+1][k];
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
    while (k < pink_lsm->versions.n_files[c->level+1])
    {
        new_files[i] = pink_lsm->versions.files[c->level+1][k];
        i++;
        k++;
    }
    kv_assert(i == new_files_n);

    qsort(new_files, new_files_n, sizeof(pink_level_list_entry *), compare_key);

    if (pink_lsm->versions.files[c->level+1])
        FREE(pink_lsm->versions.files[c->level+1]);
    pink_lsm->versions.files[c->level+1] = new_files;
    pink_lsm->versions.n_files[c->level+1] = new_files_n;

    int old_files_n = pink_lsm->versions.n_files[c->level] - c->input_n[0];
    pink_level_list_entry **old_files = malloc(old_files_n * sizeof(pink_level_list_entry *));

    i = j = k = 0;
    while (k < pink_lsm->versions.n_files[c->level])
    {
        if (pink_lsm->versions.files[c->level][k]->id == upper_iter.files[0]->id)
            break;

        old_files[i] = pink_lsm->versions.files[c->level][k];
        i++;
        k++;
    }
    k += upper_iter.files_n;
    while (k < pink_lsm->versions.n_files[c->level])
    {
        old_files[i] = pink_lsm->versions.files[c->level][k];
        i++;
        k++;
    }

    if (pink_lsm->versions.files[c->level])
        FREE(pink_lsm->versions.files[c->level]);
    pink_lsm->versions.files[c->level] = old_files;
    pink_lsm->versions.n_files[c->level] = old_files_n;

    for (int i = 0; i < upper_iter.files_n; i++)
        pink_lput(upper_iter.files[i]);
    for (int i = 0; i < lower_iter.files_n; i++)
        pink_lput(lower_iter.files[i]);

    free(results);
}

static int
binary_search(void *data, kv_key k, struct femu_ppa *data_seg_ppa, struct line_age *offset)
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

kv_value *
pink_internal_get(int eid, kv_key k1, NvmeRequest *req)
{
    kv_value *v = NULL;
    pink_level_list_entry *e = pink_lget(eid);

    if (!kv_is_cached(pink_lsm->lsm_cache, e->cache[META_SEGMENT]))
        pink_user_read_delay(&e->ppa, req);

    struct femu_ppa ppa = e->ppa;
    struct nand_page *pg = get_pg(&ppa);
    struct femu_ppa data_seg_ppa;
    struct line_age offset;

    int i = binary_search(pg->data, k1, &data_seg_ppa, &offset);
    if (i >= 0)
    {
        struct nand_page *data_seg_pg = get_pg(&data_seg_ppa);
        uint16_t *bitmap = (uint16_t *)data_seg_pg->data;
        int voff = bitmap[offset.g.in_page_idx+1];

        v = malloc(sizeof(kv_value));
        v->length = bitmap[offset.g.in_page_idx+2] - bitmap[offset.g.in_page_idx+1] - k1.len;
        v->value = malloc(v->length);
        memcpy(v->value, data_seg_pg->data + voff + k1.len, v->length);
        pink_user_read_delay(&data_seg_ppa, req);
    }

    pink_lput(e);

    return v;
}

