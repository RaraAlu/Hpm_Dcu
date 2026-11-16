#include "pdo_pipeline.h"
#include "cycle_config.h"
#include <stdio.h>
#include <string.h>

#define CHECK(expr) do { if (!(expr)) { \
    fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #expr); return 1; \
} } while (0)

static int run(const char *name)
{
    pdo_pipeline_t p;
    pdo_pipeline_view_t first, second;
    unsigned char a[PDO_PIPELINE_RX_MAX], b[PDO_PIPELINE_RX_MAX];
    memset(a, 0xA5, sizeof(a));
    memset(b, 0x5A, sizeof(b));

    if (!strcmp(name, "configuration")) {
        CHECK(!pdo_pipeline_configure(&p, 999999U, 250U, 784));
        CHECK(!pdo_pipeline_configure(&p, 5000001U, 250U, 784));
        CHECK(pdo_pipeline_configure(&p, 1000000U, 250U, 784));
        CHECK(pdo_pipeline_configure(&p, 5000000U, 250U, 544));
        CHECK(pdo_pipeline_configure(&p, 3333333U, 250U, 736));
        CHECK(!pdo_pipeline_configure(&p, 1000000U, 900U, 784));
        CHECK(!pdo_pipeline_configure(&p, 1000000U, UINT32_MAX, 784));
        CHECK(!pdo_pipeline_configure(&p, 1000000U, 1U, 784));
        CHECK(!pdo_pipeline_configure(&p, 1000000U, 250U, 785));
        CHECK(!pdo_pipeline_configure(&p, 1000000U, 250U, 0));
        pdo_pipeline_enable(&p, true);
        CHECK(!pdo_pipeline_stage(&p, a, 784, 1));
        return 0;
    }

    CHECK(pdo_pipeline_configure(&p, 2000000U, 250U, sizeof(a)));
    pdo_pipeline_enable(&p, true);
    if (!strcmp(name, "boundary")) {
        CHECK(pdo_pipeline_stage(&p, a, sizeof(a), 10));
        CHECK(!pdo_pipeline_acquire(&p, &first));
        pdo_pipeline_sync(&p, 100);
        CHECK(pdo_pipeline_acquire(&p, &first));
        CHECK(first.sequence == 10 && first.epoch == 1 && first.sync_us == 100);
        CHECK(!memcmp(first.data, a, sizeof(a)));
        CHECK(pdo_pipeline_stage(&p, b, sizeof(b), 11));
        CHECK(!pdo_pipeline_acquire(&p, &second));
        CHECK(!memcmp(first.data, a, sizeof(a)));
        pdo_pipeline_release(&p, &first);
        CHECK(pdo_pipeline_mark_published(&p, 1, 1800));
        pdo_pipeline_sync(&p, 2100);
        CHECK(pdo_pipeline_acquire(&p, &second));
        CHECK(second.sequence == 11 && second.epoch == 2);
        CHECK(!memcmp(second.data, b, sizeof(b)));
        pdo_pipeline_release(&p, &second);
        return 0;
    }
    if (!strcmp(name, "latest")) {
        for (unsigned i = 0; i < 100; ++i)
            CHECK(pdo_pipeline_stage(&p, i & 1 ? b : a, sizeof(a), i));
        pdo_pipeline_sync(&p, 0);
        CHECK(pdo_pipeline_acquire(&p, &first));
        CHECK(first.sequence == 99 && !memcmp(first.data, b, sizeof(b)));
        CHECK(!pdo_pipeline_stage(&p, a, sizeof(a) - 1, 100));
        CHECK(!pdo_pipeline_stage(&p, NULL, sizeof(a), 100));
        pdo_pipeline_release(&p, &first);
        return 0;
    }
    if (!strcmp(name, "ownership")) {
        CHECK(pdo_pipeline_stage(&p, a, sizeof(a), 1));
        pdo_pipeline_sync(&p, 0);
        CHECK(pdo_pipeline_acquire(&p, &first));
        for (unsigned i = 0; i < 100; ++i) {
            CHECK(pdo_pipeline_stage(&p, b, sizeof(b), i + 2));
            CHECK(!memcmp(first.data, a, sizeof(a)));
        }
        CHECK(pdo_pipeline_mark_published(&p, 1, 1700));
        pdo_pipeline_sync(&p, 2000);
        CHECK(pdo_pipeline_acquire(&p, &second));
        CHECK(second.sequence == 101 && !memcmp(second.data, b, sizeof(b)));
        CHECK(!memcmp(first.data, a, sizeof(a)));
        pdo_pipeline_release(&p, &second);
        pdo_pipeline_release(&p, &first);
        return 0;
    }
    if (!strcmp(name, "overrun")) {
        CHECK(pdo_pipeline_stage(&p, a, sizeof(a), 1));
        pdo_pipeline_sync(&p, 0);
        CHECK(pdo_pipeline_mark_published(&p, 1, 1700));
        CHECK(pdo_pipeline_stage(&p, b, sizeof(b), 2));
        pdo_pipeline_sync(&p, 2000);
        CHECK(pdo_pipeline_check(&p, 2000) & PDO_PIPELINE_RX_OVERRUN);
        CHECK(pdo_pipeline_acquire(&p, &first));
        CHECK(first.sequence == 2 && !memcmp(first.data, b, sizeof(b)));
        pdo_pipeline_release(&p, &first);
        return 0;
    }
    if (!strcmp(name, "missing")) {
        CHECK(pdo_pipeline_stage(&p, a, sizeof(a), 1));
        pdo_pipeline_sync(&p, 0);
        CHECK(pdo_pipeline_acquire(&p, &first));
        pdo_pipeline_release(&p, &first);
        CHECK(pdo_pipeline_mark_published(&p, 1, 1700));
        pdo_pipeline_sync(&p, 2000);
        CHECK(pdo_pipeline_check(&p, 2000) & PDO_PIPELINE_RX_MISSING);
        CHECK(!pdo_pipeline_acquire(&p, &first));
        return 0;
    }
    if (!strcmp(name, "deadline")) {
        pdo_pipeline_sync(&p, 100);
        CHECK(!pdo_pipeline_publish_due(&p, 1749));
        CHECK(pdo_pipeline_publish_due(&p, 1750));
        CHECK(pdo_pipeline_mark_published(&p, 1, 1850));
        CHECK(!pdo_pipeline_publish_due(&p, 1850));
        CHECK(pdo_pipeline_check(&p, 1850) == 0);
        pdo_pipeline_sync(&p, 2100);
        CHECK(!pdo_pipeline_mark_published(&p, 1, 2100));
        CHECK(pdo_pipeline_check(&p, 2100) & PDO_PIPELINE_TX_LATE);
        return 0;
    }
    if (!strcmp(name, "faults")) {
        pdo_pipeline_sync(&p, 100);
        CHECK(pdo_pipeline_check(&p, 1850) == 0);
        CHECK(pdo_pipeline_check(&p, 1851) & PDO_PIPELINE_TX_LATE);
        CHECK(pdo_pipeline_check(&p, 4101) & PDO_PIPELINE_SYNC_LOST);
        CHECK(pdo_pipeline_configure(&p, 1000000U, 250U, sizeof(a)));
        pdo_pipeline_enable(&p, true);
        pdo_pipeline_sync(&p, 0);
        pdo_pipeline_sync(&p, 1000);
        CHECK(pdo_pipeline_check(&p, 1000) & PDO_PIPELINE_TX_LATE);
        return 0;
    }
    if (!strcmp(name, "preempted_time")) {
        pdo_pipeline_sync(&p, 100);
        CHECK(pdo_pipeline_mark_published(&p, 1, 1800));
        pdo_pipeline_sync(&p, 2100);
        CHECK(!pdo_pipeline_publish_due(&p, 2099));
        CHECK(pdo_pipeline_check(&p, 2099) == 0);
        /* A stale completion flag must not acknowledge a newer boundary. */
        p.published = 1;
        p.published_epoch = 1;
        CHECK(pdo_pipeline_publish_due(&p, 3750));
        CHECK(pdo_pipeline_check(&p, 3851) & PDO_PIPELINE_TX_LATE);
        return 0;
    }
    if (!strcmp(name, "wrap")) {
        p.epoch = UINT32_MAX;
        pdo_pipeline_sync(&p, UINT32_MAX - 1000U);
        CHECK(p.epoch == 0);
        CHECK(!pdo_pipeline_publish_due(&p, 648));
        CHECK(pdo_pipeline_publish_due(&p, 649));
        CHECK(pdo_pipeline_mark_published(&p, 0, 699));
        CHECK(pdo_pipeline_check(&p, 700) == 0);
        return 0;
    }
    if (!strcmp(name, "reset")) {
        CHECK(pdo_pipeline_stage(&p, a, sizeof(a), 1));
        pdo_pipeline_enable(&p, false);
        pdo_pipeline_sync(&p, 100);
        CHECK(!pdo_pipeline_acquire(&p, &first));
        CHECK(!pdo_pipeline_stage(&p, b, sizeof(b), 2));
        CHECK(pdo_pipeline_configure(&p, 1000000U, 250U, sizeof(a)));
        pdo_pipeline_enable(&p, true);
        pdo_pipeline_sync(&p, 200);
        CHECK(!pdo_pipeline_acquire(&p, &first));
        CHECK(pdo_pipeline_check(&p, 200) == 0);
        return 0;
    }
    return 2;
}

int main(int argc, char **argv)
{
    return argc == 2 ? run(argv[1]) : 2;
}
