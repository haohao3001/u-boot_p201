// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2016, Amlogic Inc
 *
 * Amlogic EPT (Emmc Partition Table) partition type driver.
 *
 * Ported to the current DM-based partition framework from Amlogic's
 * legacy U-Boot ("disk/part_aml.c" and "drivers/mmc/aml_emmc_partition.c").
 *
 * The EPT is an MPT-formatted binary partition table kept inside the
 * eMMC "reserved" partition. The reserved partition starts at byte
 * offset 36MB (LBA 73728) and its layout is:
 *
 *   0x000000 - 0x003fff: partition table (this driver)
 *   0x004000 - 0x03ffff: storage key area
 *   0x400000 - 0x47ffff: dtb area
 *
 * On-disk binary layout of the table, byte-compatible with the legacy
 * "struct ptbl_rsv" written by update_ptbl_rsv():
 *
 *   magic[4]    = "MPT"
 *   version[12] = "01.00.00" (v1) / "01.02.00" (v2)
 *   count       = number of partitions in use (int)
 *   checksum    = summed u32 checksum over the entry area (int)
 *   entries[]   = up to 32 entries, each byte-compatible with the
 *                 legacy "struct partitions":
 *                 name[16], size(le64), offset(le64), mask_flags(le32)
 *                 (sizeof == 40)
 *
 * Partition numbers are exposed as 1-based as required by the current
 * framework (part 1 = first partition), while the on-disk table itself
 * is 0-based, therefore on-disk entry i maps to partition number i+1.
 */

#include <blk.h>
#include <command.h>
#include <memalign.h>
#include <part.h>
#include <vsprintf.h>
#include <asm/unaligned.h>

#define AML_MPT_OFFSET		73728	/* 36MB, in 512-byte sectors */
#define AML_MPT_MAGIC		"MPT"
#define AML_MPT_VERSION_V1	"01.00.00"
#define AML_MPT_VERSION_V2	"01.02.00"
#define AML_MPT_ENTRIES		32	/* MAX_MMC_PART_NUM */
#define AML_MPT_ENTRY_NAME_LEN	16	/* MAX_PART_NAME_LEN */

/* One table entry; byte-compatible with legacy "struct partitions" */
struct aml_ptbl_entry {
	char name[AML_MPT_ENTRY_NAME_LEN];
	u64 size;
	u64 offset;
	u32 mask_flags;
} __attribute__((aligned(8)));

/* Whole table; byte-compatible with legacy "struct ptbl_rsv" */
struct aml_mpt_ptbl {
	char magic[4];
	unsigned char version[12];
	int count;
	int checksum;
	struct aml_ptbl_entry entries[AML_MPT_ENTRIES];
};

/*
 * Parsed-table cache, mirroring the legacy global p_iptbl_ept.
 * Gets invalidated when the block device number changes.
 */
static struct {
	int devnum;
	int count;
	struct aml_ptbl_entry entries[AML_MPT_ENTRIES];
} aml_tbl = { .devnum = -1 };

/* v1 checksum (table version "01.00.00") */
static int aml_check_entries_v1(const struct aml_ptbl_entry *part, int count)
{
	/*
	 * Historical bug preserved on purpose (the legacy driver carries
	 * the comment "BUG here, do not fix it!!"): every loop iteration
	 * re-accumulates the FIRST entry instead of advancing through the
	 * array. Amlogic's burning tools compute the stored checksum the
	 * same way, so the value on disk only matches if the buggy
	 * algorithm is reproduced exactly.
	 */
	int checksum = 0;
	const u32 *p;
	int i, j;

	for (i = 0; i < count; i++) {
		p = (const u32 *)part;
		for (j = sizeof(*part) / sizeof(u32); j > 0; j--)
			checksum += *p++;
	}

	return checksum;
}

/* v2 checksum (table version "01.02.00") */
static int aml_check_entries_v2(const struct aml_ptbl_entry *part, int count)
{
	int checksum = 0;
	const u32 *buf = (const u32 *)part;
	int i, n = (count * sizeof(*part)) >> 2;

	for (i = 0; i < n; i++)
		checksum += buf[i];

	return checksum;
}

/* Return 1 for v1, 2 for v2, 0 for unsupported version */
static int aml_mpt_version(const unsigned char *ver)
{
	if (!strcmp((const char *)ver, AML_MPT_VERSION_V2))
		return 2;
	if (!strcmp((const char *)ver, AML_MPT_VERSION_V1))
		return 1;
	return 0;
}

/* Validate magic/count/checksum of a freshly read table */
static int aml_mpt_validate(const struct aml_mpt_ptbl *tbl)
{
	int version;

	if (strncmp(tbl->magic, AML_MPT_MAGIC, 3))
		return -EINVAL;
	if (tbl->count < 1 || tbl->count > AML_MPT_ENTRIES)
		return -EINVAL;

	version = aml_mpt_version(tbl->version);
	if (version == 1) {
		if (aml_check_entries_v1(tbl->entries, tbl->count) != tbl->checksum)
			return -EINVAL;
	} else if (version == 2) {
		if (aml_check_entries_v2(tbl->entries, tbl->count) != tbl->checksum)
			return -EINVAL;
	} else {
		return -EINVAL;
	}

	return 0;
}

/* Read and validate the MPT table from the device, caching the result */
static int aml_ptbl_load(struct blk_desc *desc)
{
	ALLOC_CACHE_ALIGN_BUFFER(unsigned char, buffer,
				 (sizeof(struct aml_mpt_ptbl) + 511) / 512 * 512);
	struct aml_mpt_ptbl *tbl = (struct aml_mpt_ptbl *)buffer;
	unsigned long nblks;
	int ret;

	/* the MPT offset/layout is defined for 512-byte sectors */
	if (desc->blksz != 512)
		return -ENOTSUPP;

	if (aml_tbl.devnum == desc->devnum && aml_tbl.count > 0)
		return 0;

	nblks = (sizeof(*tbl) + 511) / 512;
	if (blk_dread(desc, AML_MPT_OFFSET, nblks, buffer) != nblks)
		return -EIO;

	ret = aml_mpt_validate(tbl);
	if (ret)
		return ret;

	aml_tbl.devnum = desc->devnum;
	aml_tbl.count = tbl->count;
	memcpy(aml_tbl.entries, tbl->entries, sizeof(aml_tbl.entries));

	return 0;
}

static int part_get_info_aml(struct blk_desc *desc, int part,
			     struct disk_partition *info)
{
	struct aml_ptbl_entry *entry;
	char name[AML_MPT_ENTRY_NAME_LEN + 1];
	int ret;

	if (part < 1 || part > AML_MPT_ENTRIES)
		return -1;

	ret = aml_ptbl_load(desc);
	if (ret)
		return -1;

	if (part > aml_tbl.count)
		return -1;

	entry = &aml_tbl.entries[part - 1];

	memcpy(name, entry->name, AML_MPT_ENTRY_NAME_LEN);
	name[AML_MPT_ENTRY_NAME_LEN] = '\0';

	info->start = le64_to_cpu(get_unaligned_le64(&entry->offset)) /
		      desc->blksz;
	info->size = le64_to_cpu(get_unaligned_le64(&entry->size)) /
		     desc->blksz;
	info->blksz = desc->blksz;
	strlcpy((char *)info->name, name, PART_NAME_LEN);
	strcpy((char *)info->type, BOOT_PART_TYPE);
	info->bootable = 0;
	info->type_flags = 0;
	disk_partition_clr_uuid(info);
	disk_partition_clr_type_guid(info);
#ifdef CONFIG_DOS_PARTITION
	info->sys_ind = 0;
#endif

	return 0;
}

static void part_print_aml(struct blk_desc *desc)
{
	struct disk_partition info;
	int i;

	if (aml_ptbl_load(desc)) {
		printf("** Can't read AML partition table on device %d **\n",
		       desc->devnum);
		return;
	}

	printf("Part   Start     Sect x Size   Type  name\n");
	for (i = 1; i <= aml_tbl.count; i++) {
		if (part_get_info_aml(desc, i, &info))
			continue;
		printf(" %02d " LBAFU " " LBAFU " %6ld %.32s %.32s\n",
		       i, info.start, info.size, info.blksz,
		       info.type, info.name);
	}
}

static int part_test_aml(struct blk_desc *desc)
{
	ALLOC_CACHE_ALIGN_BUFFER(unsigned char, buffer, desc->blksz);

	if (blk_dread(desc, AML_MPT_OFFSET, 1, buffer) != 1)
		return -1;

	if (!strncmp((char *)buffer, AML_MPT_MAGIC, 3))
		return 0;

	return 1;
}

U_BOOT_PART_TYPE(aml) = {
	.name		= "AML",
	.part_type	= PART_TYPE_AML,
	.max_entries	= AML_MPT_ENTRIES,
	.get_info	= part_get_info_ptr(part_get_info_aml),
	.print		= part_print_ptr(part_print_aml),
	.test		= part_test_aml,
};