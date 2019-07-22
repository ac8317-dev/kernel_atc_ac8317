#ifndef _ATC_PARTITION_H_
#define _ATC_PARTITION_H_

struct atc_part_info {
	char part_name[32];
	char part_type[16];
	u8 mount_flag;
	u64 part_offset;
	u64 part_size;
};

#define PART_NUM			(19)

static const struct atc_part_info PartInfo[PART_NUM]={
			{ "preloader", "raw", 0, 0x0, 0x10000 },
			{ "datazone", "raw", 0, 0x10000, 0x1000 },
			{ "uboot", "raw", 0, 0x11000, 0x3EF000 },
			{ "env", "raw", 0, 0x400000, 0x200000 },
			{ "arm2", "raw", 0, 0x600000, 0x400000 },
			{ "logo", "raw", 0, 0xA00000, 0x400000 },
			{ "kernel", "raw", 0, 0xE00000, 0x400000 },
			{ "rootfs", "raw", 0, 0x1200000, 0x200000 },
			{ "system", "ext4", 1, 0x1400000, 0x25800000 },
			{ "data", "ext4", 1, 0x26C00000, 0x1F400000 },
			{ "cache", "ext4", 1, 0x46000000, 0x32000000 },
			{ "recovery", "raw", 0, 0x78000000, 0x400000 },
			{ "misc", "raw", 0, 0x78400000, 0x200000 },
			{ "backup", "ext4", 1, 0x78600000, 0x3200000 },
			{ "metazone", "raw", 0, 0x7B800000, 0x400000 },
			{ "dvp", "raw", 0, 0x7BC00000, 0x400000 },
			{ "data4write", "ext4", 1, 0x7C000000, 0x1F400000 },
			{ "navi_map", "fat", 1, 0x9B400000, 0x280000000 },
			{ "skypine", "fat", 1, 0x31B400000, 0x4000000 },
};

#endif /*_ATC_PARTITION_H_*/