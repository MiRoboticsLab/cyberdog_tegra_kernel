/*
 * Copyright (C) 2016 ARM Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef __ASM_CHECKSUM_H
#define __ASM_CHECKSUM_H

#include <linux/types.h>

static inline __sum16 csum_fold(__wsum csum)
{
	u32 sum = (__force u32)csum;
	sum += (sum >> 16) | (sum << 16);
	return ~(__force __sum16)(sum >> 16);
}
#define csum_fold csum_fold

static inline __sum16 ip_fast_csum(const void *iph, unsigned int ihl)
{
	union {
		struct {
#ifdef CONFIG_CPU_BIG_ENDIAN
			uint64_t hi;
			uint64_t lo;
#else
			uint64_t lo;
			uint64_t hi;
#endif
		};
		__uint128_t val;
	} tmp;
	u64 sum;

	asm volatile ("ldp %0, %1, [%2]\n" : "=r" (tmp.lo)
					, "=r" (tmp.hi)
					: "r" (iph));
	iph += 16;
	ihl -= 4;
	tmp.val += ((tmp.val >> 64) | (tmp.val << 64));
	sum = tmp.val >> 64;
	do {
		sum += *(const u32 *)iph;
		iph += 4;
	} while (--ihl);

	sum += ((sum >> 32) | (sum << 32));
	return csum_fold(sum >> 32);
}
#define ip_fast_csum ip_fast_csum

#include <asm-generic/checksum.h>

#endif	/* __ASM_CHECKSUM_H */
