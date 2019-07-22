/* Driver for Realtek RTS51xx USB card reader
 *
 * Copyright(c) 2009 Realtek Semiconductor Corp. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2, or (at your option) any
 * later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 *
 * Author:
 *   wwang (wei_wang@realsil.com.cn)
 *   No. 450, Shenhu Road, Suzhou Industry Park, Suzhou, China
 */
#if 1

UNUSUAL_DEV(0x19d2, 0x117, 0x0000, 0x9999,
		"unicom-3g",
		"USB 3G com",
		USB_SC_DEVICE, USB_PR_DEVICE, NULL, 0),

UNUSUAL_DEV(0x12d1, 0x1001, 0x0000, 0x9999,
		"huawei-E1750",
		"USB 3G com",
		USB_SC_DEVICE, USB_PR_DEVICE, NULL, 0),

UNUSUAL_DEV(0x12d1, 0x140c, 0x0000, 0x9999,
		"huawei-EC122",
		"USB 3G com",
		USB_SC_DEVICE, USB_PR_DEVICE, NULL, 0),

#endif 
