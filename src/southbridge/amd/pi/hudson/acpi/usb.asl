/* SPDX-License-Identifier: GPL-2.0-only */

/* 0:12.0 - EHCI */
Device(EHC1) {
	Name(_ADR, 0x00120000)
	Name(_PRW, Package() {0x0B, 3})
} /* end EHC1 */

/* 0:13.0 - EHCI */
Device(EHC2) {
	Name(_ADR, 0x00130000)
	Name(_PRW, Package() {0x0B, 3})
} /* end EHC2 */

/* 0:16.0 - EHCI */
Device(EHC3) {
	Name(_ADR, 0x00160000)
	Name(_PRW, Package() {0x0B, 3})
} /* end EHC3 */

/* 0:10.0 - XHCI 0*/
Device(XHC0) {
	Name(_ADR, 0x00100000)
	Name(_PRW, Package() {0x0B, 4})
} /* end XHC0 */
