/* SPDX-License-Identifier: GPL-2.0-only */

Scope(\_GPE) {	/* Start Scope GPE */

	/*  USB controller PME#  */
	Method(_L0B) {
		/* DBGO("\\_GPE\\_L0B\n") */
		Notify(\_SB.PCI0.EHC1, 0x02) /* NOTIFY_DEVICE_WAKE */
		Notify(\_SB.PCI0.EHC2, 0x02) /* NOTIFY_DEVICE_WAKE */
		Notify(\_SB.PCI0.EHC3, 0x02) /* NOTIFY_DEVICE_WAKE */
		Notify(\_SB.PCI0.XHC0, 0x02) /* NOTIFY_DEVICE_WAKE */
	}

	/*  GPIO0 or GEvent8 event  */
	Method(_L18) {
		/* DBGO("\\_GPE\\_L18\n") */
		Notify(\_SB.PCI0.PBR4, 0x02) /* NOTIFY_DEVICE_WAKE */
		Notify(\_SB.PCI0.PBR5, 0x02) /* NOTIFY_DEVICE_WAKE */
		Notify(\_SB.PCI0.PBR6, 0x02) /* NOTIFY_DEVICE_WAKE */
		Notify(\_SB.PCI0.PBR7, 0x02) /* NOTIFY_DEVICE_WAKE */
		Notify(\_SB.PCI0.PBR8, 0x02) /* NOTIFY_DEVICE_WAKE */
	}

	/*  SATA Controller PME#  */
	Method(_L1E) {
		Notify(\_SB.PCI0.STCR, 0x02) /* NOTIFY_DEVICE_WAKE */
	}


}	/* End Scope GPE */
