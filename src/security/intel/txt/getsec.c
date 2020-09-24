/* SPDX-License-Identifier: GPL-2.0-only */

#include <stdint.h>
#include <cpu/x86/lapic.h>
#include <cpu/x86/cr.h>
#include <cpu/x86/cache.h>
#include <cpu/x86/mp.h>

#include "txt_register.h"
#include "txt_getsec.h"

/**
 * Check for SMX support and enable it if possible.
 *
 * Returns false on error, true on success.
 */
static bool getsec_enabled(void)
{
	unsigned int ecx = cpuid_ecx(1);
	/*
	 * Check if SMX and VMX is supported by CPU.
	 */
	if (!(ecx & CPUID_SMX) || !(ecx & CPUID_VMX))
		return false;

	/*
	 * Check if SMX, VMX and GetSec instructions haven't been disabled.
	 */
	msr_t msr = rdmsr(IA32_FEATURE_CONTROL);
	if ((msr.lo & 0xff06) != 0xff06)
		return false;

	/*
	 * Enable SMX. Required to execute GetSec instruction.
	 * Chapter 2.2.4.3
	 * Intel TXT Software Development Guide (Document: 315168-015)
	 */
	write_cr4(read_cr4() | CR4_SMXE);

	return true;
}

/**
 * Get information as returned by getsec[PARAMETER].
 * Arguments can be set to NULL if not needed.
 *
 * Returns false on error, true on success.
 */
bool getsec_parameter(uint32_t *version_mask,
		      uint32_t *version_numbers_supported,
		      uint32_t *max_size_acm_area,
		      uint32_t *memory_type_mask,
		      uint32_t *senter_function_disable,
		      uint32_t *txt_feature_flags)
{
	uint32_t i, eax, ebx, ecx;

	if (!getsec_enabled())
		return false;

	/*
	 * SAFER MODE EXTENSIONS REFERENCE.
	 * Intel 64 and IA-32 Architectures Software Developer Manuals Vol 2D
	 */
	for (i = 0; i < 0x1f; i++) {
		/* Getsec[PARAMETERS] */
		asm volatile ("getsec\n"
			: "=a" (eax), "=b" (ebx), "=c" (ecx)
			: "a" (IA32_GETSEC_PARAMETERS), "b" (i) :);
		switch (eax & 0x1f) {
		case 0: /* NULL - Exit marker */
			return true;
		case 1: /* Supported AC module versions */
			if (version_mask)
				*version_mask = ebx;
			if (version_numbers_supported)
				*version_numbers_supported = ecx;
			break;
		case 2: /* Max size of authenticated code execution area */
			if (max_size_acm_area)
				*max_size_acm_area = eax & ~0x1f;
			break;
		case 3: /* External memory types supported during AC mode */
			if (memory_type_mask)
				*memory_type_mask = eax;
			break;
		case 4: /* Selective SENTER functionality control */
			if (senter_function_disable)
				*senter_function_disable = eax & (0x3f00);
			break;
		case 5: /* TXT extensions support */
			if (txt_feature_flags)
				*txt_feature_flags = eax & (0x60);
			break;
		}
	}

	return true;
}

/**
 * Get capabilities as returned by getsec[CAPABILITIES].
 *
 * Returns false on error, true on success.
 */

bool getsec_capabilities(uint32_t *eax)
{
	if (!getsec_enabled())
		return false;

	asm volatile ("getsec\n"
		: "=a" (*eax)
		: "a" (IA32_GETSEC_CAPABILITIES), "b" (0) :);

	return true;
}
