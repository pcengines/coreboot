/* SPDX-License-Identifier: GPL-2.0-only */

#include <bootstate.h>
#include <console/console.h>
#include <intelblocks/cse.h>
#include <security/vboot/vboot_common.h>
#include <soc/intel/common/reset.h>
#include <types.h>

enum cse_eop_result {
	CSE_EOP_RESULT_GLOBAL_RESET_REQUESTED,
	CSE_EOP_RESULT_SUCCESS,
	CSE_EOP_RESULT_ERROR,
};

static enum cse_eop_result cse_send_eop(void)
{
	enum {
		EOP_REQUESTED_ACTION_CONTINUE = 0,
		EOP_REQUESTED_ACTION_GLOBAL_RESET = 1,
	};
	struct end_of_post_msg {
		struct mkhi_hdr hdr;
	} __packed msg = {
		.hdr = {
			.group_id = MKHI_GROUP_ID_GEN,
			.command = MKHI_END_OF_POST,
		},
	};
	struct end_of_post_resp {
		struct mkhi_hdr hdr;
		uint32_t requested_actions;
	} __packed resp = {};
	size_t resp_size = sizeof(resp);

	/*
	 * Prerequisites:
	 * 1) HFSTS1 CWS is Normal
	 * 2) HFSTS1 COM is Normal
	 * 3) Only sent after DID (accomplished by compiling this into ramstage)
	 */
	if (!cse_is_hfs1_cws_normal() || !cse_is_hfs1_com_normal()) {
		printk(BIOS_ERR, "HECI: Prerequisites not met for sending EOP\n");
		return CSE_EOP_RESULT_ERROR;
	}

	printk(BIOS_INFO, "HECI: Sending End-of-Post\n");

	if (!heci_send_receive(&msg, sizeof(msg), &resp, &resp_size)) {
		printk(BIOS_ERR, "HECI: EOP send/receive fail\n");
		return CSE_EOP_RESULT_ERROR;
	}

	if (resp.hdr.result) {
		printk(BIOS_ERR, "HECI: EOP Resp Failed: %u\n", resp.hdr.result);
		return CSE_EOP_RESULT_ERROR;
	}

	printk(BIOS_INFO, "CSE: EOP requested action: ");

	switch (resp.requested_actions) {
	case EOP_REQUESTED_ACTION_GLOBAL_RESET:
		printk(BIOS_INFO, "global reset\n");
		return CSE_EOP_RESULT_GLOBAL_RESET_REQUESTED;
	case EOP_REQUESTED_ACTION_CONTINUE:
		printk(BIOS_INFO, "continue boot\n");
		return CSE_EOP_RESULT_SUCCESS;
	default:
		printk(BIOS_INFO, "unknown %u\n", resp.requested_actions);
		return CSE_EOP_RESULT_ERROR;
	}
}

static void handle_cse_eop_result(enum cse_eop_result result)
{
	switch (result) {
	case CSE_EOP_RESULT_GLOBAL_RESET_REQUESTED:
		printk(BIOS_INFO, "CSE requested global reset in EOP response, resetting...\n");
		do_global_reset();
		break;
	case CSE_EOP_RESULT_SUCCESS:
		printk(BIOS_INFO, "CSE EOP successful, continuing boot\n");
		break;
	case CSE_EOP_RESULT_ERROR: /* fallthrough */
	default:
		printk(BIOS_ERR, "ERROR: Failed to send EOP to CSE, %d\n", result);
		/* For vboot, trigger recovery mode if applicable, as there is
		   likely something very broken in this case. */
		if (CONFIG(VBOOT) && !vboot_recovery_mode_enabled())
			cse_trigger_vboot_recovery(CSE_EOP_FAIL);
		break;
	}
}

static void set_cse_end_of_post(void *unused)
{
	handle_cse_eop_result(cse_send_eop());
}

/*
 * Ideally, to give coreboot maximum flexibility, sending EOP would be done as
 * late possible, just before loading the payload, which would be BS_ON_EXIT
 * here, but the platforms this is currently supported for all select
 * HECI_DISABLE_USING_SMM, which runs in BS_ON_EXIT. Because sending EOP
 * requires HECI to be up, and it is not trivial to control the order in which
 * these callbacks are issued, it is called on BS_ON_ENTRY.
 */
BOOT_STATE_INIT_ENTRY(BS_PAYLOAD_LOAD, BS_ON_ENTRY, set_cse_end_of_post, NULL);
