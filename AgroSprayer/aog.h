#pragma once
void aog_begin();
void aog_update();
void aog_send_switch_now();

// Clear remote manual overrides before handing sections back to AUTO.
void aog_send_mode_now();
