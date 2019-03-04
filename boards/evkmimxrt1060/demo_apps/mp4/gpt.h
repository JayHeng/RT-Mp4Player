

void     gp_timer_init(void);
uint32_t gp_timer_get_cnt(void);
uint32_t gp_timer_compute_us(uint32_t cnt2, uint32_t cnt1);
void     gp_timer_delay(uint32_t us_delay);
void     gp_timer_measure_begin(void);
uint32_t gp_timer_measure_end(void);

