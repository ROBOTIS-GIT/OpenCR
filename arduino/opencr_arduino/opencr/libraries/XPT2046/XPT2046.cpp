
#include <XPT2046.h>

void XPT::xpt2046_init(void)
{
	uint16_t hwXpos, hwYpos;
		
	__XPT2046_CS_OUT();
	__XPT2046_CS_SET();
	__XPT2046_IRQ_IN();

	xpt2046_read_xy(&hwXpos, &hwYpos);
}

void XPT::xpt2046_read_xy(uint16_t *phwXpos, uint16_t *phwYpos)
{
	*phwXpos = xpt2046_read_average(0xD0);
	*phwYpos = xpt2046_read_average(0x90);
}


#define ERR_RANGE 50
bool XPT::xpt2046_twice_read_xy(uint16_t *phwXpos, uint16_t *phwYpos)
{
	uint16_t hwXpos1, hwYpos1, hwXpos2, hwYpos2;

	xpt2046_read_xy(&hwXpos1, &hwYpos1);
	xpt2046_read_xy(&hwXpos2, &hwYpos2);

	if (((hwXpos2 <= hwXpos1 && hwXpos1 < hwXpos2 + ERR_RANGE) || (hwXpos1 <= hwXpos2 && hwXpos2 < hwXpos1 + ERR_RANGE))
	&& ((hwYpos2 <= hwYpos1 && hwYpos1 < hwYpos2 + ERR_RANGE) || (hwYpos1 <= hwYpos2 && hwYpos2 < hwYpos1 + ERR_RANGE))) {
		*phwXpos = (hwXpos1 + hwXpos2) / 2;
		*phwYpos = (hwYpos1 + hwYpos2) / 2;
		return true;
	}

	return false;
}

XPT Xpt = XPT();


