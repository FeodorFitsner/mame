// license:BSD-3-Clause
// copyright-holders:AJR,68bit
/****************************************************************************

Driver for Southwest Technical Products video terminal 8210.

MC6802P, 2xMC6821P, INS8250N, MCM66750, MC6845P, bank of 8 dips, crystals
17.0748 (video), 1.8432 (cpu/uart). On the back is a 25-pin RS-232 port, and a
25-pin printer port.

The 8212 terminal appears similar in design to the CT-82 terminal for which
there is some documentation and a user guide. In particular it appears closest
to the 'Version B1'. There was also a 8209 terminal with an 9 inch CRT versus
the 12 inch CRT in the 8212. This terminal has also been labeled at the
CT8200.  http://www.swtpc.com/mholley/CT_82/CT82_Index.htm

The 8212 has three CRT controller configurations:
 1. 82 x 20 characters, with a height of 14 scan lines.
 2. 82 x 24 characters, with a height of 12 scan lines.
 3. 92 x 22 characters, with a height of 12 scan lines.

There are two character generators:
1. The MCM66750 with 7x9 sized characters with some descending by 3 lines
giving an effective 7x12 sized character set.
2. An EPROM defining 8x16 sized characters for which only 12 scan lines are
used, and this is used for the graphics mode.

These appear to map onto the CT-82 screen formats options as follows:
Format I:   82 by 20, MCM66750 character set. Ctrl-\ crtl-Q
Format II:  82 by 24, MCM66750 character set. Ctrl-\ crtl-R
Format III: 82 by 20, EPROM character set.    Ctrl-\ crtl-S
Format IV:  82 by 24, EPROM character set.    Ctrl-\ crtl-T
Graphics:   92 by 22, EPROM character set.    Ctrl-] crtl-V

The terminal includes a parallel printer output and Ctrl-] Ctrl-K enables
printer pass-through and Ctrl-] Ctrl-G disables pass-through.

The terminal appears to include light pen support, and the positions is read
via Ctrl-] Ctrl-B, TODO.

TODO A generic 'beep' is used. Might want to compare this with the actual 8212
to better emulate the sound.

TODO A generic keyboard is used. Might need some work to get a better keyboard
emulation. There also appear to be separate 'cursor' key inputs, with a
separate strobe, to emulate.

TODO the CB2 input of 'pia0' at address 0x0080 is polled by the firmware, and
it is the first input checked in the ISR. On a CB2 interrupt the ISR checks
for a 'break' condition on the UART and if not then it clears the UART OUT1
output. There are other suggestions in the firmware that the OUT1 and OUT2
UART lines are being driven. The operation here is unknown?

TODO Confirm the CPU XTAL. The terminal emulation appears slugish compared
with the documented claims, the hardware flow control is needed even at 9600
and it does not operate at 38400 as documented. The CT82 manual mentions an
optional 4MHz crystal and that would run the CPU at 1MHz which is the rated
speed for the CPU and perhaps the 8212 uses that 4MHz XTAL or at least
something faster the current 1.8432MHz XTAL?

****************************************************************************/

#include "machine/input_merger.h"
#include "machine/swtpc8212.h"
#include "emu.h"
#include "screen.h"
#include "speaker.h"

swtpc8212_device::swtpc8212_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, type, tag, owner, clock)
	, m_maincpu(*this, "maincpu")
	, m_pia0(*this, "pia0")
	, m_pia1(*this, "pia1")
	, m_uart(*this, "uart")
	, m_crtc(*this, "crtc")
	, m_chargen1(*this, "chargen1")
	, m_chargen2(*this, "chargen2")
	, m_video_ram(*this, "videoram")
	, m_dip_switches(*this, "dip_switches")
	, m_config(*this, "config")
	, m_one_stop_bit(*this, "one_stop_bit")
	, m_bell_timer(nullptr)
	, m_beeper(*this, "beeper")
	, m_printer(*this, "printer")
	, m_rs232_conn_txd_handler(*this)
	, m_rs232_conn_dtr_handler(*this)
	, m_rs232_conn_rts_handler(*this)
{
}

swtpc8212_device::swtpc8212_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: swtpc8212_device(mconfig, SWTPC8212, tag, owner, clock)
{
}

WRITE8_MEMBER(swtpc8212_device::latch_w)
{
	// Bits 0 to 3 control outputs that are intended to control and tape
	// 'read' and 'punch' operations. These are strobes, about 10usec, and
	// intended to trigger on the falling edge.
	//
	// Bit 0 - Read on output
	// Bit 1 - Read off output
	// Bit 2 - Punch on output
	// Bit 3 - Punch off output

	// Bit 4 - ?? usually high
	// Bit 5 - ?? usually low

	// Bits 6 and 7 change with the screen format.
	//
	// Bit 6 is zero for formats I and II, and one for formats III and IV
	// and for the graphics format. Assume this selects between the
	// characters sets, so formats III and IV might use an alternative
	// custom text character set.
	//
	// Bit 6 - character set: 0 - standard; 1 - alternate/graphics.
	//
	// Bit 7 is zero in formats I, II, III, and IV, and one for the
	// graphics format. Assume this controls the horizontal
	// inter-character gap, eliminating the gap for the graphics format.
	//
	// Bit 7 - character width: 0 - 9 dots; 1 - 8 dots.
	//
	if (BIT(data, 7) == 0)
		m_crtc->set_char_width(9);
	else
		m_crtc->set_char_width(8);

	m_latch_data = data;
}


READ8_MEMBER(swtpc8212_device::pia0_pa_r)
{
	// PA0 controls the 'duplex' mode, the echoing back of characters, and
	// appears to connect to a switch on the outer casing.
	//
	// PA1 On the CT-82 this enabled or disabled use of an optional ROM,
	// but that function is disabled in the 8212, and this is probably
	// unused.
	//
	// PA2 is Jumper B, and PA3 is Jumper A.
	uint8_t config = m_config->read();

	// TODO:
	// PA7 - cursor bit 0
	// PA6 - cursor bit 1
	// PA5 - cursor bit 2
	// PA4 - cursor bit 3

	return config;
}

READ8_MEMBER(swtpc8212_device::pia0_pb_r)
{
	return m_kbd_data;
}

void swtpc8212_device::kbd_put(uint8_t data)
{
	m_kbd_data = data;
	// Triggers on the falling edge.
	m_pia0->cb1_w(ASSERT_LINE);
	m_pia0->cb1_w(CLEAR_LINE);
	m_pia0->cb1_w(ASSERT_LINE);
}

WRITE_LINE_MEMBER(swtpc8212_device::pia0_ca2_w)
{
	if (state == 0)
	{
		m_beeper->set_state(1);
		m_bell_timer->reset(attotime::from_msec(250));
	}
}

WRITE8_MEMBER(swtpc8212_device::pia1_pa_w)
{
	// External parallel printer data output.
	m_printer_data = data;
}

READ_LINE_MEMBER(swtpc8212_device::pia1_ca1_r)
{
	// External parallel printer busy input.
	return 0;
}

WRITE_LINE_MEMBER(swtpc8212_device::pia1_ca2_w)
{
	// External parallel printer data ready.

	// Trigger on the falling edge.
	if (m_printer_data_ready == 1 && state == 0)
	{
		m_printer->output(m_printer_data);
		// Toggle the printer busy line as the software waits for a
		// falling edge.
		m_pia1->ca1_w(CLEAR_LINE);
		m_pia1->ca1_w(ASSERT_LINE);
		m_pia1->ca1_w(CLEAR_LINE);
	}
	m_printer_data_ready = state;
}

READ8_MEMBER(swtpc8212_device::pia1_pb_r)
{
	return m_dip_switches->read();
}


MC6845_UPDATE_ROW(swtpc8212_device::update_row)
{
	int x = 0;
	uint8_t *chargen = BIT(m_latch_data, 6) == 0 ? m_chargen1 : m_chargen2;

	for (int column = 0; column < x_count; column++)
	{
		uint8_t code = m_video_ram[(ma + column) & 0x07ff];
		int dcursor = (column == cursor_x);

		offs_t address = ra < 16 ? (code & 0x7f) | (ra & 0x0f) << 7 : 0;
		uint8_t data = chargen[address];
		uint8_t intensity = BIT(code, 7);

		for (int bit = 0; bit < 8; bit++)
		{
			int dout = BIT(data, 7);
			uint32_t font_color = 0;
			if ((dcursor ^ dout) && de)
			{
				if (intensity)
					font_color = rgb_t(0x10, 0xff, 0x10);
				else
					font_color = rgb_t(0x00, 0xd0, 0x00);
			}
			bitmap.pix32(y, hbp + x++) = font_color;
			data <<= 1;
		}

		// Gap between characters
		if (BIT(m_latch_data, 7) == 0)
			x++;
	}
}

void swtpc8212_device::device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr)
{
	switch (id)
	{
	case BELL_TIMER_ID:
		m_beeper->set_state(0);
		break;
	}
}

WRITE_LINE_MEMBER(swtpc8212_device::rs232_conn_dcd_w)
{
	m_uart->dcd_w(state);
}

WRITE_LINE_MEMBER(swtpc8212_device::rs232_conn_dsr_w)
{
	m_uart->dsr_w(state);
}

WRITE_LINE_MEMBER(swtpc8212_device::rs232_conn_ri_w)
{
	m_uart->ri_w(state);
}

WRITE_LINE_MEMBER(swtpc8212_device::rs232_conn_cts_w)
{
	m_uart->cts_w(state);
}

WRITE_LINE_MEMBER(swtpc8212_device::rs232_conn_rxd_w)
{
	m_uart->rx_w(state);
}

WRITE_LINE_MEMBER(swtpc8212_device::write_txd)
{
	m_rs232_conn_txd_handler(state);
}

WRITE_LINE_MEMBER(swtpc8212_device::write_dtr)
{
	m_rs232_conn_dtr_handler(state);
}

WRITE_LINE_MEMBER(swtpc8212_device::write_rts)
{
	m_rs232_conn_rts_handler(state);
}

void swtpc8212_device::device_resolve_objects()
{
	m_rs232_conn_dtr_handler.resolve_safe();
	m_rs232_conn_rts_handler.resolve_safe();
	m_rs232_conn_txd_handler.resolve_safe();
}

void swtpc8212_device::device_start()
{
	m_bell_timer = timer_alloc(BELL_TIMER_ID);

	save_item(NAME(m_latch_data));
	save_item(NAME(m_kbd_data));
	save_item(NAME(m_printer_data));
	save_item(NAME(m_printer_data_ready));
}

void swtpc8212_device::device_reset()
{
	m_kbd_data = 0;
	m_pia0->cb1_w(ASSERT_LINE);

	m_latch_data = 0x1f;

	m_beeper->set_state(0);

	m_printer_data = 0;
	m_printer_data_ready = 1;
	m_pia1->ca1_w(CLEAR_LINE);

	if (m_one_stop_bit->read())
	{
		// Patch the firmware to use one stop bit.
		uint8_t* program = memregion("program")->base();
		program[0x01ad] = 0x02;
	}
}

void swtpc8212_device::mem_map(address_map &map)
{
	map(0x0000, 0x007f).ram();
	map(0x0080, 0x0083).rw("pia0", FUNC(pia6821_device::read), FUNC(pia6821_device::write));
	map(0x0088, 0x0088).w("crtc", FUNC(mc6845_device::address_w));
	map(0x0089, 0x0089).rw("crtc", FUNC(mc6845_device::register_r), FUNC(mc6845_device::register_w));
	map(0x008c, 0x008c).w(FUNC(swtpc8212_device::latch_w));
	map(0x0090, 0x0097).rw("uart", FUNC(ins8250_device::ins8250_r), FUNC(ins8250_device::ins8250_w));
	map(0x0098, 0x009b).rw("pia1", FUNC(pia6821_device::read), FUNC(pia6821_device::write));
	map(0x4000, 0x47ff).mirror(0x1800).ram().share(m_video_ram);
	map(0xb800, 0xbfff).rom().region("program", 0);
	map(0xc000, 0xc7ff).mirror(0x3800).rom().region("program", 0x800);
}

INPUT_PORTS_START(swtpc8212)

	PORT_START("dip_switches")
	PORT_DIPNAME(0x1f, 0x19, "Baud Rate") PORT_DIPLOCATION("DIP:4,3,2,1,0")
	PORT_DIPSETTING(0x04, "110")
	PORT_DIPSETTING(0x0a, "300")
	PORT_DIPSETTING(0x0d, "600")
	PORT_DIPSETTING(0x0f, "1200")
	PORT_DIPSETTING(0x12, "2400")
	PORT_DIPSETTING(0x16, "4800")
	PORT_DIPSETTING(0x18, "7200")
	PORT_DIPSETTING(0x19, "9600")
	PORT_DIPSETTING(0x1c, "19200")
	PORT_DIPSETTING(0x1f, "38400")
	PORT_DIPNAME(0x20, 0x00, "Mode switch") PORT_DIPLOCATION("DIP:5")
	PORT_DIPSETTING(0x00, "Conversational")
	PORT_DIPSETTING(0x20, "Page edit")
	PORT_DIPNAME(0x40, 0x00, "No Parity") PORT_DIPLOCATION("DIP:6")
	PORT_DIPSETTING(0x00, "No Parity (On)")
	PORT_DIPSETTING(0x40, "Parity (Off)")
	PORT_DIPNAME(0x80, 0x00, "Parity Select") PORT_DIPLOCATION("DIP:7")
	PORT_DIPSETTING(0x00, "Odd or Mark (On)")
	PORT_DIPSETTING(0x80, "Even or Space (Off)")

	PORT_START("config")
	PORT_CONFNAME(0x01, 0x01, "Duplex")
	PORT_CONFSETTING(0x00, "Full duplex")
	PORT_CONFSETTING(0x01, "Half duplex")
	PORT_CONFNAME(0x02, 0x02, "Option ROM (Not used)")
	PORT_CONFSETTING(0x00, DEF_STR(On))
	PORT_CONFSETTING(0x02, DEF_STR(Off))
	PORT_CONFNAME(0x04, 0x04, "Parity Select (Jumper B)")
	PORT_CONFSETTING(0x00, "Odd or Even (On)")
	PORT_CONFSETTING(0x04, "Mark or Space (Off)")
	PORT_CONFNAME(0x08, 0x08, "Data bits (Jumper A)")
	PORT_CONFSETTING(0x00, "7 bit data (On)")
	PORT_CONFSETTING(0x08, "8 bit data (Off)")

	PORT_START("one_stop_bit")
	PORT_CONFNAME(0x1, 1, "One stop bit patch")
	PORT_CONFSETTING(0, "No")
	PORT_CONFSETTING(1, "Yes - apply patch")

INPUT_PORTS_END


ROM_START( swtpc8212 )
	ROM_REGION( 0x1000, "program", 0 )
	ROM_LOAD( "8224g_ver.1.1_6oct80.ic1", 0x0000, 0x0800, CRC(7d7f3c21) SHA1(f7e6e20b36a1c724a4e348bc784d0b7b5fb462a3) )
	ROM_LOAD( "8224g_ver.1.1_6oct80.ic2", 0x0800, 0x0800, CRC(2b118c22) SHA1(5fa031c834c7c582d5715764941499fcef51f477) )

	ROM_REGION( 0x0800, "chargen1", 0 )
	ROM_LOAD( "mcm66750.rom",  0x0000, 0x0800, CRC(aedc2830) SHA1(49ce17d5b5cefb24e89ed3fd59887a652501b919) )
	ROM_REGION( 0x0800, "chargen2", 0 )
	ROM_LOAD( "grafix_8x12_22aug80.bin",  0x0000, 0x0800, CRC(a525ed65) SHA1(813d2e85ddb258c5b032b959e695ad33200cbcc4) )
ROM_END

void swtpc8212_device::device_add_mconfig(machine_config &config)
{
	M6802(config, m_maincpu, 1.8432_MHz_XTAL);
	m_maincpu->set_addrmap(AS_PROGRAM, &swtpc8212_device::mem_map);

	INPUT_MERGER_ANY_HIGH(config, "mainirq").output_handler().set_inputline(m_maincpu, M6802_IRQ_LINE);

	// PA - various jumpers and cursor inputs.
	// PB - parallel keyboard data input.
	// CA1 - cursor input strobe?
	// CA2 - output, bell.
	// CB1 - parallel keyboard input strobe.
	// CB2 ??
	pia6821_device &pia0(PIA6821(config, "pia0"));
	pia0.readpa_handler().set(FUNC(swtpc8212_device::pia0_pa_r));
	pia0.readpb_handler().set(FUNC(swtpc8212_device::pia0_pb_r));
	pia0.ca2_handler().set(FUNC(swtpc8212_device::pia0_ca2_w));
	pia0.irqa_handler().set("mainirq", FUNC(input_merger_device::in_w<0>));
	pia0.irqb_handler().set("mainirq", FUNC(input_merger_device::in_w<1>));

	// PA - parallel printer data outputs.
	// PB - various config inputs.
	// CA1 - parallel printer, 'busy' input.
	// CA2 - parallel printer, 'data ready' output.
	// CB1 - Handshake input?
	// CB2 - Handshake output?
	pia6821_device &pia1(PIA6821(config, "pia1"));
	pia1.writepa_handler().set(FUNC(swtpc8212_device::pia1_pa_w));
	pia1.readca1_handler().set(FUNC(swtpc8212_device::pia1_ca1_r));
	pia1.ca2_handler().set(FUNC(swtpc8212_device::pia1_ca2_w));
	pia1.readpb_handler().set(FUNC(swtpc8212_device::pia1_pb_r));

	ins8250_device &uart(INS8250(config, "uart", 1.8432_MHz_XTAL));
	uart.out_tx_callback().set(FUNC(swtpc8212_device::write_txd));
	uart.out_dtr_callback().set(FUNC(swtpc8212_device::write_dtr));
	uart.out_rts_callback().set(FUNC(swtpc8212_device::write_rts));
	uart.out_int_callback().set("mainirq", FUNC(input_merger_device::in_w<2>));

	screen_device &screen(SCREEN(config, "screen", SCREEN_TYPE_RASTER));
	screen.set_raw(17.0748_MHz_XTAL, 918, 0, 738, 310, 0, 280);
	screen.set_screen_update("crtc", FUNC(mc6845_device::screen_update));

	mc6845_device &crtc(MC6845(config, "crtc", 17.0748_MHz_XTAL / 9));
	crtc.set_char_width(9);
	crtc.set_screen("screen");
	crtc.set_show_border_area(false);
	crtc.set_update_row_callback(FUNC(swtpc8212_device::update_row), this);

	generic_keyboard_device &keyboard(GENERIC_KEYBOARD(config, "keyboard", 0));
	keyboard.set_keyboard_callback(FUNC(swtpc8212_device::kbd_put));

	SPEAKER(config, "bell").front_center();
	BEEP(config, m_beeper, 2000);
	m_beeper->add_route(ALL_OUTPUTS, "bell", 0.25);

	PRINTER(config, m_printer, 0);
}

ioport_constructor swtpc8212_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(swtpc8212);
}

const tiny_rom_entry *swtpc8212_device::device_rom_region() const
{
	return ROM_NAME(swtpc8212);
}

DEFINE_DEVICE_TYPE(SWTPC8212, swtpc8212_device, "swtpc8212_device", "SWTPC8212")
