// license:BSD-3-Clause
// copyright-holders:68bit

#include "emu.h"
#include "swtpc8212.h"

swtpc8212_terminal_device::swtpc8212_terminal_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: swtpc8212_device(mconfig, SERIAL_TERMINAL_SWTPC8212, tag, owner, clock)
	, device_rs232_port_interface(mconfig, *this)
	, m_flow_control(*this, "flow_control")
{
}

void swtpc8212_terminal_device::device_add_mconfig(machine_config &config)
{
	swtpc8212_device::device_add_mconfig(config);

	rs232_conn_txd_handler().set(FUNC(swtpc8212_terminal_device::output_rxd));
	rs232_conn_rts_handler().set(FUNC(swtpc8212_terminal_device::route_term_rts));
	rs232_conn_dtr_handler().set(FUNC(swtpc8212_terminal_device::route_term_dtr));
}

static INPUT_PORTS_START(swtpc8212_terminal)
	PORT_INCLUDE(swtpc8212)

	PORT_START("flow_control")
	PORT_CONFNAME(0xf, 1, "Flow control")
	PORT_CONFSETTING(0x00, "None")
	PORT_CONFSETTING(0x01, "Terminal DTR to remote CTS")

INPUT_PORTS_END

ioport_constructor swtpc8212_terminal_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(swtpc8212_terminal);
}

WRITE_LINE_MEMBER(swtpc8212_terminal_device::input_txd)
{
	swtpc8212_device::rs232_conn_rxd_w(state);
}

WRITE_LINE_MEMBER(swtpc8212_terminal_device::route_term_rts)
{
	// Loop the terminal RTS output to the terminal CTS input.
	swtpc8212_device::rs232_conn_cts_w(state);
}

// This terminal uses DTR for hardware flow control.
WRITE_LINE_MEMBER(swtpc8212_terminal_device::route_term_dtr)
{
	if (m_flow_control->read() == 1)
	{
		// Connect the terminal DTR output to CTS at the other end.
		swtpc8212_terminal_device::output_cts(state);
	}
}

void swtpc8212_terminal_device::device_start()
{
	swtpc8212_device::device_start();
}

void swtpc8212_terminal_device::device_reset()
{
	// To the terminal
	swtpc8212_device::rs232_conn_cts_w(0);

	// To the computer
	output_rxd(1);
	output_dcd(0);
	output_dsr(0);
	output_cts(0);

	swtpc8212_device::device_reset();
}

DEFINE_DEVICE_TYPE(SERIAL_TERMINAL_SWTPC8212, swtpc8212_terminal_device, "swtpc8212_terminal", "SWTPC8212 Terminal")
