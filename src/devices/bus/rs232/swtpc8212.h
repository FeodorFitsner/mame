// license:BSD-3-Clause
// copyright-holders:68bit

#ifndef MAME_BUS_RS232_SWTPC8212_H
#define MAME_BUS_RS232_SWTPC8212_H

#pragma once

#include "rs232.h"
#include "machine/swtpc8212.h"


class swtpc8212_terminal_device : public swtpc8212_device,
	public device_rs232_port_interface
{
public:
	swtpc8212_terminal_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual WRITE_LINE_MEMBER(input_txd) override;

protected:
	virtual ioport_constructor device_input_ports() const override;
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_add_mconfig(machine_config &config) override;

private:
	required_ioport m_flow_control;

	DECLARE_WRITE_LINE_MEMBER(route_term_rts);
	DECLARE_WRITE_LINE_MEMBER(route_term_dtr);
};

DECLARE_DEVICE_TYPE(SERIAL_TERMINAL_SWTPC8212, swtpc8212_terminal_device)

#endif // MAME_BUS_RS232_SWTPC8212_H
