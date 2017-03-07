/*
 * I2CXL_MS_EZ.cpp
 *
 *  Created on: Feb 8, 2017
 *      Author: mdbal
 */

#include "I2CXL_MS_EZ.h"
#include <sstream>


I2CXL_EZ::I2CXL_EZ(std::string name, uint8_t address, enum frc::I2C::Port port) : _wire(port, address)
{
	_address = address;
	_port = port;
	_byte1 = 0;
	_byte2 = 0;
	_lastReportedRange = -1.0;

	std::stringstream ss;
	ss << "MaxSonar_I2C_" << name;
	ss >> _myName;

	LiveWindow::GetInstance()->AddSensor(_myName, port, this);
}

I2CXL_EZ::~I2CXL_EZ()
{

}

void I2CXL_EZ::TakeMeasurement()
{
	uint8_t cmd[2];
	cmd[0] = _address;
	cmd[1] = I2CXLMSEZ_RANGE_CMD;
	Send(&cmd[0], 2);
    frc::Timer *delay = new frc::Timer();
    delay->Start();
    while (!delay->HasPeriodPassed(0.1))
    {
        // Wait 100 milliseconds
    }
//	std::cout << _myName << ": Taking Measurement." << std::endl;
};

double I2CXL_EZ::GetLastRange()
{
	uint8_t cmd = _address | 0x01;
	Send(&cmd,1);
	_lastReportedRange = (double) GetWord();
	//if (_lastReportedRange > 0.0) std::cout << _myName << ": Measured: " << _lastReportedRange << std::endl;
	return _lastReportedRange;
};


bool I2CXL_EZ::ChangeAddress(uint8_t toAddress)
{
	// Reject invalid addresses.
	if ((toAddress == 0) || (toAddress == 80) || (toAddress == 164) || (toAddress == 170)) return false;
	if ((toAddress % 2) != 0) return false;

	uint8_t cmd[4];
	cmd[0] = _address;
	cmd[1] = I2CXLMSEZ_ADDRESS_UNLOCK1;
	cmd[2] = I2CXLMSEZ_ADDRESS_UNLOCK2;
	cmd[3] = toAddress;
	Send(&cmd[0],4);
	return true;
};

uint16_t I2CXL_EZ::GetWord()
{
    uint8_t bytes[2];
	_wire.Read(_address,2,bytes);
    uint16_t w = (bytes[0] << 8) + bytes[1];
    return w;
}

uint8_t I2CXL_EZ::GetByte()
{
	_wire.ReadOnly(1,&_byte1);
	return _byte1;
}

int8_t I2CXL_EZ::Send(uint8_t *data, uint8_t len)
{
    _wire.WriteBulk(data, len);
    return len;
}


double I2CXL_EZ::PIDGet() {
	return _lastReportedRange;
}

void I2CXL_EZ::SetPIDSourceType(PIDSourceType pidSource) {
  if (wpi_assert(pidSource == PIDSourceType::kDisplacement)) {
    m_pidSource = pidSource;
  }
}

void I2CXL_EZ::UpdateTable() {
  if (m_table != nullptr) {
	  TakeMeasurement();
	  GetLastRange();
	  m_table->PutNumber("Value", _lastReportedRange);
  }
}

void I2CXL_EZ::StartLiveWindowMode() {}

void I2CXL_EZ::StopLiveWindowMode() {}

std::string I2CXL_EZ::GetSmartDashboardType() const { return "Ultrasonic"; }

void I2CXL_EZ::InitTable(std::shared_ptr<ITable> subTable) {
  m_table = subTable;
  UpdateTable();
}

std::shared_ptr<ITable> I2CXL_EZ::GetTable() const { return m_table; }

