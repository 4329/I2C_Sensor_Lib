/*
 * I2CXL_MS_EZ.h
 *
 *  Created on: Feb 8, 2017
 *      Author: mdbal
 */

#ifndef SRC_I2CXL_MS_EZ_H_
#define SRC_I2CXL_MS_EZ_H_

#include "WPILIB.h"
#include "I2C.h"
#include "LiveWindow/LiveWindowSendable.h"
#include "PIDSource.h"
#include "SensorBase.h"
#include <string>


#define I2CXLMSEZ_DEFAULT_ADDR     0xE0
#define I2CXLMSEZ_RANGE_CMD        0x51
#define I2CXLMSEZ_ADDRESS_UNLOCK1  0xAA
#define I2CXLMSEZ_ADDRESS_UNLOCK2  0xA5

#define I2CXLMSEZ_MAXRANGE_CM      765
#define MB1202_MINRANGE_CM         25
#define MB1212_MINRANGE_CM         25
#define MB1222_MINRANGE_CM         20
#define MB1232_MINRANGE_CM         20
#define MB1242_MINRANGE_CM         20

/*
 * I2C Take Measurement Command  (should be sent 80ms prior to reading)
 * Bytes  Description            (Allow 100ms between reading for best result)
----------------------------------------------------------------
	 0      Address
	 1      0x51     Range command
 */

/*
 * I2C Report Last Range Command
 * Bytes  Description
----------------------------------------------------------------
	 0      (Address | 00000001)  Report range command
*/

/*
 * I2C Range respose  (in CM)
 * Bytes  Description
----------------------------------------------------------------
     0       Range High Byte       MSB -> LSB
     1       Range Low  Byte
*/

/*
 * I2C Change Sensor Address Command
 * Bytes  Description
----------------------------------------------------------------
	 0      Address
	 1      0xAA     Address Unlock1
	 2      0xA5     Address Unlock2
	 3      New Address (must be even number and not be [0, 80, 164, 170]

 */

class I2CXL_EZ : public PIDSource, public LiveWindowSendable
{
public:
	I2CXL_EZ(std::string name, uint8_t address, I2C::Port port=I2C::Port::kOnboard);
	virtual ~I2CXL_EZ();

	void     TakeMeasurement();
	double   GetLastRange();

	bool ChangeAddress(uint8_t toAddress);

	double PIDGet() override;
	void SetPIDSourceType(PIDSourceType pidSource) override;

	void UpdateTable() override;
	void StartLiveWindowMode() override;
	void StopLiveWindowMode() override;
	std::string GetSmartDashboardType() const override;
	void InitTable(std::shared_ptr<ITable> subTable) override;
	std::shared_ptr<ITable> GetTable() const override;


private:

	uint16_t  GetWord();
	uint8_t   GetByte();
	int8_t    Send(uint8_t *data, uint8_t len);


	I2C::Port _port;
	uint8_t   _address;
	I2C       _wire;
	uint8_t   _byte1, _byte2;
	std::string _myName;

	double    _lastReportedRange;

	std::shared_ptr<ITable> m_table;

};

#endif /* SRC_I2CXL_MS_EZ_H_ */
