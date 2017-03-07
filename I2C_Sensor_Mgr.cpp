#include "I2C_Sensor_Mgr.h"
#include "RobotMap.h"


I2C_Sensor_Mgr* I2C_Sensor_Mgr::_singleton = NULL;

I2C_Sensor_Mgr* I2C_Sensor_Mgr::Instance()
{
	if (_singleton == NULL)
	{
		_singleton = new I2C_Sensor_Mgr();
	}
	return _singleton;
}


I2C_Sensor_Mgr::I2C_Sensor_Mgr()
{
	_GearPixy = RobotMap::gearPixy;
	_GearRangeFinder = RobotMap::gearRangeFinder;
	_ShooterPixy = RobotMap::shooterPixy;
//	_ShooterRangeFinder = RobotMap::shooterRangeFinder;
}

I2C_Sensor_Mgr::~I2C_Sensor_Mgr()
{

}

void I2C_Sensor_Mgr::Update_GearPixy()
{
	_GearPixy->Clear_Signatures();
	_GearPixy->GetBlocks(100);
}

void I2C_Sensor_Mgr::Update_ShooterPixy()
{
	_ShooterPixy->Clear_Signatures();
	_ShooterPixy->GetBlocks(100);
}

std::vector<Block> I2C_Sensor_Mgr::Get_GearSignatures()
{
	return _GearPixy->Get_Signatures();
}

std::vector<Block> I2C_Sensor_Mgr::Get_ShooterSignatures()
{
	return _ShooterPixy->Get_Signatures();
}

void I2C_Sensor_Mgr::Update_GearRangeFinder()
{
	_GearRangeFinder->TakeMeasurement();
}

void I2C_Sensor_Mgr::Update_ShooterRangeFinder()
{
	_ShooterRangeFinder->TakeMeasurement();
}

double I2C_Sensor_Mgr::Get_GearRange_cm()
{
	return _GearRangeFinder->GetLastRange();
}

double I2C_Sensor_Mgr::Get_ShooterRange_cm()
{
	return _ShooterRangeFinder->GetLastRange();
}

