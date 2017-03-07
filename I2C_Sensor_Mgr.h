#ifndef I2CSENSORMGR_H
#define I2CSENSORMGR_H
#include "WPILib.h"
#include "Pixy.h"
#include "I2CXL_MS_EZ.h"
#include "RobotMap.h"



class I2C_Sensor_Mgr
{
public:
static I2C_Sensor_Mgr* Instance();

void Update_GearPixy();
void Update_ShooterPixy();

std::vector<Block> Get_GearSignatures();
std::vector<Block> Get_ShooterSignatures();


void Update_GearRangeFinder();
void Update_ShooterRangeFinder();

double Get_GearRange_cm();
double Get_ShooterRange_cm();

private:
	I2C_Sensor_Mgr();
	~I2C_Sensor_Mgr();


static I2C_Sensor_Mgr* _singleton;

std::shared_ptr<Pixy> _GearPixy;
std::shared_ptr<I2CXL_EZ> _GearRangeFinder;

std::shared_ptr<Pixy> _ShooterPixy;
std::shared_ptr<I2CXL_EZ> _ShooterRangeFinder;

};

#endif
