/*
 * Pixy.cpp
 *
 *  Created on: Feb 4, 2017
 *      Author: mdbal
 */

#include "Pixy.h"
#include "Timer.h"
#include <sstream>

void Block::print()
{
    int i, j;
    char buf[128], sig[6], d;
    bool flag;
    if (signature>PIXY_MAX_SIGNATURE) // color code! (CC)
    {
        // convert signature number to an octal string
        for (i=12, j=0, flag=false; i>=0; i-=3)
        {
            d = (signature>>i)&0x07;
            if (d>0 && !flag) flag = true;
            if (flag) sig[j++] = d + '0';
        }
        sig[j] = '\0';

        sprintf(buf, "CC block! sig: %s (%d decimal) x: %d y: %d width: %d height: %d angle %d AR %f Xoff %d Yoff %d\n", sig, signature, x, y, width, height, angle, aspectRatio, x_deviation, y_deviation);
    } else
    {
        sprintf(buf, "sig: %d x: %d y: %d width: %d height: %d AR %f Xoff %d Yoff %d \n", signature, x, y, width, height, aspectRatio, x_deviation, y_deviation);
    }
    printf(buf);
}


Pixy::Pixy(std::string name, uint8_t address, enum frc::I2C::Port port) : Wire(port, address)
{
	_address = address;
	_port = port;

	skipStart = false;
	blockCount = 0;
	signatures.reserve(PIXY_INITIAL_ARRAYSIZE);
	byte1 = 0;
	byte2 = 0;
	word = 0;
	blockType = UNDEFINED;
	x_offset = 0;
	y_offset = 0;

	std::stringstream ss;
	ss << "Pixy_" << name;
	ss >> _myName;

	LiveWindow::GetInstance()->AddSensor(_myName, port, this);
}

Pixy::~Pixy()
{
	signatures.clear();
}

void Pixy::Set_AlignmentOffset(int x, int y)
{
	x_offset = x;
	y_offset = y;
}

std::vector<Block> Pixy::Get_Signatures()
{
	return signatures;
}

void Pixy::Clear_Signatures()
{
	signatures.clear();
}


uint16_t Pixy::GetWord()
{

	Wire.ReadOnly(1,&byte1);
	Wire.ReadOnly(1,&byte2);

    uint16_t w = (byte2 << 8) + byte1;
    return w;
}

uint8_t Pixy::GetByte()
{
	Wire.ReadOnly(1,&byte1);
	return byte1;
}

int8_t Pixy::Send(uint8_t *data, uint8_t len)
{
    Wire.WriteBulk(data, len);
    return len;
}

bool Pixy::GetStart()
{
    uint16_t w, lastw;

    lastw = 0xffff;

    while(true)
    {
        w = GetWord();
        if (w==0 && lastw==0)
        {
            frc::Timer *delay = new frc::Timer();
            delay->Start();
            while (!delay->HasPeriodPassed(0.00005))
            {
                // Wait 50 microseconds
            }

            return false;
        } else if (w==PIXY_START_WORD && lastw==PIXY_START_WORD)
        {
             blockType = NORMAL_BLOCK;
             return true;
        } else if (w==PIXY_START_WORD_CC && lastw==PIXY_START_WORD)
        {
            blockType = CC_BLOCK;
            return true;
        } else if (w==PIXY_START_WORDX)
        {
            printf("reorder");
            GetByte(); // resync
        }
        lastw = w;
    }
}

void Pixy::Resize()
{
	int i = signatures.capacity();
	signatures.resize(i + PIXY_INITIAL_ARRAYSIZE);
}

uint16_t Pixy::GetBlocks(uint16_t maxBlocks)
{
     uint8_t i = 0;
     uint16_t w = 0;
     uint16_t checksum = 0;
     uint16_t sum = 0;
     Block block;

     if (!skipStart)
     {
          if (GetStart()==false)
          return 0;
     } else
     {
          skipStart = false;
     }

     for(int blockCount=0; blockCount<maxBlocks && blockCount<PIXY_MAXIMUM_ARRAYSIZE;)
     {
          checksum = GetWord();
          if (checksum==PIXY_START_WORD) // we've reached the beginning of the next frame
          {
              skipStart = true;
              blockType = NORMAL_BLOCK;
              return blockCount;
          } else if (checksum==PIXY_START_WORD_CC)
          {
              skipStart = true;
              blockType = CC_BLOCK;
              return blockCount;
          } else if (checksum==0)
          {
              return blockCount;
          }

          if (blockCount > PIXY_INITIAL_ARRAYSIZE) Resize();

          for (i = 0, sum = 0; i < 6; i++)
          {
              if (blockType==NORMAL_BLOCK && i>=5) // skip
              {
                   block.angle = 0;
                   break;
              }
              w = GetWord();
              sum += w;
              *((uint16_t *)&block + i) = w;
          }

          if (checksum == sum)
          {
        	  blockCount++;
        	  block.aspectRatio = (float) block.x / (float) block.y;
        	  block.x_top = block.x - (block.height / 2);
        	  block.x_bottom = block.x + (block.height / 2);
        	  block.y_left = block.y - (block.width / 2);
        	  block.y_right = block.y + (block.width / 2);
        	  block.x_deviation = block.x - PIXY_CENT_X;
        	  block.y_deviation = block.y - PIXY_CENT_Y;
        	  signatures.push_back(block);
          } else
          {
              printf("cs error");
          }

          w = GetWord();
          if (w == PIXY_START_WORD)
          {
              blockType = NORMAL_BLOCK;
          } else if (w == PIXY_START_WORD_CC)
          {
              blockType = CC_BLOCK;
          } else
          {
              return blockCount;
          }
     }
     return 0;
}

int8_t Pixy::SetServos(uint16_t s0, uint16_t s1)
{
     uint8_t outBuf[6];

     outBuf[0] = 0x00;
     outBuf[1] = 0xff;
     *(uint16_t *)(outBuf + 2) = s0;
     *(uint16_t *)(outBuf + 4) = s1;

     return Send(outBuf, 6);
}

int8_t Pixy::SetBrightness(uint8_t brightness)
{
     uint8_t outBuf[3];

     outBuf[0] = 0x00;
     outBuf[1] = 0xfe;
     outBuf[2] = brightness;

     return Send(outBuf, 3);
}

int8_t Pixy::SetLED(uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t outBuf[5];

    outBuf[0] = 0x00;
    outBuf[1] = 0xfd;
    outBuf[2] = r;
    outBuf[3] = g;
    outBuf[4] = b;

    return Send(outBuf, 5);
}

void Pixy::UpdateTable() {
  if (m_table != nullptr) {
	  Clear_Signatures();
	  uint16_t inView = GetBlocks();
	  m_table->PutNumber("ObjectsInView", inView);
/*	  std::vector<Block>::iterator itt = signatures.begin();

	  for (int i = 0; i < 6; i++)
	  {
		  std::stringstream ss;
		  std::string entry;
		  ss << i;
		  ss >> entry;
		  std::vector<double> temp;

		  if (itt != signatures.end())
		  {
			  itt->print();
			  temp.push_back(1.0);
			  temp.push_back((double) itt->x);
			  temp.push_back((double) itt->y);
			  temp.push_back((double) itt->width);
			  temp.push_back((double) itt->height);
			  temp.push_back((double) itt->x_top);
			  temp.push_back((double) itt->x_bottom);
			  temp.push_back((double) itt->y_left);
			  temp.push_back((double) itt->y_right);
			  temp.push_back((double) itt->x_deviation);
			  temp.push_back((double) itt->y_deviation);
			  itt++;
		  } else
		  {
			  temp.push_back(0.0);
			  temp.push_back(0.0);
			  temp.push_back(0.0);
			  temp.push_back(0.0);
			  temp.push_back(0.0);
			  temp.push_back(0.0);
			  temp.push_back(0.0);
			  temp.push_back(0.0);
			  temp.push_back(0.0);
			  temp.push_back(0.0);
			  temp.push_back(0.0);
		  }
		  llvm::ArrayRef<double> arr(temp);

		  m_table->PutNumberArray(entry, arr);
	  }*/
  }
}

void Pixy::StartLiveWindowMode() {}

void Pixy::StopLiveWindowMode() {}

std::string Pixy::GetSmartDashboardType() const { return "DigitalInput"; }

void Pixy::InitTable(std::shared_ptr<ITable> subTable) {
  m_table = subTable;
  UpdateTable();
}

std::shared_ptr<ITable> Pixy::GetTable() const { return m_table; }

