#pragma once
#include "../io/kw-io.h"
#include "../protocol/kw-protocol.h"

NS_KW_BEGIN

class SensorControl
{
public:

	virtual int StartCapture() = 0;

	virtual int GetForceDataF(float *data, uint64_t *frameNum, size_t *totalBytes) = 0;

	virtual int GetCurrentForceData(float *data) = 0;

	virtual int StopCapture() = 0;
};

class SensorControlCreator
{
public:
	virtual std::shared_ptr<SensorControl> createSensorControl();
	std::shared_ptr<IOControl> ioCtrl;
	std::shared_ptr<Protocol> proto;
};

NS_KW_END