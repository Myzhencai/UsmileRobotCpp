#ifndef kw_io_h
#define kw_io_h

#include "../kw-common.h"

NS_KW_BEGIN

/* Config */
struct IOConfCommon {
	size_t readBuffLength = 32;
	size_t writeBuffLength= 32;
	std::chrono::seconds timeOut{3};
	std::chrono::seconds retryInterval{1};
};

/* IO Control */
class IOControl
{
public:
	virtual bool isDeviceOpen() const = 0;
	virtual KWException openDevice() = 0;
	virtual KWException closeDevice() = 0;
	virtual KWErrorCode kwrite( const uint8_t *buff, size_t buffLength) = 0;
	virtual KWErrorCode kread(uint8_t *buff, size_t buffLength, size_t &readLength) = 0;
};

/* IO Creators */
class IOControlCreator
{
public:
	virtual std::shared_ptr<IOControl> createIOControler() = 0;
	IOConfCommon confCommon;
};

class SerialControlCreator final : public IOControlCreator
{
public:
	virtual std::shared_ptr<IOControl> createIOControler() override;
	std::string serialPortName;
	uint32_t baudRate;
};

class UdpControlCreator final : public IOControlCreator
{
public:
	virtual std::shared_ptr<IOControl> createIOControler() override;

	std::string sensorIp;
	std::string localIp;
	uint16_t localPort;
};

class TcpControlCreator final : public IOControlCreator
{
public:
	virtual std::shared_ptr<IOControl> createIOControler() override;

	std::string sensorIp;
	std::string localIp;
	uint16_t localPort;
};

NS_KW_END

#endif