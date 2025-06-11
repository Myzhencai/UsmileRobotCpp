#ifndef kw_protocol_h
#define kw_protocol_h

#include "../kw-common.h"

NS_KW_BEGIN

enum class SensorOrd
{
	NONE_ORD = 0,
	START_CAPTURE_ONCE = 1,
	START_CAPTURE_MUTI = 2,
	STOP_CAPTURE = 3
};



class Protocol
{
public:
    virtual int encode(SensorOrd cmd, uint8_t *outputCmd, size_t* outputLen) = 0;
    virtual int decode(const uint8_t *inputData, size_t inputLen, float *outputData) = 0;
};

class ProtocolCreator
{
public:
	virtual std::shared_ptr<Protocol> createProtocol() const = 0;
};

class  HeadTailProtocolCreator final : public ProtocolCreator
{
public:
	virtual std::shared_ptr<Protocol> createProtocol() const;
};

NS_KW_END

#endif
