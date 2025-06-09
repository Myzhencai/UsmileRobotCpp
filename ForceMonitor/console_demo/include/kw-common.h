#ifndef KW_DEF_H
#define KW_DEF_H

#include <stddef.h>
#include <thread>
#include <iostream>
#include <queue>
#include <vector>
#include <condition_variable>
#include <mutex>
#include <stdint.h>
#include <memory>
#include <string.h>
#include <string>



#define NS_KW_BEGIN namespace kw {
#define NS_KW_END }
#define NS_KW_USING using namespace kw;
#define NW_KW_MEMBER kw::

NS_KW_BEGIN

enum class KWErrorCode {
	NONE = 0,

	TIME_OUT,

	ASSERT_FALSE,
	UNKNOWN,
	RUNTIME_ERROR,

	IO_OPEN_FAILED,
	IO_READ_FAILED,
	IO_WRITE_FAILED,

	INVALID_ARGUMENTS,
	INVALID_INPUT,

	OUT_OF_RANGE,
};

/* Message or Error */
class KWMessage : public std::exception {
public:
	typedef uint16_t CodeType;

	KWMessage(CodeType code, std::string &&msg) : _code{ code }, _message{ std::move(msg) } {}
	KWMessage(KWErrorCode code, std::string &&msg) : KWMessage{ (CodeType)code, std::move(msg) } {}
	explicit KWMessage(std::string &&msg) : KWMessage(0, std::move(msg)) {}
	KWMessage() {}

    virtual const char *what() const noexcept { return _message.c_str(); }
	KWErrorCode errorCode() const { return (KWErrorCode)code(); }
	CodeType code() const { return _code; }
	bool hasError() const { return _code > 0; }

private:
	const CodeType _code = 0;
	const std::string _message;
};

typedef KWMessage KWException;

NS_KW_END

#endif
