#include <iostream>
#include <thread>
#include <kw-lib-all.h>

#define MODE 1
// 0 串口 // 1 udp // 2 tcp

// 线程控制标识
bool capturing = true;

NS_KW_USING

void test_tread_function(std::shared_ptr<SensorControl> obj)
{
	float force[6];
	uint64_t cnt;
	uint64_t removeBytes = 0;
	int res;
	uint64_t totalBytes;

	while (capturing)
	{
		res = obj->GetForceDataF(force, &cnt, &totalBytes);
		if (res == 28)
		{
			printf("count: %llu, fx: %.2f fy: %.2f fz: %.2f mx: %.2f my: %.2f mz: %.2f\n", cnt,
				force[0], force[1], force[2], force[3], force[4], force[5]);
			continue;
		}
		if (res == 1)
		{
			removeBytes++;
			printf("丢失一个字节, 共丢失%llu\n", removeBytes);
			continue;
		}

		if (res == 2)
		{
			removeBytes += 2;
			printf("丢失两个字节, 共丢失%llu\n", removeBytes);
		}
	}

	printf("丢失总字节数: %llu\n", removeBytes);
	printf("传输成功字节数: %llu\n", cnt * 28);
	printf("传输总字节数: %llu\n", totalBytes);
}

int main()
{
#if MODE == 0
	SerialControlCreator uc;

#ifdef WIN32
	uc.serialPortName = "COM6";
#else // 在linux上，加入使用usb转串口，串口名如下。
	uc.serialPortName = "/dev/ttyUSB0";
#endif
	uc.baudRate = 460800;

#elif MODE == 1
	UdpControlCreator uc;

	uc.sensorIp = "192.168.1.101";
	uc.localIp = "192.168.1.100";
	uc.localPort = 8886;

#elif MODE == 2

#endif

	auto control = uc.createIOControler();

	HeadTailProtocolCreator htc;
	auto proto = htc.createProtocol();

	SensorControlCreator scc;
	scc.ioCtrl = control;
	scc.proto = proto;

	auto obj = scc.createSensorControl();
	int hr = obj->StartCapture();
	if (hr != 0)
	{
		printf("start capture faield\n");
		return hr;
	}

	std::thread thread(test_tread_function, obj);

	std::this_thread::sleep_for(std::chrono::seconds(10));

	capturing = false;
	if (thread.joinable())
		thread.join();

	obj->StopCapture();

	printf("停止采集成功\n");
}
