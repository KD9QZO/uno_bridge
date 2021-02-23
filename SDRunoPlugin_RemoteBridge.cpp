#include <sstream>
#include <unoevent.h>
#include <iunoplugincontroller.h>
#include <vector>
#include <string>
#include <sstream>
#include <chrono>
#include <Windows.h>

#include "SDRunoPlugin_RemoteBridge.h"
#include "SDRunoPlugin_RemoteBridgeUi.h"

const char* PreFormatString(const char * zcFormat, ...) 
{
	// initialize use of the variable argument array
	va_list vaArgs;
	va_start(vaArgs, zcFormat);

	// reliably acquire the size
	// from a copy of the variable argument array
	// and a functionally reliable call to mock the formatting
	va_list vaArgsCopy;
	va_copy(vaArgsCopy, vaArgs);
	const int iLen = std::vsnprintf(NULL, 0, zcFormat, vaArgsCopy);
	va_end(vaArgsCopy);

	// return a formatted string without risking memory mismanagement
	// and without assuming any compiler or platform specific behavior
	std::vector<char> zc(iLen + 1);
	std::vsnprintf(zc.data(), zc.size(), zcFormat, vaArgs);
	va_end(vaArgs);
	std::string strText(zc.data(), iLen);

	const char* c = strText.c_str();
	return c;
}

void DbgMsg(const char * zcFormat, ...)
{
	// initialize use of the variable argument array
	va_list vaArgs;
	va_start(vaArgs, zcFormat);

	// reliably acquire the size
	// from a copy of the variable argument array
	// and a functionally reliable call to mock the formatting
	va_list vaArgsCopy;
	va_copy(vaArgsCopy, vaArgs);
	const int iLen = std::vsnprintf(NULL, 0, zcFormat, vaArgsCopy);
	va_end(vaArgsCopy);

	// return a formatted string without risking memory mismanagement
	// and without assuming any compiler or platform specific behavior
	std::vector<char> zc(iLen + 1);
	std::vsnprintf(zc.data(), zc.size(), zcFormat, vaArgs);
	va_end(vaArgs);
	std::string strText(zc.data(), iLen);

	OutputDebugStringA(strText.c_str());
}

SDRunoPlugin_RemoteBridge::SDRunoPlugin_RemoteBridge(IUnoPluginController& controller) :
	IUnoPlugin(controller),
	m_form(*this, controller),
	m_worker(nullptr),
	m_started(false)

{
}

SDRunoPlugin_RemoteBridge::~SDRunoPlugin_RemoteBridge()
{	
}

void SDRunoPlugin_RemoteBridge::HandleEvent(const UnoEvent& ev)
{
	m_form.HandleEvent(ev);	
}

void SDRunoPlugin_RemoteBridge::WorkerFunction()
{
	
	int cycles = 0;
	byte crc;
	if (isConnected) 
	{
#ifdef DEBUG
		OutputDebugStringA("Bridge worker is connected to port \r\n");
#endif
		while (m_started) {
			cycles ++;

			if (isConnected)
			{
				if (Serial.peekReceiver())
				{
					error = "receiver peeked successfully";
					char buffer[300];
					Serial.readString(buffer, '\n', 300, 0);
					//Serial.readBytes(buffer, 300, 200, 200);
					sscanf(buffer, 
						"%hu %lu %lu %hu %lu", 
						&state.DT, 
						&state.VFOFreq, 
						&state.CFreq, 
						&state.FB, 
						&state.fingerprint
					);
					
					crc = Serial.crc8((byte*)&state, sizeof(state)); // ������� crc ������� ���������
					
					if (crc == state.fingerprint) {
#ifdef DEBUG
						OutputDebugStringA("CRC Check passed \r\n");						
#endif //DEBUG
					} else {
						// ������ ����������
#ifdef DEBUG
						OutputDebugStringA("CRC Check failed \r\n");
#endif // DEBUG
					}
				}
				else
				{
					error = "Receiver not peeked";
				}
			}
#ifdef DEBUG
			DbgMsg("DT: %hu \t VFO: %lu \t CFreq: %lu \t FB: %hu \t CRC: %lu \t CCRC: %lu\r\n ", 
				state.DT, state.VFOFreq, state.CFreq, state.FB, state.fingerprint, crc);
			//m_controller.SetVfoFrequency(0, state.VFOFreq);
			m_controller.SetCenterFrequency(0, state.CFreq);
			int vrx_num = m_controller.GetVRXCount();
			DbgMsg("VRX Count %d", vrx_num);
			Sleep(2000);
#endif // DEBUG

			//@todo: re-work me. 
			//Serial.writeString(PreFormatString(
			//	"DT: %hu \t VFO: %lu \t CFreq: %lu \t FB: %hu \t CRC: %lu\r\n ",
			//	state.DT, state.VFOFreq, state.CFreq, state.FB, state.fingerprint
			//));
		}
	}
	else
	{
#ifdef DEBUG
		OutputDebugStringA("Bridge is not connected to port \r\n");
#endif
	}
	
	/*


	error = "Dummy";*/
}

void SDRunoPlugin_RemoteBridge::StartBridge(std::string addr)
{
	std::string error = "default";
	
#ifdef DEBUG
	OutputDebugStringA("Bridge is going to start core \r\n");
#endif
	
	std::lock_guard<std::mutex> l(m_lock);
	if (m_started)
	{
		return;
	}

	m_started = true;

	std::string portStub = "\\\\.\\" + addr;
	if ((int)Serial.openDevice(portStub.c_str(), 9600) != (int)1)
	{
		isConnected = false;
		error = "Error while opening port";
	}
	else {
		isConnected = true;
	}

	Sleep(100);
	
	if (isConnected) {
		m_worker = new std::thread(&SDRunoPlugin_RemoteBridge::WorkerFunction, this);
	}
	
#ifdef DEBUG
	OutputDebugStringA(error.c_str());
#endif


}

void SDRunoPlugin_RemoteBridge::StopBridge() 
{
	//Close port
#ifdef DEBUG
	//char out[MAXBUF];
	//sprintf(out, "SDRuno in %s", buffer);
	//OutputDebugStringA(out);
	OutputDebugStringA("Bridge is going to stop core");
#endif

	if (!m_started)
	{
		return;
	}

	Serial.closeDevice();
	m_started = false;
	m_worker->join();
	delete m_worker;
	m_worker = nullptr;
}

void SDRunoPlugin_RemoteBridge::UpdateSampleRate()
{
	sampleRate = (int)m_controller.GetSampleRate(0);
}

//@todo: process me
/*
Lots of handlers for events
*/